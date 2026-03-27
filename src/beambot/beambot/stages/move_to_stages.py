"""MoveTo stages - Python equivalent of move_to_stages.hpp/cpp.

Handles MoveTo operations:
- Relative moves (direction + distance)
- Cartesian pose targets (xyz + optional orientation)
- Target-based moves (joint poses from JSON or named SRDF states)
"""

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.task_constructor import core, stages
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from beambot.stages.base_stages import (
    BaseStages, joints_from_degrees, parse_constraints, apply_constraints,
)

# Known gripper tip frames and their TF parent for detection.
# Checked in order — first match wins.
_GRIPPER_TIP_FRAMES = [
    "epick_tip",           # Robotiq ePick vacuum gripper
    "robotiq_hande_end",   # Robotiq Hand-E adaptive gripper
    "2fg7_tip",            # OnRobot 2FG7 parallel gripper
    "pipette_tip_link",    # Pipettor nozzle tip
]


class MoveToStages(BaseStages):
    """Handles MoveTo action: relative moves, Cartesian poses, joint poses, named states."""

    def __init__(self, rclpy_node, arm_group: str = "", ik_frame: str = ""):
        super().__init__(rclpy_node, arm_group, ik_frame)
        # Initialize TF eagerly so the buffer is populated by the time
        # any Cartesian target or IK frame detection is needed
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.rclpy_node)

    def _ensure_tf(self):
        """No-op kept for backward compatibility."""
        pass

    def _get_current_yaw(self, frame: str = "flange") -> float:
        """Get the current yaw of a robot frame in base_link.

        Used as the default yaw for 3-value cartesian_target (XYZ only)
        so the robot maintains its current wrist orientation.

        Args:
            frame: Robot frame to query (should match the IK frame, e.g.
                   "epick_tip", "robotiq_hande_end", or "flange").

        Returns:
            Current yaw in radians, or 0.0 if TF lookup fails.
        """
        self._ensure_tf()
        try:
            t = self._tf_buffer.lookup_transform(
                "base_link", frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            q = [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            ]
            _, _, yaw = euler_from_quaternion(q)
            return yaw
        except Exception as e:
            self.logger.warn(f"Failed to get current yaw from {frame}: {e}, defaulting to 0")
            return 0.0

    def _detect_gripper_ik_frame(self) -> str:
        """Auto-detect gripper tip frame from TF, like vision_stages.py.

        Checks for known gripper tip frames in TF. If found, uses that as
        the IK frame so Cartesian targets place the gripper TIP at the target
        position, not the flange.

        Returns:
            Frame name for IK (e.g., "epick_tip", "robotiq_hande_end", or "flange")
        """
        self._ensure_tf()
        for frame in _GRIPPER_TIP_FRAMES:
            try:
                if self._tf_buffer.can_transform(
                    "base_link", frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0),
                ):
                    self.logger.info(f"Detected gripper tip frame: {frame}")
                    return frame
            except Exception:
                continue
        self.logger.info("No gripper tip frame detected, using flange")
        return "flange"

    def add_to_task(self, task: core.Task, goal, planner=None) -> 'Optional[str]':
        """Add MoveTo stages to an existing MTC task.

        This method adds stages without creating or executing the task,
        enabling batch execution of multiple tasks.

        Args:
            task: Existing MTC Task to add stages to
            goal: MoveToAction.Goal with fields:
                - target: Pose name, SRDF state, or empty for relative moves
                - planning_type: "joint" or "cartesian"
                - direction: Direction for relative moves
                - distance: Distance in meters for relative moves
                - cartesian_target: [x,y,z] or [x,y,z,r,p,y] in meters + degrees
                - frame_id: Reference frame for cartesian_target
                - poses_json: JSON string with pose definitions
            planner: Optional planner instance (creates default based on
                     planning_type if None)

        Returns:
            None if stages were added successfully, error string on failure
        """
        planning_type = goal.planning_type if goal.planning_type else ""
        use_fallback = planning_type in ("", "auto") and planner is None

        # Select single planner when not using fallbacks
        if not use_fallback and planner is None:
            if planning_type == "cartesian":
                planner = self.make_cartesian_planner()
                self.logger.info("Using Cartesian planner")
            elif planning_type == "pilz":
                planner = self.make_pilz_planner("LIN")
                self.logger.info("Using Pilz LIN planner")
            elif planning_type == "pilz_ptp":
                planner = self.make_pilz_planner("PTP")
                self.logger.info("Using Pilz PTP planner")
            else:
                planner = self.make_pipeline_planner()
                self.logger.info("Using pipeline planner (OMPL)")

        if use_fallback:
            self.logger.info("Using fallback planning (auto)")

        # Parse optional constraints
        constraints = parse_constraints(
            json.loads(goal.constraints_json) if goal.constraints_json else None
        )
        if constraints is not None:
            self.logger.info("Path constraints active for this move")

        # Case 1: Relative move (direction + distance)
        if goal.direction and goal.distance != 0.0:
            label = f"move_{goal.direction}_{goal.distance:.3f}m"
            stage = self.create_relative_move_stage(
                label, goal.direction, goal.distance,
                planner=planner, constraints=constraints
            )
            task.add(stage)
            self.logger.info(
                f"Planning relative move: {goal.direction} {goal.distance}m"
            )
            return None

        # Case 2: Cartesian pose target ([x,y,z] or [x,y,z,r,p,y])
        elif len(goal.cartesian_target) >= 3:
            # Auto-detect gripper tip frame first — needed for both IK and
            # default yaw lookup (different frames have different yaw values)
            active_ik_frame = self._detect_gripper_ik_frame()

            pose = PoseStamped()
            pose.header.frame_id = goal.frame_id if goal.frame_id else "base_link"
            pose.pose.position.x = goal.cartesian_target[0]
            pose.pose.position.y = goal.cartesian_target[1]
            pose.pose.position.z = goal.cartesian_target[2]

            if len(goal.cartesian_target) >= 6:
                r = math.radians(goal.cartesian_target[3])
                p = math.radians(goal.cartesian_target[4])
                y = math.radians(goal.cartesian_target[5])
                q = quaternion_from_euler(r, p, y)
            else:
                # Default: straight-down with current IK frame yaw
                # Preserves the robot's current yaw to avoid unreachable orientations
                current_yaw = self._get_current_yaw(active_ik_frame)
                q = quaternion_from_euler(math.pi, 0.0, current_yaw)
                self.logger.info(f"Using current yaw from {active_ik_frame}: {math.degrees(current_yaw):.1f}°")

            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            if use_fallback:
                # Fallback chain: Pilz LIN → CartesianPath
                fb = core.Fallbacks("move_to_cartesian")
                for planner_fn, suffix in [
                    (lambda: self.make_pilz_planner("LIN"), "Pilz LIN"),
                    (self.make_cartesian_planner, "CartesianPath"),
                ]:
                    move_stage = stages.MoveTo(
                        f"move_to_cartesian [{suffix}]", planner_fn()
                    )
                    move_stage.group = self.arm_group
                    ik_frame_pose = PoseStamped()
                    ik_frame_pose.header.frame_id = active_ik_frame
                    move_stage.ik_frame = ik_frame_pose
                    move_stage.setGoal(pose)
                    apply_constraints(move_stage, constraints)
                    fb.add(move_stage)
                task.add(fb)
            else:
                # Default to Cartesian planner for Cartesian targets
                if planner is None:
                    planner = self.make_cartesian_planner()

                move_stage = stages.MoveTo("move_to_cartesian", planner)
                move_stage.group = self.arm_group
                ik_frame_pose = PoseStamped()
                ik_frame_pose.header.frame_id = active_ik_frame
                move_stage.ik_frame = ik_frame_pose
                move_stage.setGoal(pose)
                apply_constraints(move_stage, constraints)
                task.add(move_stage)
            self.logger.info(
                f"Planning Cartesian move to "
                f"[{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, "
                f"{pose.pose.position.z:.3f}] in {pose.header.frame_id} "
                f"(ik_frame: {active_ik_frame})"
            )
            return None

        # Case 3: Target-based move (joint pose or SRDF state)
        elif goal.target:
            # Poses are optional for MoveTo (might use SRDF named state)
            poses = self.parse_poses(goal.poses_json)
            if poses is None:
                error = f"Failed to parse poses_json for target '{goal.target}'"
                self.logger.error(error)
                return error

            label = f"move_to_{goal.target}"

            if goal.target in poses:
                # Joint pose from poses dict
                stage = self.make_move_to_named_stage(
                    label, goal.target, poses,
                    planner=planner, constraints=constraints
                )
                if not stage:
                    return f"Pose '{goal.target}' not found or invalid"
                task.add(stage)
                self.logger.info(f"Planning move to joint pose: {goal.target}")
            else:
                # SRDF named state
                if use_fallback:
                    # Fallback chain: Pilz PTP → OMPL
                    fb = core.Fallbacks(label)
                    for planner_fn, suffix in [
                        (lambda: self.make_pilz_planner("PTP"), "Pilz PTP"),
                        (self.make_pipeline_planner, "OMPL"),
                    ]:
                        s = stages.MoveTo(f"{label} [{suffix}]", planner_fn())
                        s.group = self.arm_group
                        self._set_ik_frame(s)
                        s.setGoal(goal.target)
                        apply_constraints(s, constraints)
                        fb.add(s)
                    task.add(fb)
                else:
                    move_stage = stages.MoveTo(label, planner)
                    move_stage.group = self.arm_group
                    self._set_ik_frame(move_stage)
                    move_stage.setGoal(goal.target)
                    apply_constraints(move_stage, constraints)
                    task.add(move_stage)
                self.logger.info(f"Planning move to named state: {goal.target}")

            return None

        else:
            error = (
                "No valid move target specified. "
                "Provide (direction + distance), cartesian_target, or target."
            )
            self.logger.error(error)
            return error

    def run(self, goal) -> 'Optional[str]':
        """Execute MoveTo action.

        Args:
            goal: MoveToAction.Goal with fields:
                - target: Pose name, SRDF state, or empty for relative moves
                - planning_type: "joint" or "cartesian"
                - direction: Direction for relative moves
                - distance: Distance in meters for relative moves
                - cartesian_target: [x,y,z] or [x,y,z,r,p,y]
                - frame_id: Reference frame for cartesian_target
                - poses_json: JSON string with pose definitions

        Returns:
            None if successful, error string describing failure otherwise
        """
        task = self.create_task_template("MoveTo Task")

        error = self.add_to_task(task, goal)
        if error is not None:
            return error

        return self.load_plan_execute(task)
