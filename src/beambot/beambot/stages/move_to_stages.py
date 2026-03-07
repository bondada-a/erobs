"""MoveTo stages - Python equivalent of move_to_stages.hpp/cpp.

Handles MoveTo operations:
- Relative moves (direction + distance)
- Cartesian pose targets (xyz + optional orientation)
- Target-based moves (joint poses from JSON or named SRDF states)
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.task_constructor import core, stages
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler

from beambot.stages.base_stages import BaseStages, joints_from_degrees

# Known gripper tip frames and their TF parent for detection.
# Checked in order — first match wins.
_GRIPPER_TIP_FRAMES = [
    "epick_tip",           # OnRobot ePick vacuum gripper
    "robotiq_hande_end",   # Robotiq Hand-E adaptive gripper
    # Add new gripper tip frames here
]


class MoveToStages(BaseStages):
    """Handles MoveTo action: relative moves, Cartesian poses, joint poses, named states."""

    def __init__(self, rclpy_node, arm_group: str = "", ik_frame: str = ""):
        super().__init__(rclpy_node, arm_group, ik_frame)
        self._tf_buffer = None
        self._tf_listener = None

    def _ensure_tf(self):
        """Lazy-initialize TF buffer on first Cartesian target use."""
        if self._tf_buffer is None:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self.rclpy_node)

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

    def add_to_task(self, task: core.Task, goal, planner=None) -> bool:
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
            True if stages were added successfully, False on error
        """
        # Select planner if not provided
        if planner is None:
            planning_type = goal.planning_type if goal.planning_type else "joint"
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

        # Case 1: Relative move (direction + distance)
        if goal.direction and goal.distance != 0.0:
            stage = self.create_relative_move_stage(
                f"move_{goal.direction}_{goal.distance:.3f}m",
                goal.direction,
                goal.distance,
                planner
            )
            task.add(stage)
            self.logger.info(
                f"Planning relative move: {goal.direction} {goal.distance}m"
            )
            return True

        # Case 2: Cartesian pose target ([x,y,z] or [x,y,z,r,p,y])
        elif len(goal.cartesian_target) >= 3:
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
                # Default: straight-down (180° around X)
                q = quaternion_from_euler(math.pi, 0.0, 0.0)

            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            # Default to Cartesian planner for Cartesian targets
            if planner is None:
                planner = self.make_cartesian_planner()

            # Auto-detect gripper tip frame so the target places the gripper
            # tip (not the flange) at the desired position
            active_ik_frame = self._detect_gripper_ik_frame()

            move_stage = stages.MoveTo("move_to_cartesian", planner)
            move_stage.group = self.arm_group
            ik_frame_pose = PoseStamped()
            ik_frame_pose.header.frame_id = active_ik_frame
            move_stage.ik_frame = ik_frame_pose
            move_stage.setGoal(pose)
            task.add(move_stage)
            self.logger.info(
                f"Planning Cartesian move to "
                f"[{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, "
                f"{pose.pose.position.z:.3f}] in {pose.header.frame_id} "
                f"(ik_frame: {active_ik_frame})"
            )
            return True

        # Case 3: Target-based move (joint pose or SRDF state)
        elif goal.target:
            # Poses are optional for MoveTo (might use SRDF named state)
            poses = self.parse_poses(goal.poses_json)
            if poses is None:
                return False

            move_stage = stages.MoveTo(f"move_to_{goal.target}", planner)
            move_stage.group = self.arm_group
            self._set_ik_frame(move_stage)

            # Check if target is a defined joint pose in the JSON
            if goal.target in poses:
                joint_values = poses[goal.target]
                if isinstance(joint_values, list):
                    move_stage.setGoal(joints_from_degrees(joint_values))
                    self.logger.info(f"Planning move to joint pose: {goal.target}")
                else:
                    self.logger.error(
                        f"Invalid pose format for '{goal.target}': "
                        f"expected list, got {type(joint_values)}"
                    )
                    return False
            else:
                # Assume it's a named SRDF state
                move_stage.setGoal(goal.target)
                self.logger.info(f"Planning move to named state: {goal.target}")

            task.add(move_stage)
            return True

        else:
            self.logger.error(
                "No valid move target specified. "
                "Provide (direction + distance), cartesian_target, or target."
            )
            return False

    def run(self, goal) -> bool:
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
            True if successful, False otherwise
        """
        task = self.create_task_template("MoveTo Task")

        if not self.add_to_task(task, goal):
            return False

        return self.load_plan_execute(task)
