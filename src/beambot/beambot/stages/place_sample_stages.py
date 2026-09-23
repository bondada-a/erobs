"""Sample placement using named joint poses or vision-guided targets."""

import json

from geometry_msgs.msg import PoseStamped
from moveit.task_constructor import core, stages

from beambot.core.task_script import flange_offset_error
from beambot.stages.base_stages import (
    BaseStages,
    parse_constraints,
    apply_constraints,
)
from beambot.vision.vision_engine import VisionEngine


class PlaceSampleStages(BaseStages):
    def __init__(
        self,
        rclpy_node,
        arm_group: str = "",
        ik_frame: str = "",
        camera_type: str = None,
        camera_frame: str = None,
        marker_dictionary: str = None,
    ):
        super().__init__(rclpy_node, arm_group, ik_frame=ik_frame)
        self._vision = None
        self._vision_kwargs = {
            "arm_group": arm_group,
            "ik_frame": ik_frame,
            "camera_type": camera_type,
            "camera_frame": camera_frame,
            "marker_dictionary": marker_dictionary,
        }
        self.last_detected_pose: PoseStamped | None = None
        self.logger.info("PlaceSampleStages initialized")

    def run(self, goal) -> str | None:
        """Execute a place; return None on completion or an error string."""
        self.last_detected_pose = None

        error = flange_offset_error(
            direction=goal.offset_direction,
            distance=goal.offset_distance,
        )
        if error:
            return error

        poses = self.parse_poses(goal.poses_json)
        if poses is None:
            return "Failed to parse poses_json for place_sample"

        try:
            gripper_states = (
                json.loads(goal.gripper_states_json) if goal.gripper_states_json else {}
            )
        except json.JSONDecodeError as e:
            return f"Invalid gripper_states_json: {e}"

        try:
            constraints = parse_constraints(
                json.loads(goal.constraints_json) if goal.constraints_json else None
            )
        except json.JSONDecodeError as e:
            return f"Invalid constraints_json: {e}"

        if goal.use_vision:
            error = self._run_vision(goal, poses, gripper_states, constraints)
        else:
            error = self._run_hardcoded(goal, poses, gripper_states, constraints)

        if error is not None:
            return error

        self.logger.info("Place complete")
        return None

    def _run_vision(
        self, goal, poses: dict, gripper_states: dict, constraints
    ) -> str | None:
        """Place at a detected marker and retreat to the scan pose."""
        if self._vision is None:
            try:
                self._vision = VisionEngine(self.rclpy_node, **self._vision_kwargs)
            except Exception as e:
                return f"Vision initialization failed: {e}"

        self.logger.info(
            f"Vision place: detection={goal.detection_type or 'marker'}, "
            f"tag_id={goal.tag_id}, scan_pose={goal.scan_pose}"
        )

        # Leave the gripper unchanged while moving to the scan pose.
        task = self.create_task_template("Position for Place")

        scan_stage = self.make_move_to_named_stage(
            "scan position",
            goal.scan_pose,
            poses,
            constraints=constraints,
        )
        if not scan_stage:
            return f"Pose '{goal.scan_pose}' not found or invalid (scan position)"
        task.add(scan_stage)

        error = self.load_plan_execute(task)
        if error:
            return f"Position for place failed: {error}"

        # This path always uses marker detection.
        detection_type = goal.detection_type or "marker"
        target_pose = self._vision.detect_and_transform_tag(goal.tag_id, timeout=10.0)

        if target_pose is None:
            return (
                f"DETECTION_FAILED: {detection_type} detection failed "
                f"(tag_id={goal.tag_id})"
            )

        ik_frame_override = getattr(goal, "ik_frame", "") or ""
        approach, active_ik_frame = self._vision.compute_approach_pose(
            target_pose,
            goal.z_offset,
            marker_offset_x=goal.marker_offset_x,
            marker_offset_y=goal.marker_offset_y,
            marker_offset_z=goal.marker_offset_z,
            ik_frame_override=ik_frame_override,
        )

        offset_direction = getattr(goal, "offset_direction", "") or ""
        offset_distance = getattr(goal, "offset_distance", 0.0)
        if offset_direction and offset_distance > 0:
            approach = self._vision._apply_flange_offset(
                approach,
                offset_direction,
                offset_distance,
            )

        # Results expose the offset approach pose, not the raw detection.
        self.last_detected_pose = approach

        joint_goal = self._vision.compute_deterministic_ik(approach, active_ik_frame)

        # Approach, release and retreat share one task.
        task = self.create_task_template("Place")
        gripper_planner = self.make_joint_interpolation_planner()

        # Prefer a joint-space PTP goal; fall back to a LIN pose goal.
        approach_fb = core.Fallbacks("approach")
        if joint_goal is not None:
            ptp_stage = stages.MoveTo("approach [PTP]", self.make_pilz_planner("PTP"))
            ptp_stage.group = self.arm_group
            self._set_ik_frame(ptp_stage)
            ptp_stage.setGoal(joint_goal)
            apply_constraints(ptp_stage, constraints)
            approach_fb.add(ptp_stage)

        lin_stage = stages.MoveTo("approach [LIN]", self.make_pilz_planner("LIN"))
        lin_stage.group = self.arm_group
        ik_frame_pose = PoseStamped()
        ik_frame_pose.header.frame_id = active_ik_frame
        lin_stage.ik_frame = ik_frame_pose
        lin_stage.setGoal(approach)
        apply_constraints(lin_stage, constraints)
        approach_fb.add(lin_stage)
        task.add(approach_fb)

        release_stage = self.make_gripper_stage(
            "open gripper",
            gripper_planner,
            goal.gripper_group,
            gripper_states.get("release", ""),
        )
        if release_stage:
            task.add(release_stage)

        retreat_stage = self.make_move_to_named_stage(
            "retreat",
            goal.scan_pose,
            poses,
            constraints=constraints,
        )
        if not retreat_stage:
            return f"Pose '{goal.scan_pose}' not found or invalid (retreat)"
        task.add(retreat_stage)

        error = self.load_plan_execute(task)
        if error:
            return f"Place execution failed: {error}"

        return None

    def _run_hardcoded(
        self, goal, poses: dict, gripper_states: dict, constraints
    ) -> str | None:
        """Place using named approach/target poses, then retreat to approach."""
        self.logger.info(
            f"Hardcoded place: approach={goal.approach_pose}, target={goal.target_pose}"
        )

        task = self.create_task_template("Place Sample")
        gripper_planner = self.make_joint_interpolation_planner()

        stage = self.make_move_to_named_stage(
            "place approach",
            goal.approach_pose,
            poses,
            constraints=constraints,
        )
        if not stage:
            return f"Pose '{goal.approach_pose}' not found or invalid (approach)"
        task.add(stage)

        stage = self.make_move_to_named_stage(
            "place target",
            goal.target_pose,
            poses,
            constraints=constraints,
        )
        if not stage:
            return f"Pose '{goal.target_pose}' not found or invalid (target)"
        task.add(stage)

        release_stage = self.make_gripper_stage(
            "open gripper",
            gripper_planner,
            goal.gripper_group,
            gripper_states.get("release", ""),
        )
        if release_stage:
            task.add(release_stage)

        stage = self.make_move_to_named_stage(
            "place retreat",
            goal.approach_pose,
            poses,
            constraints=constraints,
        )
        if not stage:
            return f"Pose '{goal.approach_pose}' not found or invalid (retreat)"
        task.add(stage)

        error = self.load_plan_execute(task)
        if error:
            return f"Place execution failed: {error}"

        return None
