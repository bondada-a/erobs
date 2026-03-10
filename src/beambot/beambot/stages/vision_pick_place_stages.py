"""VisionPickPlaceStages - Vision-guided pick with hardcoded place.

Hybrid pick-and-place operation:
- PICK: Vision-guided (detects marker/circle, computes grasp pose dynamically)
- PLACE: Hardcoded joint poses (reliable, repeatable placement)

Execution sequence (10 stages across 2 MTC tasks):

  Task 1 - Position for Detection:
    1. Open gripper
    2. Move to sample_approach (joint pose - camera sees samples)

  [Runtime: Vision detection - marker or circle]

  Task 2 - Pick and Place:
    3. Move to grasp pose (Cartesian, vision-guided)
    4. Close gripper
    5. Retreat to sample_approach (joint pose)
    6. Move to place_approach (joint pose)
    7. Move to place_target (joint pose)
    8. Open gripper (release)
    9. Retreat to place_approach (joint pose)

Split into 2 tasks because detection requires robot to be in position first.
"""

import json
from typing import Any, Dict, Optional

from geometry_msgs.msg import PoseStamped
from moveit.task_constructor import core, stages

from beambot.stages.base_stages import (
    BaseStages, joints_from_degrees, parse_constraints, apply_constraints,
)
from beambot.stages.vision_stages import VisionStages


class VisionPickPlaceStages(BaseStages):
    """Handles vision-guided pick and hardcoded place operations."""

    # Detection type constants
    DETECTION_MARKER = "marker"
    DETECTION_CIRCLE = "circle"

    def __init__(
        self,
        rclpy_node,
        arm_group: str = "",
        ik_frame: str = "",
        camera_type: str = None,
        camera_frame: str = None,
        marker_dictionary: str = None,
    ):
        """Initialize VisionPickPlaceStages.

        Args:
            rclpy_node: ROS node for service calls and TF
            arm_group: MoveIt planning group for arm
            ik_frame: IK frame for motion planning
            camera_type: Camera type from beamline config (default: "zivid")
            camera_frame: Camera TF frame (default: "zivid_optical_frame")
            marker_dictionary: ArUco dictionary (default: "aruco4x4_50")
        """
        super().__init__(rclpy_node, arm_group, ik_frame=ik_frame)

        # Create VisionStages instance for marker/circle detection
        self._vision = VisionStages(
            rclpy_node,
            arm_group,
            ik_frame,
            camera_type=camera_type,
            camera_frame=camera_frame,
            marker_dictionary=marker_dictionary,
        )

        self.logger.info("VisionPickPlaceStages initialized")

    def run(self, goal) -> 'Optional[str]':
        """Execute vision-guided pick and hardcoded place.

        Args:
            goal: VisionPickPlaceAction.Goal with fields:
                - detection_type: "marker" or "circle"
                - tag_id: ArUco marker ID (for marker detection)
                - z_offset: Height above detected point (default: 0.02m)
                - sample_approach: Joint pose key for scan/approach position
                - place_approach: Joint pose key for place approach
                - place_target: Joint pose key for place
                - gripper_group: MoveIt group name
                - gripper_states_json: JSON dict of gripper states
                - poses_json: JSON with joint pose definitions

        Returns:
            None if successful, error string describing failure otherwise
        """
        self.logger.info(
            f"Vision pick/place: detection={goal.detection_type}, "
            f"tag_id={goal.tag_id}, gripper_group={goal.gripper_group}"
        )

        # Parse poses
        poses = self.parse_poses(goal.poses_json)
        if poses is None:
            return "Failed to parse poses_json for vision_pick_place"

        # Parse gripper states
        try:
            gripper_states = json.loads(goal.gripper_states_json) if goal.gripper_states_json else {}
        except json.JSONDecodeError as e:
            return f"Invalid gripper_states_json: {e}"

        # Get z_offset (default 0.02m = 2cm above detected point)
        z_offset = goal.z_offset if goal.z_offset != 0.0 else 0.02

        # Parse optional path constraints
        constraints = parse_constraints(
            json.loads(goal.constraints_json) if goal.constraints_json else None
        )

        # === TASK 1: Position for Detection ===
        self.logger.info("Task 1/2: Moving to sample_approach for detection...")
        error = self._execute_position_for_detection(goal, poses, gripper_states, constraints)
        if error is not None:
            return f"Failed to position for detection: {error}"

        # === RUNTIME: Vision Detection ===
        self.logger.info(f"Detecting {goal.detection_type}...")
        grasp_pose = self._detect_and_compute_grasp(goal, z_offset)
        if grasp_pose is None:
            detection_type = goal.detection_type or "marker"
            return (
                f"DETECTION_FAILED: Vision detection failed "
                f"(type: {detection_type}, tag_id: {goal.tag_id})"
            )

        # === TASK 2: Pick and Place ===
        self.logger.info("Task 2/2: Executing pick and place...")
        error = self._execute_pick_and_place(goal, poses, gripper_states, grasp_pose, constraints)
        if error is not None:
            return f"Vision pick/place execution failed: {error}"

        self.logger.info("Vision pick and place completed successfully")
        return None

    def _execute_position_for_detection(
        self,
        goal,
        poses: Dict[str, Any],
        gripper_states: Dict[str, str],
        constraints=None
    ) -> 'Optional[str]':
        """Execute Task 1: Open gripper and move to sample_approach.

        Returns:
            None if successful, error string on failure
        """
        task = self.create_task_template("Position for Detection")
        gripper_planner = self.make_joint_interpolation_planner()

        # 1. Open gripper
        open_stage = self.make_gripper_stage(
            "open gripper", gripper_planner, goal.gripper_group, gripper_states.get("release", "")
        )
        if open_stage:
            task.add(open_stage)

        # 2. Move to sample_approach (where camera can see samples)
        stage = self.make_move_to_named_stage(
            "sample approach", goal.sample_approach, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.sample_approach}' not found or invalid (sample approach)"
        task.add(stage)

        return self.load_plan_execute(task)

    def _detect_and_compute_grasp(
        self,
        goal,
        z_offset: float
    ) -> Optional[PoseStamped]:
        """Perform vision detection and compute grasp pose.

        Args:
            goal: Action goal with detection parameters
            z_offset: Height above detected point

        Returns:
            Grasp pose in base_link frame, or None if detection failed
        """
        detection_type = goal.detection_type or self.DETECTION_MARKER

        # Detect based on type
        if detection_type == self.DETECTION_CIRCLE:
            detected_pose = self._vision.detect_and_transform_circle(timeout=10.0)
        else:
            detected_pose = self._vision.detect_and_transform_tag(goal.tag_id, timeout=10.0)

        if detected_pose is None:
            return None

        self.logger.info(
            f"Detected at [{detected_pose.pose.position.x:.3f}, "
            f"{detected_pose.pose.position.y:.3f}, {detected_pose.pose.position.z:.3f}]"
        )

        # Compute grasp pose: detected position + z_offset, pointing straight down
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "base_link"
        grasp_pose.pose.position.x = detected_pose.pose.position.x
        grasp_pose.pose.position.y = detected_pose.pose.position.y
        grasp_pose.pose.position.z = detected_pose.pose.position.z + z_offset

        # Orientation: gripper pointing straight down (180° around Y axis)
        # Quaternion for 180° Y rotation: (0, 1, 0, 0)
        grasp_pose.pose.orientation.x = 0.0
        grasp_pose.pose.orientation.y = 1.0
        grasp_pose.pose.orientation.z = 0.0
        grasp_pose.pose.orientation.w = 0.0

        self.logger.info(
            f"Grasp pose: [{grasp_pose.pose.position.x:.3f}, "
            f"{grasp_pose.pose.position.y:.3f}, {grasp_pose.pose.position.z:.3f}] "
            f"(z_offset: {z_offset:.3f}m)"
        )

        return grasp_pose

    def _execute_pick_and_place(
        self,
        goal,
        poses: Dict[str, Any],
        gripper_states: Dict[str, str],
        grasp_pose: PoseStamped,
        constraints=None
    ) -> 'Optional[str]':
        """Execute Task 2: Pick sequence + Place sequence.

        Stages:
            3. Move to grasp pose (Cartesian)
            4. Close gripper
            5. Retreat to sample_approach
            6. Move to place_approach
            7. Move to place_target
            8. Open gripper
            9. Retreat to place_approach

        Returns:
            None if successful, error string on failure
        """
        task = self.create_task_template("Pick and Place")
        gripper_planner = self.make_joint_interpolation_planner()

        # === PICK SEQUENCE ===

        # 3. Move to grasp pose (fallback: Pilz LIN → CartesianPath)
        grasp_fb = core.Fallbacks("grasp")
        for planner_fn, suffix in [
            (lambda: self.make_pilz_planner("LIN"), "Pilz LIN"),
            (self.make_cartesian_planner, "CartesianPath"),
        ]:
            grasp_stage = stages.MoveTo(f"grasp [{suffix}]", planner_fn())
            grasp_stage.group = self.arm_group
            self._set_ik_frame(grasp_stage)
            grasp_stage.setGoal(grasp_pose)
            apply_constraints(grasp_stage, constraints)
            grasp_fb.add(grasp_stage)
        task.add(grasp_fb)

        # 4. Close gripper
        close_stage = self.make_gripper_stage(
            "close gripper", gripper_planner, goal.gripper_group, gripper_states.get("grasp", "")
        )
        if close_stage:
            task.add(close_stage)

        # 5. Retreat to sample_approach (safe joint pose)
        stage = self.make_move_to_named_stage(
            "pick retreat", goal.sample_approach, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.sample_approach}' not found or invalid (pick retreat)"
        task.add(stage)

        # === PLACE SEQUENCE ===

        # 6. Move to place_approach
        stage = self.make_move_to_named_stage(
            "place approach", goal.place_approach, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.place_approach}' not found or invalid (place approach)"
        task.add(stage)

        # 7. Move to place_target
        stage = self.make_move_to_named_stage(
            "place", goal.place_target, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.place_target}' not found or invalid (place target)"
        task.add(stage)

        # 8. Open gripper (release)
        release_stage = self.make_gripper_stage(
            "release", gripper_planner, goal.gripper_group, gripper_states.get("release", "")
        )
        if release_stage:
            task.add(release_stage)

        # 9. Retreat to place_approach
        stage = self.make_move_to_named_stage(
            "place retreat", goal.place_approach, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.place_approach}' not found or invalid (place retreat)"
        task.add(stage)

        return self.load_plan_execute(task)

