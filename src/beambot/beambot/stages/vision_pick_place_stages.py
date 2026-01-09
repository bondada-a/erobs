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
from moveit.task_constructor import stages

from beambot.stages.base_stages import BaseStages, joints_from_degrees
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

    def run(self, goal) -> bool:
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
            True if successful, False otherwise
        """
        self.logger.info(
            f"Vision pick/place: detection={goal.detection_type}, "
            f"tag_id={goal.tag_id}, gripper_group={goal.gripper_group}"
        )

        # Parse poses
        poses = self.parse_poses(goal.poses_json)
        if poses is None:
            return False

        # Parse gripper states
        try:
            gripper_states = json.loads(goal.gripper_states_json) if goal.gripper_states_json else {}
        except json.JSONDecodeError as e:
            self.logger.error(f"Invalid gripper_states_json: {e}")
            return False

        # Get z_offset (default 0.02m = 2cm above detected point)
        z_offset = goal.z_offset if goal.z_offset != 0.0 else 0.02

        # === TASK 1: Position for Detection ===
        self.logger.info("Task 1/2: Moving to sample_approach for detection...")
        if not self._execute_position_for_detection(goal, poses, gripper_states):
            self.logger.error("Failed to position for detection")
            return False

        # === RUNTIME: Vision Detection ===
        self.logger.info(f"Detecting {goal.detection_type}...")
        grasp_pose = self._detect_and_compute_grasp(goal, z_offset)
        if grasp_pose is None:
            self.logger.error("Vision detection failed")
            return False

        # === TASK 2: Pick and Place ===
        self.logger.info("Task 2/2: Executing pick and place...")
        if not self._execute_pick_and_place(goal, poses, gripper_states, grasp_pose):
            self.logger.error("Pick and place execution failed")
            return False

        self.logger.info("Vision pick and place completed successfully")
        return True

    def _execute_position_for_detection(
        self,
        goal,
        poses: Dict[str, Any],
        gripper_states: Dict[str, str]
    ) -> bool:
        """Execute Task 1: Open gripper and move to sample_approach.

        Returns:
            True if successful, False otherwise
        """
        task = self.create_task_template("Position for Detection")
        pipeline_planner = self.make_pipeline_planner()
        gripper_planner = self.make_joint_interpolation_planner()

        # 1. Open gripper
        open_stage = self.make_gripper_stage(
            "open gripper", gripper_planner, goal.gripper_group, gripper_states.get("release", "")
        )
        if open_stage:
            task.add(open_stage)

        # 2. Move to sample_approach (where camera can see samples)
        stage = self._make_move_to_named_stage(
            "sample approach", goal.sample_approach, poses, pipeline_planner
        )
        if not stage:
            return False
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
        grasp_pose: PoseStamped
    ) -> bool:
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
            True if successful, False otherwise
        """
        task = self.create_task_template("Pick and Place")
        pipeline_planner = self.make_pipeline_planner()
        gripper_planner = self.make_joint_interpolation_planner()
        cartesian_planner = self.make_cartesian_planner()

        # === PICK SEQUENCE ===

        # 3. Move to grasp pose (Cartesian for precision)
        grasp_stage = stages.MoveTo("grasp", cartesian_planner)
        grasp_stage.group = self.arm_group
        self._set_ik_frame(grasp_stage)
        grasp_stage.setGoal(grasp_pose)
        task.add(grasp_stage)

        # 4. Close gripper
        close_stage = self.make_gripper_stage(
            "close gripper", gripper_planner, goal.gripper_group, gripper_states.get("grasp", "")
        )
        if close_stage:
            task.add(close_stage)

        # 5. Retreat to sample_approach (safe joint pose)
        stage = self._make_move_to_named_stage(
            "pick retreat", goal.sample_approach, poses, pipeline_planner
        )
        if not stage:
            return False
        task.add(stage)

        # === PLACE SEQUENCE ===

        # 6. Move to place_approach
        stage = self._make_move_to_named_stage(
            "place approach", goal.place_approach, poses, pipeline_planner
        )
        if not stage:
            return False
        task.add(stage)

        # 7. Move to place_target
        stage = self._make_move_to_named_stage(
            "place", goal.place_target, poses, pipeline_planner
        )
        if not stage:
            return False
        task.add(stage)

        # 8. Open gripper (release)
        release_stage = self.make_gripper_stage(
            "release", gripper_planner, goal.gripper_group, gripper_states.get("release", "")
        )
        if release_stage:
            task.add(release_stage)

        # 9. Retreat to place_approach
        stage = self._make_move_to_named_stage(
            "place retreat", goal.place_approach, poses, pipeline_planner
        )
        if not stage:
            return False
        task.add(stage)

        return self.load_plan_execute(task)

    def _make_move_to_named_stage(
        self,
        label: str,
        pose_key: str,
        poses: Dict[str, Any],
        planner
    ) -> Optional[stages.MoveTo]:
        """Create a MoveTo stage for a named joint pose.

        Args:
            label: Stage name
            pose_key: Key in poses dict
            poses: Dictionary of pose definitions
            planner: Planner to use

        Returns:
            Configured MoveTo stage, or None if pose not found
        """
        joint_pose = self.get_joint_pose(poses, pose_key)
        if joint_pose is None:
            return None

        stage = stages.MoveTo(label, planner)
        stage.group = self.arm_group
        self._set_ik_frame(stage)
        stage.setGoal(joints_from_degrees(joint_pose))
        return stage
