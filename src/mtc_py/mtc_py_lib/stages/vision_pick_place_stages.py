"""VisionPickPlaceStages - Python equivalent of vision_pick_place_stages.cpp.

Vision-guided pick and place sequence:
- Detects pick/place targets via ArUco markers
- Computes grasp poses with configurable offsets
- Executes pick sequence with gripper operations
"""

import json
from geometry_msgs.msg import PoseStamped
from moveit.task_constructor import stages
from tf_transformations import (
    quaternion_from_euler,
    quaternion_multiply,
    quaternion_matrix,
    quaternion_from_matrix
)
import numpy as np

from mtc_py_lib.stages.base_stages import BaseStages, create_wrist3_level_constraint
from mtc_py_lib.stages.vision_stages import VisionStages


class VisionPickPlaceStages(BaseStages):
    """Handles vision-guided pick and place operations."""

    def __init__(self, rclpy_node, arm_group: str = "", ik_frame: str = ""):
        """Initialize VisionPickPlaceStages.

        Args:
            rclpy_node: ROS node for service calls and TF
            arm_group: MoveIt planning group for arm
            ik_frame: IK frame for motion planning
        """
        super().__init__(rclpy_node, arm_group, ik_frame=ik_frame)

        # Create VisionStages instance for marker detection
        self._vision = VisionStages(rclpy_node, arm_group, ik_frame)

        self.logger.info("VisionPickPlaceStages initialized")

    def run(self, goal) -> bool:
        """Execute VisionPickPlace action.

        Args:
            goal: VisionPickPlaceAction.Goal with fields:
                - pick_tag_id: ArUco marker ID for pick
                - place_tag_id: ArUco marker ID for place (-1 = use default)
                - gripper_group: MoveIt group name (from config)
                - gripper_states_json: JSON dict of semantic states
                - grasp_offset_json: JSON offset configuration
                - approach_offset: Vertical approach distance
                - retreat_offset: Vertical retreat distance

        Returns:
            True if successful, False otherwise
        """
        self.logger.info(
            f"Executing vision pick/place: pick_tag={goal.pick_tag_id}, "
            f"place_tag={goal.place_tag_id}, gripper_group={goal.gripper_group}"
        )

        # Parse gripper states
        try:
            gripper_states = json.loads(goal.gripper_states_json) if goal.gripper_states_json else {}
        except json.JSONDecodeError as e:
            self.logger.error(f"Invalid gripper_states_json: {e}")
            return False

        # Parse grasp offset (default: 5cm above, 180° rotation)
        if goal.grasp_offset_json:
            try:
                grasp_offset = json.loads(goal.grasp_offset_json)
            except json.JSONDecodeError as e:
                self.logger.error(f"Invalid grasp_offset_json: {e}")
                return False
        else:
            grasp_offset = {"x": 0, "y": 0, "z": 0.05, "rpy": [0, 3.14159, 0]}

        # Detect pick target
        pick_tag_pose = self._vision.detect_and_transform_tag(goal.pick_tag_id, 10.0)
        if pick_tag_pose is None:
            self.logger.error(f"Failed to detect pick tag {goal.pick_tag_id}")
            return False

        # Compute pick poses (minimal offset approach)
        grasp_pose = PoseStamped()
        grasp_pose.header = pick_tag_pose.header
        grasp_pose.pose.position.x = pick_tag_pose.pose.position.x
        grasp_pose.pose.position.y = pick_tag_pose.pose.position.y
        grasp_pose.pose.position.z = pick_tag_pose.pose.position.z + 0.02  # 2cm above

        # Point down orientation (y=1 means 180° around Y)
        grasp_pose.pose.orientation.x = 0.0
        grasp_pose.pose.orientation.y = 1.0
        grasp_pose.pose.orientation.z = 0.0
        grasp_pose.pose.orientation.w = 0.0

        pick_approach = self._compute_offset_pose(grasp_pose, goal.approach_offset)
        pick_retreat = self._compute_offset_pose(grasp_pose, goal.retreat_offset)

        self.logger.info(
            f"Pick: grasp=[{grasp_pose.pose.position.x:.3f}, "
            f"{grasp_pose.pose.position.y:.3f}, {grasp_pose.pose.position.z:.3f}], "
            f"approach=+{goal.approach_offset:.2f}m, retreat=+{goal.retreat_offset:.2f}m"
        )

        # Compute place poses
        if goal.place_tag_id >= 0:
            place_tag_pose = self._vision.detect_and_transform_tag(goal.place_tag_id, 10.0)
            if place_tag_pose is None:
                self.logger.error(f"Failed to detect place tag {goal.place_tag_id}")
                return False
            place_pose = self._compute_grasp_pose(place_tag_pose, grasp_offset)
            place_approach = self._compute_offset_pose(place_pose, goal.approach_offset)
            place_retreat = self._compute_offset_pose(place_pose, goal.retreat_offset)
        else:
            # Default place position
            place_pose = PoseStamped()
            place_pose.header.frame_id = "base_link"
            place_pose.pose.position.x = 0.4
            place_pose.pose.position.y = 0.3
            place_pose.pose.position.z = 0.15
            place_pose.pose.orientation.x = 0.0
            place_pose.pose.orientation.y = 1.0
            place_pose.pose.orientation.z = 0.0
            place_pose.pose.orientation.w = 0.0
            place_approach = self._compute_offset_pose(place_pose, goal.approach_offset)
            place_retreat = self._compute_offset_pose(place_pose, goal.retreat_offset)

        # Build MTC task
        task = self.create_task_template("Vision Pick and Place")
        gripper_planner = self.make_joint_interpolation_planner()
        pipeline = self.make_pipeline_planner()

        cartesian = self.make_cartesian_planner()
        cartesian.step_size = 0.005
        cartesian.min_fraction = 0.5

        # Pick sequence
        open_stage = self.make_gripper_stage(
            "open gripper", gripper_planner, goal.gripper_group, gripper_states.get("release", "")
        )
        if open_stage:
            task.add(open_stage)

        task.add(self._make_cartesian_move_stage("pick approach", pick_approach, pipeline, False))
        task.add(self._make_cartesian_move_stage("grasp", grasp_pose, cartesian, True))

        close_stage = self.make_gripper_stage(
            "close gripper", gripper_planner, goal.gripper_group, gripper_states.get("grasp", "")
        )
        if close_stage:
            task.add(close_stage)

        task.add(self._make_cartesian_move_stage("pick retreat", pick_retreat, cartesian, True))

        # Place sequence (currently disabled for testing as in C++)
        self.logger.warn("Place sequence disabled - pick only")

        return self.load_plan_execute(task)

    def _compute_grasp_pose(
        self,
        tag_pose: PoseStamped,
        offset: dict
    ) -> PoseStamped:
        """Compute grasp pose from tag pose with offset.

        Args:
            tag_pose: Detected marker pose
            offset: Dict with x, y, z and optional rpy

        Returns:
            Grasp pose
        """
        result = PoseStamped()
        result.header = tag_pose.header
        result.header.frame_id = "base_link"

        # Get tag transform
        q = [
            tag_pose.pose.orientation.x,
            tag_pose.pose.orientation.y,
            tag_pose.pose.orientation.z,
            tag_pose.pose.orientation.w
        ]

        # Apply position offset in local frame
        rot_matrix = quaternion_matrix(q)[:3, :3]
        offset_local = np.array([
            offset.get("x", 0.0),
            offset.get("y", 0.0),
            offset.get("z", 0.0)
        ])
        offset_world = rot_matrix @ offset_local

        result.pose.position.x = tag_pose.pose.position.x + offset_world[0]
        result.pose.position.y = tag_pose.pose.position.y + offset_world[1]
        result.pose.position.z = tag_pose.pose.position.z + offset_world[2]

        # Apply rotation offset if specified
        if "rpy" in offset and len(offset["rpy"]) == 3:
            rpy = offset["rpy"]
            q_offset = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            q_result = quaternion_multiply(q, q_offset)
            result.pose.orientation.x = q_result[0]
            result.pose.orientation.y = q_result[1]
            result.pose.orientation.z = q_result[2]
            result.pose.orientation.w = q_result[3]
        else:
            result.pose.orientation = tag_pose.pose.orientation

        return result

    def _compute_offset_pose(
        self,
        base_pose: PoseStamped,
        z_offset: float
    ) -> PoseStamped:
        """Compute pose with vertical offset.

        Args:
            base_pose: Base pose
            z_offset: Vertical offset in meters

        Returns:
            Offset pose
        """
        result = PoseStamped()
        result.header = base_pose.header
        result.pose = base_pose.pose
        result.pose.position.z = base_pose.pose.position.z + z_offset
        return result

    def _make_cartesian_move_stage(
        self,
        label: str,
        target: PoseStamped,
        planner,
        apply_wrist_constraint: bool
    ) -> stages.MoveTo:
        """Create a Cartesian move stage.

        Args:
            label: Stage name
            target: Target pose
            planner: Planner to use
            apply_wrist_constraint: Whether to keep wrist level

        Returns:
            Configured MoveTo stage
        """
        stage = stages.MoveTo(label, planner)
        stage.group = self.arm_group
        self._set_ik_frame(stage)
        stage.setGoal(target)

        if apply_wrist_constraint:
            stage.setPathConstraints(create_wrist3_level_constraint())

        return stage
