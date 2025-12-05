"""Core MTC utilities - Python equivalent of base_stages.hpp/cpp.

Provides the BaseStages class which all stage implementations inherit from.
Contains planner factories, direction vectors, and common utilities.
"""

import json
import math
from typing import Any, Dict, List, Optional, Tuple
from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped, Vector3, Vector3Stamped
from std_msgs.msg import Header
from moveit_msgs.msg import Constraints, JointConstraint

from mtc_py_lib.core.mtc_node import MTCNode


# Direction vectors matching C++ implementation in base_stages.cpp
# Z is inverted (flange Z points into tool)
DIRECTION_VECTORS: Dict[str, Tuple[float, float, float]] = {
    "forward":  ( 1.0,  0.0,  0.0), "x":  ( 1.0,  0.0,  0.0),
    "backward": (-1.0,  0.0,  0.0), "-x": (-1.0,  0.0,  0.0),
    "right":    ( 0.0,  1.0,  0.0), "y":  ( 0.0,  1.0,  0.0),
    "left":     ( 0.0, -1.0,  0.0), "-y": ( 0.0, -1.0,  0.0),
    "up":       ( 0.0,  0.0, -1.0), "z":  ( 0.0,  0.0, -1.0),
    "down":     ( 0.0,  0.0,  1.0), "-z": ( 0.0,  0.0,  1.0),
}

# UR5e default joint names (matches base_stages.cpp)
DEFAULT_JOINT_NAMES: List[str] = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

# Hardcoded scaling factors (matching C++ at 20%)
VELOCITY_SCALING = 0.8
ACCELERATION_SCALING = 0.8

# Default group/frame names matching C++ (base_stages.cpp)
DEFAULT_ARM_GROUP = "ur_arm"
DEFAULT_IK_FRAME = "flange"

# Tool exchange dock spacing
DOCK_SPACING_METERS = 0.1524  # 6 inches between dock stations


class BaseStages:
    """Base class providing MTC utilities for all stage implementations.

    Provides:
    - Task template creation with robot model
    - Planner factories (pipeline, cartesian, joint interpolation)
    - Directional move stage creation
    - Joint angle conversion utilities
    - Task planning and execution
    """

    def __init__(
        self,
        rclpy_node,
        arm_group: str = "",
        gripper_group: str = "",
        ik_frame: str = ""
    ):
        """Initialize base stages.

        Args:
            rclpy_node: The rclpy node (for logging)
            arm_group: MoveIt planning group for arm (default: ur_arm)
            gripper_group: MoveIt planning group for gripper
            ik_frame: Frame for IK calculations (default: flange)
        """
        self.rclpy_node = rclpy_node  # For logging
        self.mtc_node = MTCNode.get_instance()  # For MTC operations
        self.arm_group = arm_group if arm_group else DEFAULT_ARM_GROUP
        self.gripper_group = gripper_group
        self.ik_frame = ik_frame if ik_frame else DEFAULT_IK_FRAME
        self.logger = rclpy_node.get_logger()

    def create_task_template(self, name: str) -> core.Task:
        """Create a new MTC task with standard configuration.

        Args:
            name: Task name for identification

        Returns:
            Configured MTC Task with robot model loaded and CurrentState added
        """
        task = core.Task()
        task.name = name
        task.loadRobotModel(self.mtc_node.node)

        # Add current state as first stage
        task.add(stages.CurrentState("current_state"))
        return task

    def make_pipeline_planner(self) -> core.PipelinePlanner:
        """Create OMPL pipeline planner with standard configuration.

        Matches C++ behavior: PipelinePlanner(node_, "ompl")

        Returns:
            Configured PipelinePlanner using OMPL with RRTConnect (default)
        """
        # Specify "ompl" pipeline explicitly (matches C++)
        # Note: Don't set planner.planner to avoid "Cannot find planning configuration"
        # warning - OMPL defaults to RRTConnect anyway
        planner = core.PipelinePlanner(self.mtc_node.node, "ompl")
        planner.goal_joint_tolerance = 1e-4
        planner.max_velocity_scaling_factor = VELOCITY_SCALING
        planner.max_acceleration_scaling_factor = ACCELERATION_SCALING
        return planner

    def make_cartesian_planner(self) -> core.CartesianPath:
        """Create Cartesian path planner with standard configuration.

        Returns:
            Configured CartesianPath planner
        """
        planner = core.CartesianPath()
        planner.max_velocity_scaling_factor = VELOCITY_SCALING
        planner.max_acceleration_scaling_factor = ACCELERATION_SCALING
        planner.step_size = 0.001  # Matches C++ base_stages.cpp
        planner.min_fraction = 0.6  # Matches C++ base_stages.cpp
        return planner

    def make_joint_interpolation_planner(self) -> core.JointInterpolationPlanner:
        """Create joint interpolation planner (typically for gripper).

        Returns:
            Configured JointInterpolationPlanner
        """
        planner = core.JointInterpolationPlanner()
        planner.max_velocity_scaling_factor = VELOCITY_SCALING
        planner.max_acceleration_scaling_factor = ACCELERATION_SCALING
        return planner

    def _set_ik_frame(self, stage) -> None:
        """Set the ik_frame property on a stage.

        This is required for IK calculations in MTC stages.
        Matches C++: stage->properties().configureInitFrom(PARENT, {"ik_frame"})

        Args:
            stage: MTC stage (MoveTo, MoveRelative, etc.)
        """
        ik_frame_pose = PoseStamped()
        ik_frame_pose.header.frame_id = self.ik_frame
        stage.ik_frame = ik_frame_pose

    def create_relative_move_stage(
        self,
        label: str,
        direction: str,
        distance: float,
        planner
    ) -> stages.MoveRelative:
        """Create a MoveRelative stage for directional movement.

        Args:
            label: Stage name for identification
            direction: Direction string ("forward", "backward", "left",
                      "right", "up", "down", "x", "-x", "y", "-y", "z", "-z")
            distance: Distance in meters (positive value)
            planner: Planner instance (CartesianPath or PipelinePlanner)

        Returns:
            Configured MoveRelative stage

        Raises:
            ValueError: If direction is not recognized
        """
        if direction not in DIRECTION_VECTORS:
            raise ValueError(
                f"Unknown direction: '{direction}'. "
                f"Valid options: {list(DIRECTION_VECTORS.keys())}"
            )

        vec = DIRECTION_VECTORS[direction]

        stage = stages.MoveRelative(label, planner)
        stage.group = self.arm_group
        self._set_ik_frame(stage)

        # Create direction vector
        header = Header(frame_id=self.ik_frame)
        direction_vec = Vector3Stamped(
            header=header,
            vector=Vector3(
                x=vec[0] * distance,
                y=vec[1] * distance,
                z=vec[2] * distance
            )
        )
        stage.setDirection(direction_vec)

        return stage

    def load_plan_execute(self, task: core.Task) -> bool:
        """Initialize, plan, and execute the task.

        Matches C++ behavior:
        1. init() - Initialize all stages
        2. plan() - Find solution(s)
        3. execute() - Execute and check result

        Args:
            task: Configured MTC task

        Returns:
            True if planning and execution succeeded, False otherwise
        """
        from moveit_msgs.msg import MoveItErrorCodes

        try:
            # Step 0: Load robot model (matches C++ base_stages.cpp)
            # This is critical for proper trajectory execution
            # Note: getRobotModel() return type isn't exposed to Python,
            # so we always call loadRobotModel (it's idempotent)
            self.logger.info("Loading robot model...")
            task.loadRobotModel(self.mtc_node.node)

            # Step 1: Initialize task (critical - C++ does this!)
            self.logger.info(f"Initializing task: {task.name}")
            try:
                task.init()
            except Exception as e:
                self.logger.error(f"Task init failed: {task.name} - {e}")
                return False

            # Step 2: Plan
            self.logger.info(f"Planning task: {task.name}")
            if not task.plan(max_solutions=1):
                self.logger.error(f"Planning failed for task: {task.name}")
                return False

            # Step 3: Execute and check result
            if not task.solutions:
                self.logger.error(f"No solutions found for task: {task.name}")
                return False

            self.logger.info(
                f"Found {len(task.solutions)} solution(s), executing: {task.name}"
            )

            # Execute returns MoveItErrorCodes
            result = task.execute(task.solutions[0])

            # Check execution result (matching C++ behavior)
            if result.val != MoveItErrorCodes.SUCCESS:
                self.logger.error(
                    f"Execution failed for task: {task.name} "
                    f"(error code: {result.val})"
                )
                return False

            self.logger.info(f"Task completed successfully: {task.name}")
            return True

        except Exception as e:
            self.logger.error(f"Task execution failed: {task.name} - {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return False

    @staticmethod
    def joints_from_degrees(degrees: List[float]) -> Dict[str, float]:
        """Convert joint angles from degrees to radians dict.

        Args:
            degrees: List of 6 joint angles in degrees

        Returns:
            Dictionary mapping joint names to radian values

        Example:
            >>> BaseStages.joints_from_degrees([0, -90, 90, -90, -90, 0])
            {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': -1.5707..., ...}
        """
        if len(degrees) != len(DEFAULT_JOINT_NAMES):
            raise ValueError(
                f"Expected {len(DEFAULT_JOINT_NAMES)} joint values, "
                f"got {len(degrees)}"
            )
        return {
            name: math.radians(deg)
            for name, deg in zip(DEFAULT_JOINT_NAMES, degrees)
        }

    @staticmethod
    def default_joint_names() -> List[str]:
        """Get the default UR5e joint names.

        Returns:
            List of joint names in order
        """
        return DEFAULT_JOINT_NAMES.copy()

    @staticmethod
    def create_wrist3_level_constraint() -> Constraints:
        """Create a path constraint to keep wrist_3_joint level.

        Used during pick operations to maintain tool orientation.

        Returns:
            Constraints message with wrist_3_joint locked at 0.0
        """
        constraint = Constraints()
        jc = JointConstraint()
        jc.joint_name = "wrist_3_joint"
        jc.position = 0.0
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraint.joint_constraints.append(jc)
        return constraint

    def parse_poses(
        self, poses_json: str, required: bool = False
    ) -> Optional[Dict[str, Any]]:
        """Parse poses JSON string with consistent error handling.

        Consolidates the duplicate _parse_poses() methods from subclasses.

        Args:
            poses_json: JSON string containing pose definitions
            required: If True, empty/invalid JSON returns None (error case).
                     If False, returns empty dict (poses optional).

        Returns:
            Dictionary of pose names to values.
            Returns None if required=True and poses_json is empty/invalid.
            Returns {} if required=False and poses_json is empty/invalid.

        Example:
            # For operations where poses are required (pick_place, tool_exchange)
            poses = self.parse_poses(goal.poses_json, required=True)
            if poses is None:
                return False

            # For operations where poses are optional (moveto with named state)
            poses = self.parse_poses(goal.poses_json, required=False)
        """
        if not poses_json:
            if required:
                self.logger.error("poses_json is required but empty")
                return None
            return {}

        try:
            return json.loads(poses_json)
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse poses_json: {e}")
            return None if required else {}

    def get_joint_pose(
        self, poses: Dict[str, Any], pose_key: str
    ) -> Optional[List[float]]:
        """Get and validate a joint pose from the poses dictionary.

        Args:
            poses: Dictionary of pose definitions
            pose_key: Key to look up in poses dict

        Returns:
            List of 6 joint angles in degrees, or None if invalid/missing
        """
        if pose_key not in poses:
            self.logger.error(f"Pose '{pose_key}' not found in poses_json")
            return None

        joint_pose = poses[pose_key]
        if not isinstance(joint_pose, list) or len(joint_pose) != 6:
            self.logger.error(
                f"'{pose_key}' must be array of 6 joint angles, "
                f"got {type(joint_pose).__name__}"
            )
            return None

        return joint_pose
