"""Core MTC utilities - Python equivalent of base_stages.hpp/cpp.

Provides the BaseStages class which all stage implementations inherit from.
Contains planner factories, direction vectors, and common utilities.
"""

import json
import math
import traceback
from typing import Any, Dict, List, Optional, Tuple

import rclcpp
from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped, Vector3, Vector3Stamped
from std_msgs.msg import Header
from moveit_msgs.msg import (
    Constraints, JointConstraint, OrientationConstraint, MoveItErrorCodes,
)
from tf_transformations import quaternion_from_euler


# MoveIt error code → human-readable name mapping
# See: http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/MoveItErrorCodes.html
MOVEIT_ERROR_NAMES: Dict[int, str] = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.PREEMPTED: "PREEMPTED",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
    MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
    MoveItErrorCodes.INVALID_OBJECT_NAME: "INVALID_OBJECT_NAME",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "COLLISION_CHECKING_UNAVAILABLE",
    MoveItErrorCodes.ROBOT_STATE_STALE: "ROBOT_STATE_STALE",
    MoveItErrorCodes.SENSOR_INFO_STALE: "SENSOR_INFO_STALE",
    MoveItErrorCodes.COMMUNICATION_FAILURE: "COMMUNICATION_FAILURE",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}


# Direction vectors for relative moves in ik_frame
# Updated 2024: Compensates for 180° wrist rotation (camera mount XACRO change)
# - forward/backward (X): unchanged
# - left/right (Y): swapped
# - up/down (Z): swapped
DIRECTION_VECTORS: Dict[str, Tuple[float, float, float]] = {
    "forward":  ( 1.0,  0.0,  0.0), "x":  ( 1.0,  0.0,  0.0),
    "backward": (-1.0,  0.0,  0.0), "-x": (-1.0,  0.0,  0.0),
    "right":    ( 0.0, -1.0,  0.0), "y":  ( 0.0, -1.0,  0.0),
    "left":     ( 0.0,  1.0,  0.0), "-y": ( 0.0,  1.0,  0.0),
    "up":       ( 0.0,  0.0,  1.0), "z":  ( 0.0,  0.0,  1.0),
    "down":     ( 0.0,  0.0, -1.0), "-z": ( 0.0,  0.0, -1.0),
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
VELOCITY_SCALING = 0.2
ACCELERATION_SCALING = 0.2

# Default group/frame names matching C++ (base_stages.cpp)
DEFAULT_ARM_GROUP = "ur_arm"
DEFAULT_IK_FRAME = "flange"

# Module-level rclcpp node initialization for MTC operations.
# MTC requires rclcpp.Node (C++ backed via pybind11), not rclpy.Node, because
# MTC's C++ code expects rclcpp::Node::SharedPtr. This is independent of rclpy.
#
# Module-level init is safe because:
# 1. Python caches imported modules (no double-init risk)
# 2. Action servers always use MTC immediately (no benefit to lazy init)
# 3. Each server runs in separate process (no shared state concerns)
#
# IMPORTANT: The rclcpp.Node pybind11 wrapper does NOT expose declare_parameter().
# Parameters MUST be passed via NodeOptions.arguments using --ros-args -p key:=value.
# With automatically_declare_parameters_from_overrides=True, these are auto-declared
# on the C++ node and visible to MoveIt's PlanningPipeline.
#
# The __node:=beambot_mtc remapping is critical: ros2 launch injects
# --ros-args -r __node:=<server_name> which renames ALL nodes in the process.
# Without this override, both the rclcpp MTC node and the rclpy action server
# node end up with the same name, causing the shared /rosout publisher to be
# unregistered when either node is destroyed — silently breaking all /rosout
# logging for the process.
rclcpp.init()
_options = rclcpp.NodeOptions()
_options.automatically_declare_parameters_from_overrides = True
_options.allow_undeclared_parameters = True
_options.arguments = [
    "--ros-args",
    "-r", "__node:=beambot_mtc",
    # OMPL planning pipeline
    "-p", "ompl.planning_plugin:=ompl_interface/OMPLPlanner",
    "-p", "ompl.start_state_max_bounds_error:=0.1",
    # Pilz industrial motion planner pipeline
    "-p", "pilz_industrial_motion_planner.planning_plugin:=pilz_industrial_motion_planner/CommandPlanner",
    # Pilz cartesian limits (must match pilz_cartesian_limits.yaml in MoveIt configs)
    "-p", "robot_description_planning.cartesian_limits.max_trans_vel:=1.0",
    "-p", "robot_description_planning.cartesian_limits.max_trans_acc:=2.25",
    "-p", "robot_description_planning.cartesian_limits.max_trans_dec:=-5.0",
    "-p", "robot_description_planning.cartesian_limits.max_rot_vel:=1.57",
    "-p", "robot_description_planning.cartesian_limits.max_rot_acc:=3.15",
    "-p", "robot_description_planning.cartesian_limits.max_rot_dec:=-5.0",
    # Pilz PTP joint acceleration limits (UR5e: 5.0 rad/s² conservative, actual ~10-15)
    # NOTE: Must match values in ur5e_moveit_configs/*/config/joint_limits.yaml
    "-p", "robot_description_planning.joint_limits.shoulder_pan_joint.has_acceleration_limits:=true",
    "-p", "robot_description_planning.joint_limits.shoulder_pan_joint.max_acceleration:=5.0",
    "-p", "robot_description_planning.joint_limits.shoulder_lift_joint.has_acceleration_limits:=true",
    "-p", "robot_description_planning.joint_limits.shoulder_lift_joint.max_acceleration:=5.0",
    "-p", "robot_description_planning.joint_limits.elbow_joint.has_acceleration_limits:=true",
    "-p", "robot_description_planning.joint_limits.elbow_joint.max_acceleration:=5.0",
    "-p", "robot_description_planning.joint_limits.wrist_1_joint.has_acceleration_limits:=true",
    "-p", "robot_description_planning.joint_limits.wrist_1_joint.max_acceleration:=5.0",
    "-p", "robot_description_planning.joint_limits.wrist_2_joint.has_acceleration_limits:=true",
    "-p", "robot_description_planning.joint_limits.wrist_2_joint.max_acceleration:=5.0",
    "-p", "robot_description_planning.joint_limits.wrist_3_joint.has_acceleration_limits:=true",
    "-p", "robot_description_planning.joint_limits.wrist_3_joint.max_acceleration:=5.0",
]
_mtc_node = rclcpp.Node("beambot_mtc", _options)


def joints_from_degrees(degrees: List[float]) -> Dict[str, float]:
    """Convert joint angles from degrees to radians dict.

    Args:
        degrees: List of 6 joint angles in degrees

    Returns:
        Dictionary mapping joint names to radian values

    Example:
        >>> joints_from_degrees([0, -90, 90, -90, -90, 0])
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


def parse_constraints(constraints_dict: Optional[Dict[str, Any]]) -> Optional[Constraints]:
    """Parse a constraints dict from task JSON into a Constraints msg.

    All angles (position, tolerances, orientation) are in degrees and
    converted to radians internally, consistent with the rest of the framework.

    Args:
        constraints_dict: Dict with optional keys "joint_constraints" and
                         "orientation_constraints". None or empty dict returns None.

    Returns:
        Constraints message, or None if no constraints specified.
    """
    if not constraints_dict:
        return None

    constraints = Constraints()

    for jc_dict in constraints_dict.get("joint_constraints", []):
        jc = JointConstraint()
        jc.joint_name = jc_dict["joint_name"]
        jc.position = math.radians(jc_dict["position"])
        jc.tolerance_above = math.radians(jc_dict.get("tolerance_above", 1.0))
        jc.tolerance_below = math.radians(jc_dict.get("tolerance_below", 1.0))
        jc.weight = float(jc_dict.get("weight", 1.0))
        constraints.joint_constraints.append(jc)

    for oc_dict in constraints_dict.get("orientation_constraints", []):
        oc = OrientationConstraint()
        oc.header.frame_id = oc_dict.get("frame_id", "base_link")
        oc.link_name = oc_dict["link_name"]

        # orientation as [roll, pitch, yaw] in degrees -> quaternion
        orient = oc_dict["orientation"]
        q = quaternion_from_euler(
            math.radians(orient[0]),
            math.radians(orient[1]),
            math.radians(orient[2]),
        )
        oc.orientation.x = q[0]
        oc.orientation.y = q[1]
        oc.orientation.z = q[2]
        oc.orientation.w = q[3]

        # tolerance as [x, y, z] in degrees -> radians
        tol = oc_dict.get("tolerance", [5.0, 5.0, 5.0])
        oc.absolute_x_axis_tolerance = math.radians(tol[0])
        oc.absolute_y_axis_tolerance = math.radians(tol[1])
        oc.absolute_z_axis_tolerance = math.radians(tol[2])

        # parameterization: default ROTATION_VECTOR (matches MTC demo)
        param = oc_dict.get("parameterization", "rotation_vector")
        if param == "rotation_vector":
            oc.parameterization = OrientationConstraint.ROTATION_VECTOR
        elif param == "euler_xyz":
            oc.parameterization = OrientationConstraint.XYZ_EULER_ANGLES
        elif isinstance(param, int):
            oc.parameterization = param

        oc.weight = float(oc_dict.get("weight", 1.0))
        constraints.orientation_constraints.append(oc)

    # Return None if no actual constraints were added
    if not constraints.joint_constraints and not constraints.orientation_constraints:
        return None

    return constraints


def apply_constraints(stage, constraints: Optional[Constraints]) -> None:
    """Apply path constraints to an MTC stage if constraints are provided.

    Args:
        stage: MTC stage (MoveTo, MoveRelative) with path_constraints property
        constraints: Constraints message, or None to skip
    """
    if constraints is not None:
        stage.path_constraints = constraints


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
        ik_frame: str = ""
    ):
        """Initialize base stages.

        Args:
            rclpy_node: The rclpy node (for logging, clock, spinning futures)
            arm_group: MoveIt planning group for arm (default: ur_arm)
            ik_frame: Frame for IK calculations (default: flange)
        """
        self.rclpy_node = rclpy_node
        self._mtc_node = _mtc_node  # For MTC operations (C++ backed)
        self.arm_group = arm_group if arm_group else DEFAULT_ARM_GROUP
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
        task.loadRobotModel(self._mtc_node)

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
        planner = core.PipelinePlanner(self._mtc_node, "ompl")
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
        planner.step_size = 0.0005 # 1mm steps - good for collision detection
        planner.min_fraction = 0.9  # Require near-complete path (was 0.6)
        return planner

    def make_pilz_planner(self, mode: str = "LIN") -> core.PipelinePlanner:
        """Create Pilz industrial motion planner.

        Args:
            mode: "LIN" (straight-line Cartesian) or "PTP" (point-to-point joint)
        """
        planner = core.PipelinePlanner(self._mtc_node, "pilz_industrial_motion_planner")
        planner.planner = mode
        planner.max_velocity_scaling_factor = VELOCITY_SCALING
        planner.max_acceleration_scaling_factor = ACCELERATION_SCALING
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
        planner,
        constraints: Optional[Constraints] = None
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
        apply_constraints(stage, constraints)

        return stage

    def load_plan_execute(self, task: core.Task) -> Optional[str]:
        """Initialize, plan, and execute the task.

        Matches C++ behavior:
        1. init() - Initialize all stages
        2. plan() - Find solution(s)
        3. execute() - Execute and check result

        Args:
            task: Configured MTC task

        Returns:
            None if successful, error string describing failure otherwise
        """
        try:
            # Step 1: Initialize task
            self.logger.info(f"Initializing task: {task.name}")
            try:
                task.init()
            except Exception as e:
                error = f"Task init failed for '{task.name}': {e}"
                self.logger.error(error)
                return error

            # Step 2: Plan
            self.logger.info(f"Planning task: {task.name}")
            if not task.plan(max_solutions=1):
                error = f"PLANNING_FAILED: No motion plan found for task '{task.name}'"
                self.logger.error(error)
                return error

            # Step 3: Execute and check result
            if not task.solutions:
                error = f"PLANNING_FAILED: No solutions found for task '{task.name}'"
                self.logger.error(error)
                return error

            self.logger.info(
                f"Found {len(task.solutions)} solution(s), executing: {task.name}"
            )

            # Execute returns MoveItErrorCodes
            result = task.execute(task.solutions[0])

            # Check execution result (matching C++ behavior)
            if result.val != MoveItErrorCodes.SUCCESS:
                error_name = MOVEIT_ERROR_NAMES.get(result.val, "UNKNOWN")
                error = (
                    f"EXECUTION_FAILED: Task '{task.name}' execution failed: "
                    f"{error_name} (error code: {result.val})"
                )
                self.logger.error(error)
                return error

            self.logger.info(f"Task completed successfully: {task.name}")
            return None

        except Exception as e:
            self.logger.error(f"Task execution failed: {task.name} - {e}")
            self.logger.error(traceback.format_exc())
            return f"Task exception for '{task.name}': {e}"

    def parse_poses(self, poses_json: str) -> Optional[Dict[str, Any]]:
        """Parse poses JSON string.

        Args:
            poses_json: JSON string containing pose definitions

        Returns:
            Parsed dict, empty dict if poses_json is empty, None on parse error.
        """
        if not poses_json:
            return {}
        try:
            return json.loads(poses_json)
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse poses_json: {e}")
            return None

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

    def make_move_to_named_stage(
        self,
        label: str,
        pose_key: str,
        poses: Dict[str, Any],
        planner,
        constraints: Optional[Constraints] = None
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
        apply_constraints(stage, constraints)
        return stage

    def make_gripper_stage(
        self,
        label: str,
        planner,
        gripper_group: str,
        state_name: str
    ) -> Optional[stages.MoveTo]:
        """Create a gripper stage for a specific state.

        Args:
            label: Stage name
            planner: Planner to use
            gripper_group: MoveIt group name (from config)
            state_name: SRDF state name to move to

        Returns:
            Configured MoveTo stage for gripper, or None if no gripper/state
        """
        if not gripper_group or not state_name:
            self.logger.info(f"No gripper group or state for '{label}' - skipping")
            return None

        stage = stages.MoveTo(label, planner)
        stage.group = gripper_group
        stage.setGoal(state_name)
        return stage
