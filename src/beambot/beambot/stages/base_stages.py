"""Core MTC utilities shared by every stage subclass.

Provides the BaseStages class plus planner factories, direction vectors,
task planning/execution helpers, and the module-level rclcpp node that
MTC's C++ backend requires.
"""

import json
import math
import os
import sys
import threading
import time
import traceback
from typing import Any

import rclcpp
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Vector3, Vector3Stamped
from moveit.task_constructor import core, stages
from moveit_msgs.msg import (
    Constraints, JointConstraint, MoveItErrorCodes, OrientationConstraint,
)
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler


# MoveIt error code → human-readable name mapping
# See: http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/MoveItErrorCodes.html
MOVEIT_ERROR_NAMES: dict[int, str] = {
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
DIRECTION_VECTORS: dict[str, tuple[float, float, float]] = {
    "forward":  ( 1.0,  0.0,  0.0), "x":  ( 1.0,  0.0,  0.0),
    "backward": (-1.0,  0.0,  0.0), "-x": (-1.0,  0.0,  0.0),
    "right":    ( 0.0, -1.0,  0.0), "y":  ( 0.0, -1.0,  0.0),
    "left":     ( 0.0,  1.0,  0.0), "-y": ( 0.0,  1.0,  0.0),
    "up":       ( 0.0,  0.0,  1.0), "z":  ( 0.0,  0.0,  1.0),
    "down":     ( 0.0,  0.0, -1.0), "-z": ( 0.0,  0.0, -1.0),
}

# UR5e default joint names
DEFAULT_JOINT_NAMES: list[str] = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

# Global velocity/acceleration scaling (20% of joint limits)
VELOCITY_SCALING = 0.2
ACCELERATION_SCALING = 0.2

# Default MoveIt planning group and IK frame
DEFAULT_ARM_GROUP = "ur_arm"
DEFAULT_IK_FRAME = "flange"


def wait_for_future(future, timeout: float, poll_interval: float = 0.01) -> bool:
    """Poll ``future.done()`` until complete or timeout, without spinning.

    Returns True iff the future completed within the timeout.

    Used everywhere a stage or action-callback calls a ROS service or action
    client. We can't use ``rclpy.spin_until_future_complete`` here: this code
    runs inside a callback on a node whose MultiThreadedExecutor is already
    spinning elsewhere, and re-entering the executor raises "Executor is
    already spinning". The executor is what actually delivers the response to
    the future; this function just waits for that to happen.

    Uses time.monotonic() so wall-clock jumps (NTP, manual time changes) don't
    skew the timeout. Caller retrieves the value via ``future.result()`` and
    does its own logging/cleanup — this helper intentionally returns only a
    boolean to keep call sites explicit about their error handling.
    """
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() >= deadline:
            return False
        time.sleep(poll_interval)
    return True

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
#
# ROSOUT FIX: Disable rosout on the MTC rclcpp node entirely. On Humble,
# rcl's rosout hashmap has no reference counting — when MTC's internal nodes
# (Introspection, executor) are destroyed, they unregister the rosout publisher
# for ALL same-named nodes, silently breaking /rosout logging for the rclpy
# action server. With enable_rosout=False, the MTC node never touches the
# hashmap, keeping the rclpy node's /rosout publisher intact. MoveIt C++
# internal logs still go to stdout/stderr via rcutils.
def _load_joint_accel_limits() -> dict[str, float]:
    """Collect max_acceleration for every joint declared under any gripper's
    joint_limits.yaml in ur5e_moveit_config/config/<gripper>/.

    The active gripper isn't known at module-import time (beambot_mtc is built
    before the orchestrator picks a gripper), so we load the union across all
    configs. This is safe because:
      - Arm joints are shared across all grippers with identical limits.
      - Gripper-specific joints are disjoint (only declared in that gripper).
    TOTG only consults the limits of joints actually in a trajectory, so the
    extra entries are inert.

    If two configs ever declare different max_acceleration values for the same
    joint, a WARN is printed — that's the trip-wire indicating this union
    approach has stopped being valid and beambot_mtc should become
    gripper-aware (load only the active gripper's yaml instead).
    """
    cfg_root = os.path.join(
        get_package_share_directory("ur5e_moveit_config"), "config"
    )
    limits: dict[str, float] = {}
    for entry in sorted(os.listdir(cfg_root)):
        path = os.path.join(cfg_root, entry, "joint_limits.yaml")
        if not os.path.isfile(path):
            continue
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        for joint, spec in (data.get("joint_limits") or {}).items():
            if not spec.get("has_acceleration_limits"):
                continue
            if "max_acceleration" not in spec:
                continue
            value = float(spec["max_acceleration"])
            existing = limits.get(joint)
            if existing is not None and existing != value:
                print(
                    f"[beambot_mtc] WARN: max_acceleration for '{joint}' "
                    f"differs across gripper configs ({existing} vs {value} "
                    f"in {entry}/joint_limits.yaml); using first-seen "
                    f"({existing}). Time to make beambot_mtc gripper-aware.",
                    file=sys.stderr,
                )
                continue
            limits[joint] = value
    return limits


def _build_joint_limit_args(limits: dict[str, float]) -> list[str]:
    args: list[str] = []
    for joint, accel in limits.items():
        args += [
            "-p", f"robot_description_planning.joint_limits.{joint}.has_acceleration_limits:=true",
            "-p", f"robot_description_planning.joint_limits.{joint}.max_acceleration:={accel}",
        ]
    return args


rclcpp.init()
_options = rclcpp.NodeOptions()
_options.automatically_declare_parameters_from_overrides = True
_options.allow_undeclared_parameters = True
_options.enable_rosout = False
_options.arguments = [
    "--ros-args",
    "-r", "__node:=beambot_mtc",
    # OMPL planning pipeline
    "-p", "ompl.planning_plugins:=['ompl_interface/OMPLPlanner']",
    "-p", "ompl.start_state_max_bounds_error:=0.1",
    # Pilz industrial motion planner pipeline
    "-p", "pilz_industrial_motion_planner.planning_plugins:=['pilz_industrial_motion_planner/CommandPlanner']",
    # Pilz cartesian limits (must match pilz_cartesian_limits.yaml in MoveIt configs)
    "-p", "robot_description_planning.cartesian_limits.max_trans_vel:=1.0",
    "-p", "robot_description_planning.cartesian_limits.max_trans_acc:=2.25",
    "-p", "robot_description_planning.cartesian_limits.max_trans_dec:=-5.0",
    "-p", "robot_description_planning.cartesian_limits.max_rot_vel:=1.57",
    "-p", "robot_description_planning.cartesian_limits.max_rot_acc:=3.15",
    "-p", "robot_description_planning.cartesian_limits.max_rot_dec:=-5.0",
    # Pilz PTP / TOTG joint acceleration limits — union across all gripper
    # joint_limits.yaml files under ur5e_moveit_config/config/<gripper>/.
    *_build_joint_limit_args(_load_joint_accel_limits()),
]
_mtc_node = rclcpp.Node("beambot_mtc", _options)


def joints_from_degrees(degrees: list[float]) -> dict[str, float]:
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


def parse_constraints(constraints_dict: dict[str, Any] | None) -> Constraints | None:
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


def apply_constraints(stage, constraints: Constraints | None) -> None:
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
        # MTC introspection publishes solutions to RViz's Motion Planning Tasks
        # panel. Creates/destroys Introspection nodes that poison rcl's rosout
        # hashmap on Humble (unfixed rcl bug since 2019, ros2/rcl#984). Effect:
        # MTC C++ internal logs break after first task, but our rclpy-based logs
        # (detection, IK, vacuum) are unaffected. Re-enabled for RViz visualization.
        task = core.Task()
        task.enableIntrospection(True)
        task.name = name
        task.loadRobotModel(self._mtc_node)

        # Add current state as first stage
        task.add(stages.CurrentState("current_state"))
        return task

    def make_pipeline_planner(self) -> core.PipelinePlanner:
        """Create OMPL pipeline planner with standard configuration.

        Returns:
            Configured PipelinePlanner using OMPL with RRTConnect (default)
        """
        # Don't set planner.planner_id — OMPL defaults to RRTConnect, and
        # setting it explicitly triggers "Cannot find planning configuration"
        # warnings unless the config lists the named planner.
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
        # Jazzy MTC dropped the mutable .planner attribute; planner_id is now a
        # constructor kwarg.
        planner = core.PipelinePlanner(
            self._mtc_node, "pilz_industrial_motion_planner", planner_id=mode
        )
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
        """Set the ik_frame property on a stage (required for MTC IK).

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
        planner=None,
        constraints: Constraints | None = None
    ):
        """Create a MoveRelative stage (or Fallbacks container) for directional movement.

        When planner is provided, returns a single MoveRelative stage.
        When planner is None, returns a Fallbacks container:
        Pilz LIN → CartesianPath → Pilz PTP.

        Args:
            label: Stage name for identification
            direction: Direction string ("forward", "backward", "left",
                      "right", "up", "down", "x", "-x", "y", "-y", "z", "-z")
            distance: Distance in meters (positive value)
            planner: Planner instance, or None for automatic fallback chain
            constraints: Optional path constraints

        Returns:
            Configured MoveRelative or Fallbacks stage

        Raises:
            ValueError: If direction is not recognized
        """
        if direction not in DIRECTION_VECTORS:
            raise ValueError(
                f"Unknown direction: '{direction}'. "
                f"Valid options: {list(DIRECTION_VECTORS.keys())}"
            )

        vec = DIRECTION_VECTORS[direction]

        # Build planner list: single planner or fallback chain
        if planner is not None:
            planners = [(planner, None)]
        else:
            planners = [
                (self.make_pilz_planner("LIN"), "Pilz LIN"),
                (self.make_cartesian_planner(), "CartesianPath"),
                (self.make_pilz_planner("PTP"), "Pilz PTP"),
            ]

        if len(planners) == 1:
            # Single planner — return a plain MoveRelative stage
            stage = stages.MoveRelative(label, planners[0][0])
            stage.group = self.arm_group
            self._set_ik_frame(stage)
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

        # Multiple planners — return a Fallbacks container
        fb = core.Fallbacks(label)
        for p, suffix in planners:
            stage = stages.MoveRelative(f"{label} [{suffix}]", p)
            stage.group = self.arm_group
            self._set_ik_frame(stage)
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
            fb.add(stage)
        return fb

    def load_plan_execute(self, task: core.Task) -> str | None:
        """Initialize, plan, and execute the task.

        Runs init() -> plan() -> execute(), logs the planned end-state joint
        angles, and returns None on success or a structured error string
        (PLANNING_FAILED, EXECUTION_FAILED: <MoveItErrorName>, etc.).

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

            # Log planned end-state joint angles for debugging IK issues
            try:
                sol = task.solutions[0]
                sol_msg = sol.toMsg()
                for i, sub in enumerate(sol_msg.sub_trajectory):
                    traj = sub.trajectory.joint_trajectory
                    if traj.joint_names and traj.points:
                        last_pt = traj.points[-1]
                        pairs = [(n, math.degrees(p)) for n, p in zip(traj.joint_names, last_pt.positions)]
                        joint_str = ', '.join(f'{n}={v:.2f}' for n, v in pairs)
                        self.logger.info(f"Planned [{i}]: {joint_str}")
            except Exception as e:
                self.logger.warning(f"Could not extract planned joints: {e}")

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

    def parse_poses(self, poses_json: str) -> dict[str, Any] | None:
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
        self, poses: dict[str, Any], pose_key: str
    ) -> list[float] | None:
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
        poses: dict[str, Any],
        planner=None,
        constraints: Constraints | None = None
    ):
        """Create a MoveTo stage (or Fallbacks container) for a named joint pose.

        When planner is provided, returns a single MoveTo stage with that planner.
        When planner is None, returns a Fallbacks container: Pilz PTP → OMPL.

        Args:
            label: Stage name
            pose_key: Key in poses dict
            poses: Dictionary of pose definitions
            planner: Planner to use, or None for automatic fallback chain
            constraints: Optional path constraints

        Returns:
            Configured MoveTo or Fallbacks stage, or None if pose not found
        """
        joint_pose = self.get_joint_pose(poses, pose_key)
        if joint_pose is None:
            return None

        joint_goal = joints_from_degrees(joint_pose)

        if planner is not None:
            stage = stages.MoveTo(label, planner)
            stage.group = self.arm_group
            self._set_ik_frame(stage)
            stage.setGoal(joint_goal)
            apply_constraints(stage, constraints)
            return stage

        # Fallback chain: Pilz PTP → OMPL
        fb = core.Fallbacks(label)
        for planner_fn, suffix in [
            (lambda: self.make_pilz_planner("PTP"), "Pilz PTP"),
            (self.make_pipeline_planner, "OMPL"),
        ]:
            stage = stages.MoveTo(f"{label} [{suffix}]", planner_fn())
            stage.group = self.arm_group
            self._set_ik_frame(stage)
            stage.setGoal(joint_goal)
            apply_constraints(stage, constraints)
            fb.add(stage)
        return fb

    def make_gripper_stage(
        self,
        label: str,
        planner,
        gripper_group: str,
        state_name: str
    ) -> stages.MoveTo | None:
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

    def compute_deterministic_ik(
        self, approach: PoseStamped, ik_frame: str
    ) -> dict[str, float] | None:
        """Compute IK via /compute_ik service for a deterministic joint goal.

        Uses a snapshot of the current joint positions as the seed, quantized
        to 0.01° for reproducibility. Since /compute_ik runs a single KDL
        attempt without random re-seeding, the result is deterministic for a
        given seed+target pair. (#51)

        Returns:
            Joint dict {name: radians} for the arm group, or None on failure.
        """
        js_holder = [None]
        js_event = threading.Event()

        def _on_js(msg):
            js_holder[0] = msg
            js_event.set()

        js_sub = self.rclpy_node.create_subscription(JointState, '/joint_states', _on_js, 10)
        js_event.wait(timeout=1.0)
        self.rclpy_node.destroy_subscription(js_sub)

        if not js_holder[0]:
            self.logger.warning("No joint_states for IK seed")
            return None

        quantized = []
        for p in js_holder[0].position:
            deg = math.degrees(p)
            deg_q = round(deg, 2)
            quantized.append(math.radians(deg_q))

        try:
            ik_client = self.rclpy_node.create_client(GetPositionIK, '/compute_ik')
            if not ik_client.wait_for_service(timeout_sec=2.0):
                self.logger.warning("/compute_ik service not available")
                return None

            req = GetPositionIK.Request()
            req.ik_request.group_name = self.arm_group
            req.ik_request.ik_link_name = ik_frame
            req.ik_request.robot_state.joint_state.name = list(js_holder[0].name)
            req.ik_request.robot_state.joint_state.position = quantized
            req.ik_request.pose_stamped = approach
            req.ik_request.timeout.sec = 1

            future = ik_client.call_async(req)
            if not wait_for_future(future, timeout=5.0):
                self.logger.warning("/compute_ik timed out")
                self.rclpy_node.destroy_client(ik_client)
                return None
            self.rclpy_node.destroy_client(ik_client)

            result = future.result()
            if result and result.error_code.val == 1:  # SUCCESS
                joint_goal = {}
                for name, pos in zip(result.solution.joint_state.name,
                                     result.solution.joint_state.position):
                    if name in DEFAULT_JOINT_NAMES:
                        joint_goal[name] = pos
                if len(joint_goal) == 6:
                    return joint_goal

            self.logger.warning(f"compute_ik failed: error_code={result.error_code.val if result else 'None'}")
            return None

        except Exception as e:
            self.logger.warning(f"Deterministic IK error: {e}")
            return None

