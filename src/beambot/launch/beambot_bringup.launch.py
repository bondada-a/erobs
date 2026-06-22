"""Launch all beambot action servers.

This launch file starts all the beambot action servers:
- beambot_moveto: MoveTo operations
- beambot_endeffector: Gripper operations
- beambot_toolexchange: Tool exchange operations
- beambot_vision_moveto: Vision-guided moves
- beambot_sample: Pick and place sample operations (vision or hardcoded)
- beambot_pipettor: Pipettor operations
- beambot_orchestrator: Central coordinator

Usage:
    ros2 launch beambot beambot_bringup.launch.py
    ros2 launch beambot beambot_bringup.launch.py enable_vision:=false
    ros2 launch beambot beambot_bringup.launch.py use_mock_hardware:=true
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from tracetools_launch.action import Trace


def generate_launch_description():
    """Generate launch description for beambot servers."""

    # OMPL + Pilz pipeline parameters for PipelinePlanner
    # Required because Python's rclcpp.Node binding lacks declare_parameter().
    # Without these, PipelinePlanner falls back to CHOMP instead of OMPL.
    #
    # The Pilz response_adapters (ValidateSolution in particular) are critical:
    # on Jazzy, missing adapter config means Pilz PTP returns collision-containing
    # trajectories as valid solutions. MTC's Fallbacks container then treats the
    # first (unchecked) solution as success without ever trying OMPL.
    ompl_args = [
        "--ros-args",
        "-p",
        "ompl.planning_plugins:=['ompl_interface/OMPLPlanner']",
        "-p",
        "ompl.request_adapters:=['default_planning_request_adapters/ResolveConstraintFrames','default_planning_request_adapters/ValidateWorkspaceBounds','default_planning_request_adapters/CheckStartStateBounds','default_planning_request_adapters/CheckStartStateCollision']",
        "-p",
        "ompl.response_adapters:=['default_planning_response_adapters/AddTimeOptimalParameterization','default_planning_response_adapters/ValidateSolution','default_planning_response_adapters/DisplayMotionPath']",
        "-p",
        "pilz_industrial_motion_planner.planning_plugins:=['pilz_industrial_motion_planner/CommandPlanner']",
        "-p",
        "pilz_industrial_motion_planner.request_adapters:=['default_planning_request_adapters/ResolveConstraintFrames','default_planning_request_adapters/ValidateWorkspaceBounds','default_planning_request_adapters/CheckStartStateBounds','default_planning_request_adapters/CheckStartStateCollision']",
        "-p",
        "pilz_industrial_motion_planner.response_adapters:=['default_planning_response_adapters/ValidateSolution','default_planning_response_adapters/DisplayMotionPath']",
    ]

    # Declare launch arguments. The beamline YAML is loaded by every framework
    # consumer from $BEAMBOT_BEAMLINE_CONFIG (set in the environment), not via
    # a launch arg — keeps the deployment site choice undeniable.

    declare_enable_vision = DeclareLaunchArgument(
        "enable_vision",
        default_value="true",
        description="Enable vision servers (requires Zivid camera)",
    )

    declare_enable_pipettor = DeclareLaunchArgument(
        "enable_pipettor", default_value="true", description="Enable pipettor server"
    )

    declare_use_mock_hardware = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Use fake hardware (simulation mode, no real robot)",
    )

    declare_enable_batching = DeclareLaunchArgument(
        "enable_batching",
        default_value="true",
        description="Enable MTC stage batching (false = each task via action server)",
    )

    declare_enable_tracing = DeclareLaunchArgument(
        "enable_tracing",
        default_value="false",
        description="Enable ros2_tracing (LTTng). Writes CTF trace to "
        "~/.ros/tracing/<trace_session_name>-<timestamp>/. "
        "Analyze with: ros2 run tracetools_analysis auto <path>",
    )

    declare_trace_session_name = DeclareLaunchArgument(
        "trace_session_name",
        default_value="beambot",
        description="LTTng session name (timestamp is appended automatically)",
    )

    declare_orchestrator_log_level = DeclareLaunchArgument(
        "orchestrator_log_level",
        default_value="info",
        description="Logger severity for the orchestrator node (debug|info|warn|"
        'error). Set to "warn" to suppress the ~25 INFO lines/goal '
        "on the hot path — diagnostic for the planning-latency "
        "investigation (does suppressing INFO collapse early-goal "
        'latency?). Default "info" preserves normal behavior.',
    )

    enable_vision = LaunchConfiguration("enable_vision")
    enable_pipettor = LaunchConfiguration("enable_pipettor")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    enable_batching = LaunchConfiguration("enable_batching")
    enable_tracing = LaunchConfiguration("enable_tracing")
    trace_session_name = LaunchConfiguration("trace_session_name")
    orchestrator_log_level = LaunchConfiguration("orchestrator_log_level")

    # ros2_tracing: opt-in via enable_tracing:=true. Instruments every rclcpp
    # callback, publish, take, and executor event across all nodes launched
    # after this action. Default events_ust set (see Trace() defaults) covers
    # callback_start/end + publish + subscribe which is what perf reports need.
    # events_kernel=[] because we don't have lttng-modules installed; userspace
    # tracing alone gives per-callback timings without kernel-level jitter data.
    trace_action = Trace(
        session_name=trace_session_name,
        append_timestamp=True,
        events_kernel=[],
        condition=IfCondition(enable_tracing),
    )

    # Shared IK config: same kinematics.yaml move_group loads, wrapped under the
    # key its RobotModelLoader reads. Required — without it Cartesian/IK tasks
    # (e.g. tool_exchange dock) fail with "No kinematics solver instantiated".
    from beambot.config_loader import moveit_config_package

    _kinematics_path = os.path.join(
        get_package_share_directory(moveit_config_package()),
        "config",
        "kinematics.yaml",
    )
    with open(_kinematics_path) as _f:
        _robot_description_kinematics = yaml.safe_load(_f)

    action_server_parameters = [
        {"use_sim_time": False},
        {"robot_description_kinematics": _robot_description_kinematics},
    ]

    # MoveTo action server
    move_to_server = Node(
        package="beambot",
        executable="move_to_server.py",
        name="beambot_moveto_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=ompl_args,
    )

    # EndEffector action server
    end_effector_server = Node(
        package="beambot",
        executable="end_effector_server.py",
        name="beambot_endeffector_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=ompl_args,
    )

    # ToolExchange action server
    tool_exchange_server = Node(
        package="beambot",
        executable="tool_exchange_server.py",
        name="beambot_toolexchange_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=ompl_args,
    )

    # Vision MoveTo action server (conditional)
    vision_server = Node(
        package="beambot",
        executable="vision_server.py",
        name="beambot_vision_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=ompl_args,
        condition=IfCondition(enable_vision),
    )

    # Vision Task action server (unified pipeline, issue #88; conditional).
    # Hosts migrated vision task types (v1: vision_moveto) alongside the legacy
    # vision server during the incremental migration.
    vision_task_server = Node(
        package="beambot",
        executable="vision_task_server.py",
        name="beambot_vision_task_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=ompl_args,
        condition=IfCondition(enable_vision),
    )

    # Sample action server (pick_sample + place_sample, conditional)
    sample_server = Node(
        package="beambot",
        executable="sample_server.py",
        name="beambot_sample_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=ompl_args,
        condition=IfCondition(enable_vision),
    )

    # Zivid camera node - provides /capture_and_detect_markers service (conditional)
    zivid_camera = Node(
        package="zivid_camera",
        executable="zivid_camera",
        name="zivid_camera",
        output="screen",
        parameters=[
            {
                "settings_2d_file_path": PathJoinSubstitution(
                    [FindPackageShare("beambot"), "config", "zivid_settings.yml"]
                ),
                "settings_file_path": PathJoinSubstitution(
                    [FindPackageShare("beambot"), "config", "scene_capture_noproj.yml"]
                ),
                "frame_id": "zivid_optical_frame",
            }
        ],
        condition=IfCondition(enable_vision),
    )

    # Pipettor action server (conditional)
    pipettor_server = Node(
        package="beambot",
        executable="pipettor_server.py",
        name="beambot_pipettor_server",
        output="screen",
        parameters=action_server_parameters,
        arguments=ompl_args,
        condition=IfCondition(enable_pipettor),
    )

    # Orchestrator - manages MoveIt lifecycle and task coordination
    # Now includes action_server_parameters and ompl_args because batching
    # executes MTC stages directly in this process (not via action servers)
    orchestrator = Node(
        package="beambot",
        executable="orchestrator.py",
        name="beambot_orchestrator",
        output="screen",
        parameters=action_server_parameters
        + [
            {"use_mock_hardware": use_mock_hardware},
            {"enable_batching": enable_batching},
        ],
        arguments=ompl_args
        + [
            "--ros-args",
            "--log-level",
            ["beambot_orchestrator:=", orchestrator_log_level],
        ],
    )

    return LaunchDescription(
        [
            # Launch arguments
            declare_enable_vision,
            declare_enable_pipettor,
            declare_use_mock_hardware,
            declare_enable_batching,
            declare_enable_tracing,
            declare_trace_session_name,
            declare_orchestrator_log_level,
            # Tracing (conditional - must come BEFORE any Node so tracepoints
            # in those processes are captured from process start)
            trace_action,
            # Core servers (always launched)
            move_to_server,
            end_effector_server,
            tool_exchange_server,
            # Vision servers (conditional)
            vision_server,
            vision_task_server,
            sample_server,
            # Zivid camera (conditional - provides /capture_and_detect_markers service)
            zivid_camera,
            # Pipettor server (conditional)
            pipettor_server,
            # Orchestrator (always launched)
            orchestrator,
        ]
    )
