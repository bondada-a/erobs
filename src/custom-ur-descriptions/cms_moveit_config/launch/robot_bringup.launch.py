"""Unified MoveIt launch file for UR5e with parameterized gripper support.

Usage:
    ros2 launch cms_moveit_config robot_bringup.launch.py gripper:=epick
    ros2 launch cms_moveit_config robot_bringup.launch.py gripper:=hande use_mock_hardware:=true

Supported grippers: none, epick, hande, 2fg7, pipettor

This replaces 5 separate MoveIt config packages with a single parameterized package.
Uses OpaqueFunction pattern (same as UR driver upstream) to resolve gripper arg
before constructing MoveItConfigsBuilder calls.

NOTE: robot_state_publisher is NOT started here — the UR driver's ur_control.launch.py
starts its own RSP unconditionally. Starting a second one causes duplicate /tf
publications with subtly different kinematic models (#51).
"""

from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


# ─── Per-gripper configuration ──────────────────────────────────────────────
# Each entry defines everything that differs between gripper configurations.
#
# ros2_control controllers: every gripper loads the SAME shared base file
# (ur_base_controllers.yaml = the full UR controller set, gripper-agnostic).
# Gripper-specific controllers are added by the per-gripper spawner Nodes below
# via --param-file <gripper>_controllers.yaml. This replaces 5 near-identical
# ~220-line copies, so the next ur_robot_driver controller-set change is a
# one-file edit (see #86).

_BASE_CONTROLLERS = "ur_base_controllers.yaml"

GRIPPER_CONFIGS = {
    "none": {
        "urdf": "ur_standalone.xacro",
        "controllers": _BASE_CONTROLLERS,
        "moveit_controllers": "none/moveit_controllers.yaml",
        "tool_voltage": "0",
        "use_tool_communication": "true",
        "tool_comm_params": {},
    },
    "epick": {
        "urdf": "ur_with_zivid_epick.xacro",
        "controllers": _BASE_CONTROLLERS,
        "moveit_controllers": "epick/moveit_controllers.yaml",
        "tool_voltage": "24",
        "use_tool_communication": "true",
        "tool_comm_params": {
            "tool_baud_rate": "115200",
            "tool_parity": "0",
            "tool_stop_bits": "1",
            "tool_rx_idle_chars": "1.5",
            "tool_tx_idle_chars": "3.5",
        },
    },
    "hande": {
        "urdf": "ur_with_zivid_hande.xacro",
        "controllers": _BASE_CONTROLLERS,
        "moveit_controllers": "hande/moveit_controllers.yaml",
        "tool_voltage": "24",
        "use_tool_communication": "false",  # hande driver manages socat via create_socat_tty
        "tool_comm_params": {},
    },
    "2fg7": {
        "urdf": "ur_with_zivid_2fg7.xacro",
        "controllers": _BASE_CONTROLLERS,
        "moveit_controllers": "2fg7/moveit_controllers.yaml",
        "tool_voltage": "24",
        "use_tool_communication": "true",
        "tool_comm_params": {
            "tool_baud_rate": "1000000",
            "tool_parity": "2",
            "tool_stop_bits": "1",
            "tool_rx_idle_chars": "1.5",
            "tool_tx_idle_chars": "3.5",
        },
    },
    "pipettor": {
        "urdf": "ur_with_zivid_pipettor.xacro",
        "controllers": _BASE_CONTROLLERS,
        "moveit_controllers": "pipettor/moveit_controllers.yaml",
        "tool_voltage": "24",
        "use_tool_communication": "true",
        "tool_comm_params": {},
    },
}


def launch_setup(context, *args, **kwargs):
    """Resolve gripper arg and build all launch actions.

    This runs inside OpaqueFunction so LaunchConfiguration values are available
    as plain strings — required because MoveItConfigsBuilder.load_yaml() resolves
    eagerly and needs string paths, not LaunchConfiguration substitutions.
    """
    gripper = LaunchConfiguration("gripper").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    ur_type = LaunchConfiguration("ur_type").perform(context)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)

    if gripper not in GRIPPER_CONFIGS:
        raise ValueError(
            f"Unknown gripper '{gripper}'. "
            f"Supported: {list(GRIPPER_CONFIGS.keys())}"
        )

    config = GRIPPER_CONFIGS[gripper]
    pkg_share = get_package_share_directory("cms_moveit_config")
    desc_share = get_package_share_directory("cms_robot_description")

    # ── Xacro args ──────────────────────────────────────────────────────
    xacro_args = {
        "name": ur_type,
        "ur_type": ur_type,
        "tf_prefix": tf_prefix,
        "robot_ip": robot_ip,
    }

    # Gripper-specific xacro args
    if gripper == "epick":
        xacro_args["cup_profile"] = LaunchConfiguration("cup_profile").perform(context)
    elif gripper == "hande":
        xacro_args["socat_ip_address"] = robot_ip

    # ── UR driver ───────────────────────────────────────────────────────
    ur_launch_args = {
        "ur_type": ur_type,
        "robot_ip": robot_ip,
        "tf_prefix": tf_prefix,
        "use_mock_hardware": use_mock_hardware,
        "launch_rviz": "false",
        "description_package": "ur_description",
        "description_file": os.path.join(desc_share, "urdf", config["urdf"]),
        "controllers_file": os.path.join(pkg_share, "config", config["controllers"]),
        "kinematics_params_file": os.path.join(
            desc_share, "config", "ur5e_calibration.yaml"),
        "use_tool_communication": config["use_tool_communication"],
        "tool_voltage": config["tool_voltage"],
        # Jazzy: hardware loads async, so spawners need longer timeout to avoid
        # retry cycles while controller_manager is busy initializing.
        "controller_spawner_timeout": "30",
    }
    # RS485 params (2fg7 needs 1Mbps/Even)
    ur_launch_args.update(config["tool_comm_params"])

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur_robot_driver"),
                "launch", "ur_control.launch.py",
            )
        ),
        launch_arguments=ur_launch_args.items(),
    )

    # ── MoveIt config ───────────────────────────────────────────────────
    # joint_limits and trajectory_execution use absolute paths because
    # MoveItConfigsBuilder's default inference looks for config/joint_limits.yaml
    # at the package root, but our per-gripper files are in config/<gripper>/.
    moveit_config = (
        MoveItConfigsBuilder("ur_moveit", package_name="cms_moveit_config")
        .robot_description(
            file_path=os.path.join(desc_share, "urdf", config["urdf"]),
            mappings=xacro_args,
        )
        .robot_description_semantic(
            file_path=os.path.join(pkg_share, "srdf", "ur.srdf.xacro"),
            mappings={"gripper": gripper},
        )
        .joint_limits(
            file_path=os.path.join(
                pkg_share, "config", gripper, "joint_limits.yaml"),
        )
        .trajectory_execution(
            file_path=os.path.join(
                pkg_share, "config", config["moveit_controllers"]),
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description_kinematics,
            moveit_config.to_dict(),
            move_group_capabilities,
            # Wait for joint_states before declaring ready (controllers load async in Jazzy)
            {"planning_scene_monitor_options.wait_for_initial_state_timeout": 30.0},
        ],
    )

    # ── RViz ────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_share, "rviz", "view_robot_mtc.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        # The orchestrator imports cv2 (YOLO warm-up) before launching this
        # subprocess. opencv-python's config-3.py forcibly sets
        # QT_QPA_PLATFORM_PLUGIN_PATH to its bundled Qt5 plugins, which RViz2
        # (built against system Qt6) then loads and aborts on (ABI mismatch).
        # Override it back to the system Qt6 platforms dir for RViz only.
        additional_env={
            "QT_QPA_PLATFORM_PLUGIN_PATH": "/usr/lib/x86_64-linux-gnu/qt6/plugins/platforms",
        },
    )

    # Payload is set by the orchestrator's lifecycle manager after launch
    # (readiness-gated via the set_payload service), not here.

    # ── Build launch actions list ───────────────────────────────────────
    actions = [ur_control_launch]

    # Common nodes
    actions.append(run_move_group_node)
    actions.append(rviz_node)

    # ── Gripper-specific nodes ──────────────────────────────────────────
    # Gripper controllers are NOT in the shared base controllers file; each
    # spawner carries its controller's type+params via --param-file overlay
    # (config/<gripper>_controllers.yaml). The spawner reads the overlay with
    # plain yaml.safe_load — no $(var ...) expansion — so overlays use literal
    # values. See #86.
    if gripper == "epick":
        # One spawner for both ePick controllers (native mode takes multiple
        # names) — shares a single load/switch and removes any ordering ambiguity.
        actions.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["epick_gripper_action_controller", "epick_status_publisher_controller",
                       "-c", "/controller_manager",
                       "--param-file", os.path.join(pkg_share, "config", "epick_controllers.yaml")],
        ))
    elif gripper == "hande":
        actions.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_action_controller", "-c", "/controller_manager",
                       "--param-file", os.path.join(pkg_share, "config", "hande_controllers.yaml")],
        ))
    elif gripper == "2fg7":
        actions.append(TimerAction(
            period=3.0,  # Wait for tool_communication to create /tmp/ttyUR
            actions=[
                Node(
                    package="onrobot_2fg7_driver",
                    executable="onrobot_2fg7_driver_node",
                    name="onrobot_2fg7_driver",
                    output="screen",
                    parameters=[{
                        "serial_port": "/tmp/ttyUR",
                        "slave_id": 65,
                        "baudrate": 1000000,
                        "use_fake_hardware": use_mock_hardware == "true",
                    }],
                ),
            ],
        ))
    elif gripper == "pipettor":
        actions.append(TimerAction(
            period=3.0,  # Wait for tool_communication to create /tmp/ttyUR
            actions=[
                Node(
                    package="pipette_driver",
                    executable="pipette_driver_node",
                    name="pipette_driver_node",
                    output="screen",
                    parameters=[
                        {"serial_port": "/tmp/ttyUR"},
                        {"use_fake_hardware": use_mock_hardware == "true"},
                    ],
                ),
            ],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        # ── Arguments ───────────────────────────────────────────────────
        DeclareLaunchArgument(
            "gripper", default_value="none",
            choices=list(GRIPPER_CONFIGS.keys()),
            description="Gripper type: none, epick, hande, 2fg7, pipettor",
        ),
        DeclareLaunchArgument(
            "robot_ip", default_value="192.168.1.101",
            description="Default matches CMS beamline YAML; orchestrator always "
                        "overrides via robot_ip:= from $BEAMBOT_BEAMLINE_CONFIG.",
        ),
        DeclareLaunchArgument(
            "ur_type", default_value="ur5e",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="false",
        ),
        DeclareLaunchArgument(
            "tf_prefix", default_value="",
            description="Joint/link name prefix; must match controllers' $(var tf_prefix).",
        ),
        # ePick cup profile (only used when gripper:=epick).
        # Profile name resolves to dimensions via suction_cups.yaml in the xacro.
        DeclareLaunchArgument(
            "cup_profile", default_value="3mm_dia",
            description="ePick suction cup profile name (from suction_cups.yaml)",
        ),
        # ── OpaqueFunction resolves gripper then builds all nodes ───────
        OpaqueFunction(function=launch_setup),
    ])
