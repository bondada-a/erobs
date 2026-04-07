"""Unified MoveIt launch file for UR5e with parameterized gripper support.

Usage:
    ros2 launch ur5e_moveit_config robot_bringup.launch.py gripper:=epick
    ros2 launch ur5e_moveit_config robot_bringup.launch.py gripper:=hande use_fake_hardware:=true

Supported grippers: none, epick, hande, 2fg7, pipettor

This replaces 5 separate MoveIt config packages with a single parameterized package.
Uses OpaqueFunction pattern (same as UR driver upstream) to resolve gripper arg
before constructing MoveItConfigsBuilder calls.

NOTE: robot_state_publisher is NOT started here — the UR driver's ur_control.launch.py
starts its own RSP unconditionally. Starting a second one causes duplicate /tf
publications with subtly different kinematic models (#51).
"""

from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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

GRIPPER_CONFIGS = {
    "none": {
        "urdf": "ur_standalone.xacro",
        "controllers": "none/ur_controllers.yaml",
        "moveit_controllers": "none/moveit_controllers.yaml",
        "tool_voltage": "0",
        "use_tool_communication": "true",
        "tool_comm_params": {},
        "payload_mass": 1.430,
        "payload_cog": {"x": -0.038, "y": -0.022, "z": -0.055},
    },
    "epick": {
        "urdf": "ur_with_zivid_epick.xacro",
        "controllers": "epick/ur_epick_controllers.yaml",
        "moveit_controllers": "epick/moveit_controllers.yaml",
        "tool_voltage": "24",
        "use_tool_communication": "false",  # Manual tool_communication node with delay
        "tool_comm_params": {},
        "payload_mass": 2.150,
        "payload_cog": {"x": 0.018, "y": -0.015, "z": -0.036},
    },
    "hande": {
        "urdf": "ur_with_zivid_hande.xacro",
        "controllers": "hande/ur_hande_controllers.yaml",
        "moveit_controllers": "hande/moveit_controllers.yaml",
        "tool_voltage": "24",
        "use_tool_communication": "false",  # hande driver manages socat via create_socat_tty
        "tool_comm_params": {},
        "payload_mass": 2.520,
        "payload_cog": {"x": 0.018, "y": -0.013, "z": -0.031},
    },
    "2fg7": {
        "urdf": "ur_with_zivid_2fg7.xacro",
        "controllers": "2fg7/ur_2fg7_controllers.yaml",
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
        "payload_mass": 2.210,
        "payload_cog": {"x": 0.018, "y": -0.013, "z": -0.031},
    },
    "pipettor": {
        "urdf": "ur_with_zivid_pipettor.xacro",
        "controllers": "pipettor/ur_pipettor_controllers.yaml",
        "moveit_controllers": "pipettor/moveit_controllers.yaml",
        "tool_voltage": "24",
        "use_tool_communication": "true",
        "tool_comm_params": {},
        "payload_mass": 1.630,
        "payload_cog": {"x": 0.010, "y": -0.010, "z": -0.020},
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
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)

    if gripper not in GRIPPER_CONFIGS:
        raise ValueError(
            f"Unknown gripper '{gripper}'. "
            f"Supported: {list(GRIPPER_CONFIGS.keys())}"
        )

    config = GRIPPER_CONFIGS[gripper]
    pkg_share = get_package_share_directory("ur5e_moveit_config")
    desc_share = get_package_share_directory("ur5e_robot_description")

    # ── Xacro args ──────────────────────────────────────────────────────
    xacro_args = {
        "name": ur_type,
        "ur_type": ur_type,
        "tf_prefix": "",
    }

    # Gripper-specific xacro args
    if gripper == "epick":
        for arg in ["extension_length", "extension_radius",
                     "suction_cup_height", "suction_cup_radius"]:
            xacro_args[arg] = LaunchConfiguration(arg).perform(context)
    elif gripper == "hande":
        xacro_args["socat_ip_address"] = robot_ip

    # ── UR driver ───────────────────────────────────────────────────────
    ur_launch_args = {
        "ur_type": ur_type,
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "launch_rviz": "false",
        "description_package": "ur_description",
        "description_file": os.path.join(desc_share, "urdf", config["urdf"]),
        "controllers_file": os.path.join(pkg_share, "config", config["controllers"]),
        "kinematics_params_file": os.path.join(
            desc_share, "config", "ur5e_calibration.yaml"),
        "use_tool_communication": config["use_tool_communication"],
        "tool_voltage": config["tool_voltage"],
    }
    # RS485 params (2fg7 needs 1Mbps/Even)
    ur_launch_args.update(config["tool_comm_params"])

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch", "ur_control.launch.py"
            ])
        ]),
        launch_arguments=ur_launch_args.items(),
    )

    # ── MoveIt config ───────────────────────────────────────────────────
    # joint_limits and trajectory_execution use absolute paths because
    # MoveItConfigsBuilder's default inference looks for config/joint_limits.yaml
    # at the package root, but our per-gripper files are in config/<gripper>/.
    moveit_config = (
        MoveItConfigsBuilder("ur_moveit", package_name="ur5e_moveit_config")
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
            publish_robot_description=True,
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
    )

    # ── Payload ─────────────────────────────────────────────────────────
    cog = config["payload_cog"]
    set_payload = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call",
                    "/io_and_status_controller/set_payload",
                    "ur_msgs/srv/SetPayload",
                    f"{{mass: {config['payload_mass']}, "
                    f"center_of_gravity: {{x: {cog['x']}, y: {cog['y']}, z: {cog['z']}}}}}",
                ],
                output="screen",
            )
        ],
    )

    # ── Build launch actions list ───────────────────────────────────────
    actions = []

    # ePick special case: manual tool_communication node + delayed ur_control
    if gripper == "epick":
        tool_communication = Node(
            package="ur_robot_driver",
            executable="tool_communication.py",
            name="tool_communication_node",
            output="screen",
            parameters=[{"robot_ip": robot_ip}],
        )
        delayed_ur_control = TimerAction(
            period=1.5,  # Wait for tool_communication to create /tmp/ttyUR via socat
            actions=[ur_control_launch],
        )
        actions.append(tool_communication)
        actions.append(delayed_ur_control)
    else:
        actions.append(ur_control_launch)

    # Common nodes
    actions.append(run_move_group_node)
    actions.append(rviz_node)
    actions.append(set_payload)

    # ── Gripper-specific nodes ──────────────────────────────────────────
    if gripper == "epick":
        actions.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["epick_gripper_action_controller", "-c", "/controller_manager"],
        ))
        actions.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["epick_status_publisher_controller", "-c", "/controller_manager"],
        ))
    elif gripper == "hande":
        actions.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_action_controller", "-c", "/controller_manager"],
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
                        "use_fake_hardware": use_fake_hardware,
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
                        {"use_fake_hardware": use_fake_hardware},
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
            description="Gripper type: none, epick, hande, 2fg7, pipettor",
        ),
        DeclareLaunchArgument(
            "robot_ip", default_value="192.168.1.10",
        ),
        DeclareLaunchArgument(
            "ur_type", default_value="ur5e",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="false",
        ),
        # ePick cup profile args (only used when gripper:=epick)
        DeclareLaunchArgument(
            "extension_length", default_value="0.013",
            description="ePick suction cup extension length (m)",
        ),
        DeclareLaunchArgument(
            "extension_radius", default_value="0.004",
            description="ePick suction cup extension radius (m)",
        ),
        DeclareLaunchArgument(
            "suction_cup_height", default_value="0.003",
            description="ePick suction cup height (m)",
        ),
        DeclareLaunchArgument(
            "suction_cup_radius", default_value="0.0015",
            description="ePick suction cup radius (m)",
        ),
        # ── OpaqueFunction resolves gripper then builds all nodes ───────
        OpaqueFunction(function=launch_setup),
    ])
