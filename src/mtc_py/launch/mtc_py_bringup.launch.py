"""Launch all mtc_py action servers.

This launch file starts all the Python MTC action servers:
- mtc_moveto_py: MoveTo operations
- mtc_endeffector_py: Gripper operations
- mtc_pickplace_py: Pick and place sequences
- mtc_toolexchange_py: Tool exchange operations
- mtc_vision_moveto_py: Vision-guided moves
- mtc_vision_pickplace_py: Vision-guided pick/place
- mtc_pipettor_py: Pipettor operations
- mtc_orchestrator_py: Central coordinator

Usage:
    ros2 launch mtc_py mtc_py_bringup.launch.py
    ros2 launch mtc_py mtc_py_bringup.launch.py enable_vision:=false
    ros2 launch mtc_py mtc_py_bringup.launch.py beamline_config:=config/ur3e_beamline.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for mtc_py servers."""

    # OMPL parameters for PipelinePlanner
    # Required because Python's rclcpp.Node binding lacks declare_parameter().
    # Without these, PipelinePlanner falls back to CHOMP instead of OMPL.
    ompl_args = [
        '--ros-args',
        '-p', 'ompl.planning_plugin:=ompl_interface/OMPLPlanner',
        '-p', 'ompl.request_adapters:=default_planner_request_adapters/AddTimeOptimalParameterization',
    ]

    # Declare launch arguments
    declare_beamline_config = DeclareLaunchArgument(
        'beamline_config',
        default_value='config/default_beamline.yaml',
        description='Path to beamline configuration YAML (relative to mtc_py package)'
    )

    declare_enable_vision = DeclareLaunchArgument(
        'enable_vision',
        default_value='true',
        description='Enable vision servers (requires Zivid camera)'
    )

    declare_enable_pipettor = DeclareLaunchArgument(
        'enable_pipettor',
        default_value='true',
        description='Enable pipettor server'
    )

    enable_vision = LaunchConfiguration('enable_vision')
    enable_pipettor = LaunchConfiguration('enable_pipettor')

    # Resolve beamline config path (relative to mtc_py package)
    beamline_config_path = PathJoinSubstitution([
        FindPackageShare('mtc_py'),
        LaunchConfiguration('beamline_config')
    ])

    # Action servers need kinematics for Cartesian planning (matches C++ mtc_bringup.launch.py)
    # Robot URDF comes from /robot_description topic published by move_group
    action_server_parameters = [
        {'use_sim_time': False},
        {
            'robot_description_kinematics': {
                'ur_arm': {
                    'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                    'kinematics_solver_search_resolution': 0.001,
                    'kinematics_solver_timeout': 1.0,
                    'kinematics_solver_attempts': 10
                }
            }
        }
    ]

    # MoveTo action server
    move_to_server = Node(
        package='mtc_py',
        executable='move_to_server.py',
        name='mtc_moveto_server_py',
        output='screen',
        parameters=action_server_parameters,
        arguments=ompl_args,
    )

    # EndEffector action server
    end_effector_server = Node(
        package='mtc_py',
        executable='end_effector_server.py',
        name='mtc_endeffector_server_py',
        output='screen',
        parameters=action_server_parameters,
        arguments=ompl_args,
    )

    # PickPlace action server
    pick_place_server = Node(
        package='mtc_py',
        executable='pick_place_server.py',
        name='mtc_pickplace_server_py',
        output='screen',
        parameters=action_server_parameters,
        arguments=ompl_args,
    )

    # ToolExchange action server
    tool_exchange_server = Node(
        package='mtc_py',
        executable='tool_exchange_server.py',
        name='mtc_toolexchange_server_py',
        output='screen',
        parameters=action_server_parameters,
        arguments=ompl_args,
    )

    # Vision MoveTo action server (conditional)
    vision_server = Node(
        package='mtc_py',
        executable='vision_server.py',
        name='mtc_vision_server_py',
        output='screen',
        parameters=action_server_parameters,
        arguments=ompl_args,
        condition=IfCondition(enable_vision),
    )

    # Vision PickPlace action server (conditional)
    vision_pick_place_server = Node(
        package='mtc_py',
        executable='vision_pick_place_server.py',
        name='mtc_vision_pickplace_server_py',
        output='screen',
        parameters=action_server_parameters,
        arguments=ompl_args,
        condition=IfCondition(enable_vision),
    )

    # Pipettor action server (conditional)
    pipettor_server = Node(
        package='mtc_py',
        executable='pipettor_server.py',
        name='mtc_pipettor_server_py',
        output='screen',
        parameters=action_server_parameters,
        arguments=ompl_args,
        condition=IfCondition(enable_pipettor),
    )

    # Orchestrator - manages MoveIt lifecycle and task coordination
    orchestrator = Node(
        package='mtc_py',
        executable='orchestrator.py',
        name='mtc_orchestrator_py',
        output='screen',
        parameters=[
            {'beamline_config': beamline_config_path},
        ],
    )

    return LaunchDescription([
        # Launch arguments
        declare_beamline_config,
        declare_enable_vision,
        declare_enable_pipettor,
        # Core servers (always launched)
        move_to_server,
        end_effector_server,
        pick_place_server,
        tool_exchange_server,
        # Vision servers (conditional)
        vision_server,
        vision_pick_place_server,
        # Pipettor server (conditional)
        pipettor_server,
        # Orchestrator (always launched)
        orchestrator,
    ])
