"""Launch all mtc_py action servers.

This launch file starts all the Python MTC action servers:
- mtc_moveto_py: MoveTo operations
- mtc_endeffector_py: Gripper operations
- mtc_pickplace_py: Pick and place sequences
- mtc_toolexchange_py: Tool exchange operations
- mtc_vision_py: Vision-guided moves
- mtc_vision_pickplace_py: Vision-guided pick/place
- mtc_orchestrator_py: Central coordinator

Usage:
    ros2 launch mtc_py mtc_py_bringup.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for mtc_py servers."""

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
        executable='move_to_server_node.py',
        name='mtc_moveto_server_py',
        output='screen',
        parameters=action_server_parameters,
    )

    # EndEffector action server
    end_effector_server = Node(
        package='mtc_py',
        executable='end_effector_server_node.py',
        name='mtc_endeffector_server_py',
        output='screen',
        parameters=action_server_parameters,
    )

    # PickPlace action server
    pick_place_server = Node(
        package='mtc_py',
        executable='pick_place_server_node.py',
        name='mtc_pickplace_server_py',
        output='screen',
        parameters=action_server_parameters,
    )

    # ToolExchange action server
    tool_exchange_server = Node(
        package='mtc_py',
        executable='tool_exchange_server_node.py',
        name='mtc_toolexchange_server_py',
        output='screen',
        parameters=action_server_parameters,
    )

    # Orchestrator
    orchestrator = Node(
        package='mtc_py',
        executable='orchestrator_node.py',
        name='mtc_orchestrator_py',
        output='screen',
    )

    return LaunchDescription([
        move_to_server,
        end_effector_server,
        pick_place_server,
        tool_exchange_server,
        orchestrator,
    ])
