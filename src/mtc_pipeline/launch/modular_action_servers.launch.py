#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.56.101',
        description='Robot IP address'
    )

    # Action servers need kinematics for Cartesian planning
    # Use arm-only kinematics (works with all grippers)
    # Robot URDF comes from /robot_description topic published by move_group
    action_server_parameters = [
        {'use_sim_time': False},
        {
            'robot_description_kinematics': {
                'ur_arm': {
                    'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                    'kinematics_solver_search_resolution': 0.001,
                    'kinematics_solver_timeout': 0.1,
                    'kinematics_solver_attempts': 3
                }
            }
        }
    ]

    # Modular Action Servers - connect to MoveIt managed by orchestrator
    pickplace_action_server = Node(
        package='mtc_pipeline',
        executable='pickplace_action_server',
        name='pickplace_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    toolexchange_action_server = Node(
        package='mtc_pipeline',
        executable='toolexchange_action_server',
        name='toolexchange_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    moveto_action_server = Node(
        package='mtc_pipeline',
        executable='moveto_action_server',
        name='moveto_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    end_effector_action_server = Node(
        package='mtc_pipeline',
        executable='end_effector_action_server',
        name='end_effector_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    # Main Orchestrator - manages MoveIt lifecycle
    orchestrator = Node(
        package='mtc_pipeline',
        executable='mtc_orchestrator_action_server',
        name='mtc_orchestrator_action_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'robot_ip': LaunchConfiguration('robot_ip')},
        ]
    )

    return LaunchDescription([
        robot_ip_arg,
        pickplace_action_server,
        toolexchange_action_server,
        moveto_action_server,
        end_effector_action_server,
        orchestrator,
    ])
