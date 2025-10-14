#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    pick_place_action_server = Node(
        package='mtc_pipeline',
        executable='pick_place_action_server',
        name='pick_place_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    tool_exchange_action_server = Node(
        package='mtc_pipeline',
        executable='tool_exchange_action_server',
        name='tool_exchange_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    move_to_action_server = Node(
        package='mtc_pipeline',
        executable='move_to_action_server',
        name='move_to_action_server',
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

    vision_action_server = Node(
        package='mtc_pipeline',
        executable='vision_action_server',
        name='vision_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    # AprilTag detector for vision-based tasks
    apriltag_detector = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_detector',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('mtc_pipeline'),
            'config',
            'apriltag_config.yaml'
        ])],
        remappings=[
            ('image_rect', '/color/image_color'),
            ('camera_info', '/color/camera_info'),
        ]
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
        pick_place_action_server,
        tool_exchange_action_server,
        move_to_action_server,
        end_effector_action_server,
        vision_action_server,
        apriltag_detector,
        orchestrator,
    ])
