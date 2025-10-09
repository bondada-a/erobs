#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch argument for enabling AprilTag detector
    launch_apriltag_arg = DeclareLaunchArgument(
        'launch_apriltag',
        default_value='true',
        description='Launch AprilTag detector node'
    )

    launch_apriltag = LaunchConfiguration('launch_apriltag')

    # Action server parameters - same pattern as modular_action_servers.launch.py
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

    # Vision action server
    vision_action_server = Node(
        package='mtc_pipeline',
        executable='vision_action_server',
        name='vision_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    # AprilTag detector (optional)
    apriltag_detector = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_detector',
        output='screen',
        condition=IfCondition(launch_apriltag),
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

    return LaunchDescription([
        launch_apriltag_arg,
        vision_action_server,
        apriltag_detector
    ])
