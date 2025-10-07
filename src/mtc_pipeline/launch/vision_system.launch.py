#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    launch_apriltag_arg = DeclareLaunchArgument(
        'launch_apriltag',
        default_value='true',
        description='Launch AprilTag detector'
    )

    camera_namespace_arg = DeclareLaunchArgument(
        'camera_namespace',
        default_value='/zivid_camera',
        description='Camera namespace for image topics'
    )

    tag_config_file_arg = DeclareLaunchArgument(
        'tag_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mtc_pipeline'),
            'config',
            'apriltag_config.yaml'
        ]),
        description='Path to AprilTag configuration file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_apriltag = LaunchConfiguration('launch_apriltag')
    camera_namespace = LaunchConfiguration('camera_namespace')
    tag_config_file = LaunchConfiguration('tag_config_file')

    # Vision action server (always launched)
    vision_action_server = Node(
        package='mtc_pipeline',
        executable='vision_action_server',
        name='vision_action_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
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
    )

    # AprilTag detector node (conditional)
    apriltag_detector = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_detector',
        namespace='',
        output='screen',
        condition=IfCondition(launch_apriltag),
        parameters=[tag_config_file],
        remappings=[
            # Remap to camera topics
            ('image_rect', PathJoinSubstitution([camera_namespace, 'color', 'image_raw'])),
            ('camera_info', PathJoinSubstitution([camera_namespace, 'color', 'camera_info'])),
            # Output topics
            ('detections', '/apriltag/detections'),
            ('detection_image', '/apriltag/detection_image')
        ]
    )

    # Optional: Visualization node for debugging
    tag_visualization = Node(
        package='rviz2',
        executable='rviz2',
        name='apriltag_visualization',
        output='screen',
        condition=IfCondition('false'),  # Set to 'true' to enable visualization
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('mtc_pipeline'),
            'config',
            'apriltag_visualization.rviz'
        ])]
    )

    # Log information
    log_info = LogInfo(
        msg=['Vision system launching with AprilTag detection: ', launch_apriltag]
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        launch_apriltag_arg,
        camera_namespace_arg,
        tag_config_file_arg,

        # Actions
        log_info,
        vision_action_server,
        apriltag_detector,
        tag_visualization
    ])