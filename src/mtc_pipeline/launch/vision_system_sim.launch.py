#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    tag_ids_arg = DeclareLaunchArgument(
        'tag_ids',
        default_value='[0, 1, 2]',
        description='List of tag IDs to simulate'
    )

    moving_tags_arg = DeclareLaunchArgument(
        'moving_tags',
        default_value='false',
        description='Enable moving tags for dynamic simulation'
    )

    movement_radius_arg = DeclareLaunchArgument(
        'movement_radius',
        default_value='0.05',
        description='Radius of circular movement for tags (meters)'
    )

    movement_speed_arg = DeclareLaunchArgument(
        'movement_speed',
        default_value='0.3',
        description='Speed of tag movement (radians/second)'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    tag_ids = LaunchConfiguration('tag_ids')
    moving_tags = LaunchConfiguration('moving_tags')
    movement_radius = LaunchConfiguration('movement_radius')
    movement_speed = LaunchConfiguration('movement_speed')

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

    # Mock AprilTag detector for simulation
    mock_detector = Node(
        package='mtc_pipeline',
        executable='mock_apriltag_detector',
        name='mock_apriltag_detector',
        output='screen',
        parameters=[{
            'tag_ids': tag_ids,
            'publish_moving_tags': moving_tags,
            'movement_radius': movement_radius,
            'movement_speed': movement_speed,
            'camera_frame': 'zivid_optical_frame'
        }]
    )

    # Static transform publisher for camera frame (if not already published)
    # This simulates the camera being mounted on the robot
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_transform',
        arguments=['0.1', '0.0', '0.5', '0', '0', '0', 'base_link', 'zivid_optical_frame']
    )

    # Log information
    log_info = LogInfo(
        msg='Vision system simulation started with mock AprilTag detector'
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        tag_ids_arg,
        moving_tags_arg,
        movement_radius_arg,
        movement_speed_arg,

        # Actions
        log_info,
        vision_action_server,
        mock_detector,
        camera_tf
    ])