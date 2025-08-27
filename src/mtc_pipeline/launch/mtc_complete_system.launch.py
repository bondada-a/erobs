#!/usr/bin/env python3

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    task_file_arg = DeclareLaunchArgument(
        'task_file',
        description='Path to the JSON task file'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.56.101',
        description='Robot IP address'
    )
    
    gripper_type_arg = DeclareLaunchArgument(
        'gripper_type',
        default_value='none',
        description='Type of gripper (hande, epick, none)'
    )
    
    # Action server node
    action_server_node = Node(
        package='mtc_pipeline',
        executable='mtc_orchestrator_action_server',
        name='mtc_orchestrator_action_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
        ]
    )
    
    # Client node - launched with a delay to ensure action server is ready
    client_node = Node(
        package='mtc_pipeline',
        executable='mtc_action_client_example',
        name='mtc_action_client',
        output='screen',
        parameters=[
            {'use_sim_time': False},
        ],
        arguments=[
            LaunchConfiguration('task_file'),
            LaunchConfiguration('robot_ip'),
            LaunchConfiguration('gripper_type')
        ],
        # Ensure this node runs in the same context as the action server
        namespace='',
        remappings=[
            # Remap to ensure we're using the same topics
            ('/joint_states', '/joint_states'),
            ('/robot_description', '/robot_description'),
            ('/robot_description_semantic', '/robot_description_semantic'),
        ]
    )
    
    # Launch client with a delay to ensure action server is ready
    delayed_client = TimerAction(
        period=5.0,  # Wait 5 seconds for action server to be ready
        actions=[client_node]
    )
    
    # Add logging
    log_info = LogInfo(
        msg=['Launching complete MTC system with task file: ', LaunchConfiguration('task_file')]
    )
    
    return LaunchDescription([
        task_file_arg,
        robot_ip_arg,
        gripper_type_arg,
        log_info,
        action_server_node,
        delayed_client
    ])
