#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    
    # Client node with proper parameter inheritance
    client_node = Node(
        package='mtc_pipeline',
        executable='mtc_action_client_example',
        name='mtc_action_client',
        output='screen',
        parameters=[
            # Inherit robot description parameters from the MoveIt configuration
            {'use_sim_time': False},
        ],
        arguments=[
            LaunchConfiguration('task_file'),
            LaunchConfiguration('robot_ip'),
            LaunchConfiguration('gripper_type')
        ],
        # Ensure this node runs in the same context as the MoveIt configuration
        namespace='',
        remappings=[
            # Remap to ensure we're using the same topics as the MoveIt configuration
            ('/joint_states', '/joint_states'),
            ('/robot_description', '/robot_description'),
            ('/robot_description_semantic', '/robot_description_semantic'),
        ]
    )
    
    # Add logging
    log_info = LogInfo(
        msg=['Launching MTC Action Client with task file: ', LaunchConfiguration('task_file')]
    )
    
    return LaunchDescription([
        task_file_arg,
        robot_ip_arg,
        gripper_type_arg,
        log_info,
        client_node
    ])
