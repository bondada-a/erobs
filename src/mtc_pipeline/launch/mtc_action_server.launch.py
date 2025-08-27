#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('mtc_pipeline')
    
    # Launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.101',
        description='Robot IP address'
    )
    
    # Launch the MTC Action Server
    mtc_action_server_node = Node(
        package='mtc_pipeline',
        executable='mtc_orchestrator_action_server',
        name='mtc_orchestrator_action_server',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip')
        }]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        mtc_action_server_node
    ])
