#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate launch description for MTC GUI Client"""
    
    # Launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.101',
        description='Robot IP address'
    )
    
    # MTC GUI Client node
    gui_client_node = Node(
        package='mtc_gui',
        executable='mtc_gui_client',
        name='mtc_gui_client',
        output='screen',
        parameters=[
            {'robot_ip': LaunchConfiguration('robot_ip')}
        ]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        gui_client_node
    ])
