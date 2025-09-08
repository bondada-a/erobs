#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.56.101',
        description='IP address of the robot'
    )
    
    # Get launch configuration
    robot_ip = LaunchConfiguration('robot_ip')
    
    # MoveTo Action Server with proper planning parameters
    moveto_action_server = Node(
        package='mtc_pipeline',
        executable='moveto_action_server',
        name='moveto_action_server',
        output='screen',
        parameters=[{
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization',
            'start_state_max_bounds_error': 0.1,
            'robot_ip': robot_ip
        }]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        moveto_action_server,
    ])
