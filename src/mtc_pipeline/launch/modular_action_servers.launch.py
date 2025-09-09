#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.56.101',
        description='Robot IP address'
    )
    
    gripper_arg = DeclareLaunchArgument(
        'gripper',
        default_value='hande',
        description='Gripper type (hande, epick, none)'
    )
    
    launch_moveit_arg = DeclareLaunchArgument(
        'launch_moveit',
        default_value='true',
        description='Whether to launch MoveIt move_group'
    )
    
    # Get launch configuration
    robot_ip = LaunchConfiguration('robot_ip')
    gripper = LaunchConfiguration('gripper')
    launch_moveit = LaunchConfiguration('launch_moveit')
    
    # Determine MoveIt config based on gripper
    moveit_config = PythonExpression([
        "'ur_zivid_hande_moveit_config' if '", gripper, "' == 'hande' else ",
        "'ur_zivid_epick_moveit_config' if '", gripper, "' == 'epick' else ",
        "'ur_standalone_moveit_config'"
    ])
    
    # Launch MoveIt move_group (conditional)
    move_group_launch = ExecuteProcess(
        condition=IfCondition(launch_moveit),
        cmd=[
            'bash', '-c',
            f'source /home/aditya/work/github_ws/erobs/install/setup.bash && '
            f'ros2 launch {moveit_config} move_group.launch.py robot_ip:={robot_ip}'
        ],
        output='screen'
    )
    
    # Modular Action Servers
    pickplace_action_server = Node(
        package='mtc_pipeline',
        executable='pickplace_action_server',
        name='pickplace_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    toolexchange_action_server = Node(
        package='mtc_pipeline',
        executable='toolexchange_action_server',
        name='toolexchange_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    moveto_action_server = Node(
        package='mtc_pipeline',
        executable='moveto_action_server',
        name='moveto_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    endeffector_action_server = Node(
        package='mtc_pipeline',
        executable='endeffector_action_server',
        name='endeffector_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    # Main Orchestrator (without embedded action servers)
    orchestrator = Node(
        package='mtc_pipeline',
        executable='mtc_orchestrator_action_server',
        name='mtc_orchestrator_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'gripper': gripper,
            'robot_ip': robot_ip,
        }]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        gripper_arg,
        launch_moveit_arg,
        move_group_launch,
        pickplace_action_server,
        toolexchange_action_server,
        moveto_action_server,
        endeffector_action_server,
        orchestrator,
    ])
