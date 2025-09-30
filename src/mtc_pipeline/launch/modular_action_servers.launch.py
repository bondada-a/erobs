#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    # Get evaluated launch configuration
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    gripper = LaunchConfiguration('gripper').perform(context)
    launch_moveit = LaunchConfiguration('launch_moveit').perform(context)

    # Determine MoveIt config package name based on gripper
    if gripper == 'hande':
        moveit_config_pkg_name = 'ur_zivid_hande_moveit_config'
    elif gripper == 'epick':
        moveit_config_pkg_name = 'ur_zivid_epick_moveit_config'
    else:
        moveit_config_pkg_name = 'ur_standalone_moveit_config'

    # Load MoveIt configs
    moveit_configs = MoveItConfigsBuilder(
        robot_name="ur", package_name=moveit_config_pkg_name
    ).to_moveit_configs()

    # Launch MoveIt move_group (conditional)
    move_group_launch = ExecuteProcess(
        condition=IfCondition(launch_moveit),
        cmd=[
            'bash', '-c',
            [f'source /home/aditya/work/github_ws/erobs/install/setup.bash && ros2 launch {moveit_config_pkg_name} move_group.launch.py robot_ip:={robot_ip}']
        ],
        output='screen'
    )

    # Action servers need kinematics configs and SRDF but NOT URDF
    # Robot description (URDF) comes from the currently running MoveIt
    action_server_parameters = [
        {'use_sim_time': False},
        # Include SRDF (semantic description) - planning groups, end effectors, etc.
        moveit_configs.robot_description_semantic,
        # Include kinematics solvers and configuration
        moveit_configs.robot_description_kinematics,
        moveit_configs.planning_pipelines,
        moveit_configs.trajectory_execution,
        moveit_configs.planning_scene_monitor,
    ]

    # Modular Action Servers
    pickplace_action_server = Node(
        package='mtc_pipeline',
        executable='pickplace_action_server',
        name='pickplace_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    toolexchange_action_server = Node(
        package='mtc_pipeline',
        executable='toolexchange_action_server',
        name='toolexchange_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    moveto_action_server = Node(
        package='mtc_pipeline',
        executable='moveto_action_server',
        name='moveto_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    endeffector_action_server = Node(
        package='mtc_pipeline',
        executable='endeffector_action_server',
        name='endeffector_action_server',
        output='screen',
        parameters=action_server_parameters
    )
    
    # Main Orchestrator
    orchestrator = Node(
        package='mtc_pipeline',
        executable='mtc_orchestrator_action_server',
        name='mtc_orchestrator_action_server',
        output='screen',
        parameters=[
            moveit_configs.to_dict(),
            {'use_sim_time': False},
            {'gripper': gripper},
            {'robot_ip': robot_ip},
        ]
    )

    nodes_to_launch = [
        move_group_launch,
        pickplace_action_server,
        toolexchange_action_server,
        moveto_action_server,
        endeffector_action_server,
        orchestrator,
    ]
    return nodes_to_launch

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
        default_value='false',
        description='Whether to launch MoveIt move_group (false for delegation - orchestrator manages MoveIt)'
    )
    
    return LaunchDescription([
        robot_ip_arg,
        gripper_arg,
        launch_moveit_arg,
        OpaqueFunction(function=launch_setup),
    ])
