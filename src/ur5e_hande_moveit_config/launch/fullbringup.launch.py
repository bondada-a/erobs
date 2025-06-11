#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument('UR_TYPE', default_value='ur5e'),
        DeclareLaunchArgument('ROBOT_IP', default_value='192.168.56.101'),  # Change default as needed
        DeclareLaunchArgument('DESCRIPTION_PKG', default_value='ur5e_hande_robot_description'),
        DeclareLaunchArgument('DESCRIPTION_FILE', default_value='ur_with_hande.xacro'),
        DeclareLaunchArgument('LAUNCH_RVIZ', default_value='true'),
        DeclareLaunchArgument('MOVEIT_CONFIG_PKG', default_value='ur5e_hande_moveit_config'),
        DeclareLaunchArgument('MOVEIT_CONFIG_FILE', default_value='ur.srdf'),
        DeclareLaunchArgument('LAUNCH_SERVO', default_value='false'),
    ]

    # Launch ur_control
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur_robot_driver'),
            '/launch/ur_control.launch.py'
        ]),
        launch_arguments={
            'ur_type':         LaunchConfiguration('UR_TYPE'),
            'robot_ip':        LaunchConfiguration('ROBOT_IP'),
            'description_package': LaunchConfiguration('DESCRIPTION_PKG'),
            'description_file': LaunchConfiguration('DESCRIPTION_FILE'),
            'launch_rviz':     LaunchConfiguration('LAUNCH_RVIZ'),
            'tool_voltage':    '24'
        }.items()
    )

    # Tool communication node
    tool_comm_node = Node(
        package='ur_robot_driver',
        executable='tool_communication.py',
        name='tool_communication',
        output='screen',
        parameters=[{'robot_ip': LaunchConfiguration('ROBOT_IP')}]
    )

    # Gripper service node
    gripper_service_node = Node(
        package='gripper_service',
        executable='gripper_service',
        name='gripper_service',
        output='screen'
    )

    # Gripper action bridge node
    gripper_action_bridge_node = Node(
        package='gripper_service',
        executable='gripper_action_bridge.py',
        name='gripper_action_bridge',
        output='screen'
    )

    # MoveIt launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur_moveit_config'),
            '/launch/ur_moveit.launch.py'
        ]),
        launch_arguments={
            'ur_type': LaunchConfiguration('UR_TYPE'),
            'robot_ip': LaunchConfiguration('ROBOT_IP'),
            'launch_rviz': LaunchConfiguration('LAUNCH_RVIZ'),
            'description_package': LaunchConfiguration('DESCRIPTION_PKG'),
            'description_file': LaunchConfiguration('DESCRIPTION_FILE'),
            'moveit_config_package': LaunchConfiguration('MOVEIT_CONFIG_PKG'),
            'moveit_config_file': LaunchConfiguration('MOVEIT_CONFIG_FILE'),
            'launch_servo': LaunchConfiguration('LAUNCH_SERVO'),
        }.items()
    )

    return LaunchDescription(
        declared_arguments +
        [
            ur_control_launch,
            tool_comm_node,
            gripper_service_node,
            gripper_action_bridge_node,
            moveit_launch,
        ]
    )