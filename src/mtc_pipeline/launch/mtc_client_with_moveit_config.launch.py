#!/usr/bin/env python3

from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


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
    
    ur_type_arg = DeclareLaunchArgument(
        'ur_type', 
        default_value='ur5e',
        description='UR robot type'
    )
    
    # XACRO arguments for robot description
    xacro_args = {
        "name": "ur", 
        "ur_type": LaunchConfiguration("ur_type"), 
        "tf_prefix": "" 
    }
    
    # Load MoveIt! configuration using the same pattern as the hande config
    moveit_config = (
        MoveItConfigsBuilder("ur_moveit", package_name="ur_hande_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("ur5e_hande_robot_description"), 
                "urdf", 
                "ur_with_hande.xacro"
            ),
            mappings=xacro_args
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_semantic(file_path="config/ur.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    # Client node with proper MoveIt configuration inheritance
    client_node = Node(
        package='mtc_pipeline',
        executable='mtc_action_client_example',
        name='mtc_action_client',
        output='screen',
        parameters=[
            # Inherit all MoveIt configuration parameters
            moveit_config.to_dict(),
            {'use_sim_time': False},
        ],
        arguments=[
            LaunchConfiguration('task_file'),
            LaunchConfiguration('robot_ip'),
            LaunchConfiguration('gripper_type')
        ]
    )
    
    # Add logging
    log_info = LogInfo(
        msg=['Launching MTC Action Client with MoveIt config and task file: ', LaunchConfiguration('task_file')]
    )
    
    return LaunchDescription([
        task_file_arg,
        robot_ip_arg,
        gripper_type_arg,
        ur_type_arg,
        log_info,
        client_node
    ])
