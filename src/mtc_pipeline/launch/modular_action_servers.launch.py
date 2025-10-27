#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.56.101',
        description='Robot IP address'
    )

    # Action servers need kinematics for Cartesian planning
    # Use arm-only kinematics (works with all grippers)
    # Robot URDF comes from /robot_description topic published by move_group
    action_server_parameters = [
        {'use_sim_time': False},
        {
            'robot_description_kinematics': {
                'ur_arm': {
                    'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                    'kinematics_solver_search_resolution': 0.001,
                    'kinematics_solver_timeout': 1.0,
                    'kinematics_solver_attempts': 10
                }
            }
        }
    ]

    # Modular Action Servers - connect to MoveIt managed by orchestrator
    pick_place_action_server = Node(
        package='mtc_pipeline',
        executable='pick_place_action_server',
        name='pick_place_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    tool_exchange_action_server = Node(
        package='mtc_pipeline',
        executable='tool_exchange_action_server',
        name='tool_exchange_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    move_to_action_server = Node(
        package='mtc_pipeline',
        executable='move_to_action_server',
        name='move_to_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    end_effector_action_server = Node(
        package='mtc_pipeline',
        executable='end_effector_action_server',
        name='end_effector_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    vision_action_server = Node(
        package='mtc_pipeline',
        executable='vision_action_server',
        name='vision_action_server',
        output='screen',
        parameters=action_server_parameters + [
            {'publish_marker_frames': True},  # Enable TF publishing for RViz visualization
            {'ik_frame': 'epick_tip'},  # Auto-detect: '' | Force EPick: 'epick_tip' | Force Hand-E: 'robotiq_hande_end'
            {'z_offset': 0.025}  # Positive = higher above marker (10cm above)
        ]
    )

    pipettor_action_server = Node(
        package='mtc_pipeline',
        executable='pipettor_action_server',
        name='pipettor_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    vision_pick_place_action_server = Node(
        package='mtc_pipeline',
        executable='vision_pick_place_action_server',
        name='vision_pick_place_action_server',
        output='screen',
        parameters=action_server_parameters
    )

    # Zivid camera node with 2D and 3D capture settings
    zivid_camera = Node(
        package='zivid_camera',
        executable='zivid_camera',
        name='zivid_camera',
        output='screen',
        parameters=[{
            'settings_2d_file_path': '/home/aditya/work/github_ws/erobs/src/zivid_settings.yml',
            'settings_file_path': '/home/aditya/work/github_ws/erobs/src/zivid_3d_settings.yml',  # For 3D marker detection
            'frame_id': 'zivid_optical_frame'
        }]
    )

    # AprilTag detector REMOVED - now using Zivid built-in ArUco detection
    # Detection happens via /capture_and_detect_markers service (no separate node needed)

    # Pipettor driver - launched by ur_zivid_pipettor_moveit_config (not here)
    # This ensures /tmp/ttyUR exists before pipette_driver_node starts

    # Main Orchestrator - manages MoveIt lifecycle
    orchestrator = Node(
        package='mtc_pipeline',
        executable='mtc_orchestrator_action_server',
        name='mtc_orchestrator_action_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'robot_ip': LaunchConfiguration('robot_ip')},
        ]
    )

    return LaunchDescription([
        robot_ip_arg,
        pick_place_action_server,
        tool_exchange_action_server,
        move_to_action_server,
        end_effector_action_server,
        vision_action_server,
        pipettor_action_server,
        vision_pick_place_action_server,
        zivid_camera,
        # apriltag_detector removed - using Zivid built-in ArUco detection
        # pipettor_driver launched by MoveIt config, not here
        orchestrator,
    ])
