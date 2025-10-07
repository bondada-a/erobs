"""
Launch file to load and publish shared planning scene
Include this in all MoveIt config launch files
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments
    scene_config_arg = DeclareLaunchArgument(
        'scene_config',
        default_value='beamline_scene.yaml',
        description='YAML file defining planning scene obstacles'
    )

    # Scene publisher node
    scene_publisher = Node(
        package='erobs_planning_scene',
        executable='scene_publisher.py',
        name='shared_planning_scene_publisher',
        output='screen',
        parameters=[{
            'scene_config': LaunchConfiguration('scene_config')
        }]
    )

    return LaunchDescription([
        scene_config_arg,
        scene_publisher
    ])
