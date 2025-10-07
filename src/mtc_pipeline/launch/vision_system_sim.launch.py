#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Vision action server
    vision_action_server = Node(
        package='mtc_pipeline',
        executable='vision_action_server',
        name='vision_action_server',
        output='screen'
    )

    # Mock AprilTag detector (publishes fake tags at fixed positions)
    mock_detector = Node(
        package='mtc_pipeline',
        executable='mock_apriltag_detector',
        name='mock_apriltag_detector',
        output='screen',
        parameters=[{
            'tag_ids': [0, 1, 2],
            'publish_moving_tags': False
        }]
    )

    return LaunchDescription([
        vision_action_server,
        mock_detector
    ])
