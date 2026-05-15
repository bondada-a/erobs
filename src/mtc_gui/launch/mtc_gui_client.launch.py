#!/usr/bin/env python3
"""Launch the MTC GUI client.

The GUI reads its beamline configuration (including robot IP) from
$BEAMBOT_BEAMLINE_CONFIG. There are no launch arguments — set the env
var before launching, e.g.:

    export BEAMBOT_BEAMLINE_CONFIG=$(realpath src/beambot/config/cms_beamline.yaml)
    ros2 launch mtc_gui mtc_gui_client.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mtc_gui',
            executable='mtc_gui_client',
            name='mtc_gui_client',
            output='screen',
        ),
    ])
