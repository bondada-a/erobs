#!/usr/bin/env python3
"""
Test launch file for Zivid hand-eye calibration using static transform publisher.

This file publishes the calibrated transform as a static TF for testing purposes
before applying changes to the URDF.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with static transform publisher."""

    # Calibration values from hand_eye_transform.yaml
    # Converted from millimeters to meters and rotation matrix to quaternion

    return LaunchDescription([
        # Static transform publisher for calibrated flange -> optical frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="zivid_calibration_static_tf",
            output="screen",
            arguments=[
                "--x", "-0.05435",      # Translation X in meters
                "--y", "-0.10490",      # Translation Y in meters
                "--z", "-0.19139",      # Translation Z in meters
                "--qx", "-0.01493",     # Quaternion X
                "--qy", "0.02668",      # Quaternion Y
                "--qz", "-0.00867",     # Quaternion Z
                "--qw", "0.99949",      # Quaternion W
                "--frame-id", "flange",
                "--child-frame-id", "zivid_optical_frame_test",
            ],
        ),

        # Optional: Echo TF for debugging
        Node(
            package="tf2_ros",
            executable="tf2_echo",
            name="tf_echo_calibration",
            output="screen",
            arguments=["flange", "zivid_optical_frame_test"],
            on_exit="shutdown",
        ),
    ])


if __name__ == "__main__":
    generate_launch_description()