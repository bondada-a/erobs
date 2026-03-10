""" Static transform publisher acquired via ChArUco hand-eye calibration """
""" EYE-TO-HAND: base_link -> zed_left_camera_optical_frame """
""" Calibrated: 2026-03-09, 11 samples, Park solver """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        # Calibration result: base_link -> zed_left_camera_optical_frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "zed_left_camera_optical_frame",
                "--x",
                "-0.967543",
                "--y",
                "-0.735554",
                "--z",
                "0.044037",
                "--qx",
                "-0.632920",
                "--qy",
                "0.294752",
                "--qz",
                "-0.219842",
                "--qw",
                "0.681324",
            ],
        ),
        # Bridge: optical_frame -> camera_frame (inverse of standard optical rotation)
        # Needed because ZED publishes point cloud in zed_left_camera_frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "zed_left_camera_optical_frame",
                "--child-frame-id",
                "zed_left_camera_frame",
                "--qx",
                "0.5",
                "--qy",
                "-0.5",
                "--qz",
                "0.5",
                "--qw",
                "0.5",
            ],
        ),
    ]
    return LaunchDescription(nodes)
