""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: base_link -> zed_camera_center """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "zed_camera_center",
                "--x",
                "-2.1",
                "--y",
                "0.7068",
                "--z",
                "1.2553",
                "--qx",
                "0.0302",
                "--qy",
                "0.2029",
                "--qz",
                "-0.1052",
                "--qw",
                "0.9731",
                # "--roll",
                # "-0",
                # "--pitch",
                # "0",
                # "--yaw",
                # "-0",
            ],
        ),
    ]
    return LaunchDescription(nodes)
