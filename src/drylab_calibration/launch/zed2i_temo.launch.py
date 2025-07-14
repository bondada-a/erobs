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
                "-1.5294",
                "--y",
                "1.3404",
                "--z",
                "0.4894",
                "--qx",
                "0",
                "--qy",
                "0",
                "--qz",
                "-0.4270",
                "--qw",
                "0.9042",
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
