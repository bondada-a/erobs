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
                "-0.944134",
                "--y",
                "0.61108",
                "--z",
                "0.755722",
                "--qx",
                "0.0619314",
                "--qy",
                "0.207791",
                "--qz",
                "-0.221667",
                "--qw",
                "0.950711",
                # "--roll",
                # "0.227645",
                # "--pitch",
                # "0.376473",
                # "--yaw",
                # "-0.501681",
            ],
        ),
    ]
    return LaunchDescription(nodes)
