""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: base_link -> zed_left_camera_frame """
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
                "zed_left_camera_frame",
                "--x",
                "-0.952485",
                "--y",
                "0.707833",
                "--z",
                "0.724495",
                "--qx",
                "0.0691074",
                "--qy",
                "0.165335",
                "--qz",
                "-0.280394",
                "--qw",
                "0.94301",
                # "--roll",
                # "0.233998",
                # "--pitch",
                # "0.276583",
                # "--yaw",
                # "-0.610743",
            ],
        ),
    ]
    return LaunchDescription(nodes)
