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
                "-0.000762647",
                "--y",
                "0.696573",
                "--z",
                "0.779518",
                "--qx",
                "0.140081",
                "--qy",
                "0.149452",
                "--qz",
                "-0.82638",
                "--qw",
                "0.524535",
                # "--roll",
                # "0.406141",
                # "--pitch",
                # "-0.0748046",
                # "--yaw",
                # "-1.99504",
            ],
        ),
    ]
    return LaunchDescription(nodes)
