""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: base_link -> camera_link """
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
                "camera_link",
                "--x",
                "0.800506",
                "--y",
                "0.430185",
                "--z",
                "0.256334",
                "--qx",
                "-0.0138355",
                "--qy",
                "-0.00573375",
                "--qz",
                "0.97664",
                "--qw",
                "-0.214359",
                # "--roll",
                # "0.0171371",
                # "--pitch",
                # "-0.0245689",
                # "--yaw",
                # "-2.70926",
            ],
        ),
    ]
    return LaunchDescription(nodes)
