""" Static transform publisher for ZED camera - Manual calibration """
""" EYE-TO-HAND: base_link -> zed_camera_center """
""" Position: X=-65in, Y=+12in, Z=+23in (flat orientation) """
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
                "1.651",  # +65 inches in meters (flipped due to 180° Z rotation)
                "--y",
                "-0.3048",  # -12 inches in meters (flipped due to 180° Z rotation)
                "--z",
                "0.5842",  # +23 inches in meters
                "--qx",
                "0",  # 180-degree rotation about Z axis (flips X and Y)
                "--qy",
                "0",
                "--qz",
                "1",
                "--qw",
                "0",
            ],
        ),
    ]
    return LaunchDescription(nodes)
