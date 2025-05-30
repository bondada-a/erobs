from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the node obstacle_builder with a parameter file."""
    action_cmd = Node(
        package="cms_beamtime",
        executable="pdf_beamtime_server.py",
        parameters=[
            PathJoinSubstitution([FindPackageShare("cms_beamtime"), "config", "obstacles.yaml"]),
            PathJoinSubstitution([FindPackageShare("cms_beamtime"), "config", "joint_poses.yaml"]),
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(action_cmd)

    return ld
