"""Bring up the EPICS bridge (vcs-imported) + the action-status PV adapter.

The bridge package is read-only; all customization is this package's config,
passed to the bridge via its config_file launch arg.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("erob_epics"), "config", "cms_bridge_config.yaml"]
    )
    declare_config = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="EPICS bridge YAML (PV<->topic map)",
    )
    config_file = LaunchConfiguration("config_file")

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("epics_ros2_bridge"),
                 "launch", "epics_ros2_bridge.launch.py"]
            )
        ),
        launch_arguments={"config_file": config_file}.items(),
    )

    status_adapter = Node(
        package="erob_epics",
        executable="action_status_to_pv.py",
        name="action_status_to_pv",
        output="screen",
    )

    return LaunchDescription([declare_config, bridge, status_adapter])
