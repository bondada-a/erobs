from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to your YAML file with octomap parameters
    octomap_params_file = PathJoinSubstitution([
        FindPackageShare("pdf_beamtime"),  # Your custom package
        "config",
        "move_group_params.yaml"
    ])

    # Include the original UR MoveIt launch file
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py"
            ])
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "description_package": "ur5e_hande_robot_description",
            "description_file": "ur_with_hande.xacro",
            "moveit_config_package": "ur5e_hande_moveit_config",
            "moveit_config_file": "ur.srdf",
            "launch_rviz": "true",
            "launch_servo": "false",
        }.items()
    )

    return LaunchDescription([
        ur_moveit_launch,
    ])
