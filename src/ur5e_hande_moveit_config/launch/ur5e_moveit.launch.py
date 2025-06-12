import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='Type of Universal Robot'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'description_package',
            default_value='ur5e_hande_robot_description',
            description='Robot description package'
        ),
        DeclareLaunchArgument(
            'launch_servo',
            default_value='false',
            description='Launch MoveIt Servo'
        ),
        DeclareLaunchArgument(
            'description_file',
            default_value='ur_with_hande.xacro',
            description='Robot description xacro file'
        ),
        DeclareLaunchArgument(
            'moveit_config_package',
            default_value='ur5e_hande_moveit_config',
            description='MoveIt config package'
        ),
        DeclareLaunchArgument(
            'moveit_config_file',
            default_value='ur.srdf',
            description='MoveIt SRDF file'
        ),
    ]

    # Paths
    ur_moveit_config_share = get_package_share_directory('ur_moveit_config')
    ur_moveit_launch = os.path.join(ur_moveit_config_share, 'launch', 'ur_moveit.launch.py')

    # Include the original launch file with remapped arguments
    ur_moveit_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_launch),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'description_package': LaunchConfiguration('description_package'),
            'launch_servo': LaunchConfiguration('launch_servo'),
            'description_file': LaunchConfiguration('description_file'),
            'moveit_config_package': LaunchConfiguration('moveit_config_package'),
            'moveit_config_file': LaunchConfiguration('moveit_config_file'),
        }.items()
    )

    return LaunchDescription(declared_arguments + [
        ur_moveit_launch_include
    ])