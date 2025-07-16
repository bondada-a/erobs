from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur5e_hande")
        .robot_description(file_path="config/ur.urdf.xacro")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    poses_file_arg = DeclareLaunchArgument(
        'poses_file',
        default_value='/home/user/poses.json',
        description='Full path to the JSON file with poses'
    )
    operation_arg = DeclareLaunchArgument(
        'operation',
        default_value='load',
        description='Operation: "load" (attach EE) or "dock" (detach EE)'
    )

    dock_number_arg = DeclareLaunchArgument(
    "dock_number",
    default_value="3",                     # centre dock
    description="Dock index (1–5)"
    )


    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_pipeline",
        executable="mtc_toolexchange",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'poses_file': LaunchConfiguration('poses_file')},
            {'operation': LaunchConfiguration('operation')},
            {"dock_number": LaunchConfiguration("dock_number")}, 
        ],
    )

    return LaunchDescription([poses_file_arg, operation_arg, pick_place_demo, dock_number_arg])  # Add dock_number_arg to the launch description