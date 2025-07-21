from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

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

    mtc_orchestrator_node = Node(
        package="mtc_pipeline",
        executable="mtc_orchestrator",   # <--- change to your new executable!
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'poses_file': LaunchConfiguration('poses_file')}
        ],
    )

    return LaunchDescription([poses_file_arg, mtc_orchestrator_node])
