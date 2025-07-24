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

    # MTC pick_place node
    pick_place = Node(
        package="mtc_pipeline",
        executable="mtc_pickplace",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'poses_file': LaunchConfiguration('poses_file')}
        ],
    )

    return LaunchDescription([poses_file_arg, pick_place])