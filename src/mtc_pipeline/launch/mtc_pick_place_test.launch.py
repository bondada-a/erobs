from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Configure MoveIt using your URDF/SRDF
    ur_type = DeclareLaunchArgument('ur_type', default_value='ur5e')
    robot_ip = DeclareLaunchArgument('robot_ip', default_value='192.168.1.10')
    use_fake_hardware = DeclareLaunchArgument('use_fake_hardware', default_value='false')
    description_package = DeclareLaunchArgument('description_package', default_value='ur5e_hande_robot_description')
    description_file = DeclareLaunchArgument('description_file', default_value='ur_with_hande.xacro')
    controllers_file = DeclareLaunchArgument('controllers_file', default_value=os.path.join(get_package_share_directory("ur_hande_moveit_config"), "config", "ur_hande_controllers.yaml"))


    xacro_args = {"name": "ur", "ur_type": LaunchConfiguration("ur_type"), "tf_prefix": "" }

    moveit_config = (
        MoveItConfigsBuilder("ur_moveit",package_name="ur_hande_moveit_config")
        .robot_description(file_path=os.path.join(get_package_share_directory("ur5e_hande_robot_description"), "urdf", "ur_with_hande.xacro"),mappings=xacro_args)
        .robot_description_semantic(file_path="config/ur.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Optional arg to override pose config JSON
    poses_file_arg = DeclareLaunchArgument(
        'poses_file',
        default_value='/home/user/poses.json',
        description='Full path to the JSON file with poses'
    )

    # Node that runs the MTC PickPlaceStages
    pick_place_node = Node(
        package="mtc_pipeline",
        executable="mtc_test_runner",  # your compiled test node
        name="mtc_test_runner",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'poses_file': LaunchConfiguration('poses_file')}
        ]
    )

    return LaunchDescription([
        ur_type,
        robot_ip,
        use_fake_hardware,
        description_package,
        description_file,
        controllers_file,
        poses_file_arg,

        pick_place_node
    ])
