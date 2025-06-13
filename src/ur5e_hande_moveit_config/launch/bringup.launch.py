from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur", package_name="ur5e_hande_moveit_config").to_moveit_configs()
    move_group_ld = generate_move_group_launch(moveit_config)
    rviz_ld = generate_moveit_rviz_launch(moveit_config)
    return LaunchDescription(move_group_ld.entities + rviz_ld.entities)