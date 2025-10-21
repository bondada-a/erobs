from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch UR5e joint velocity teleoperation with 8BitDo controller"""

    # Declare arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='Joystick device ID (0 for js0, 1 for js1, etc.)'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='8bitdo_ultimate_2c.yaml',
        description='Controller configuration file'
    )

    max_joint_velocity_arg = DeclareLaunchArgument(
        'max_joint_velocity',
        default_value='0.5',
        description='Maximum joint velocity in rad/s'
    )

    # Joy node for 8BitDo controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 50.0,
        }],
        output='screen'
    )

    # Joint teleop node
    teleop_node = Node(
        package='ur5e_teleop',
        executable='joint_teleop_node.py',
        name='joint_teleop_node',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'velocity_topic': '/forward_velocity_controller/commands',
            'joy_topic': '/joy',
            'gripper_action': '/gripper_action_controller/gripper_cmd',
            'max_joint_velocity': LaunchConfiguration('max_joint_velocity'),
        }],
        output='screen'
    )

    return LaunchDescription([
        joy_dev_arg,
        config_file_arg,
        max_joint_velocity_arg,
        joy_node,
        teleop_node
    ])
