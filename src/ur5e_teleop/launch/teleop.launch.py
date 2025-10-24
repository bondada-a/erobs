from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Launch setup function to handle runtime configurations"""

    # Launch arguments
    config_file = LaunchConfiguration('config_file')
    joy_dev = LaunchConfiguration('joy_dev')
    twist_topic = LaunchConfiguration('twist_topic')
    base_frame = LaunchConfiguration('base_frame')

    # Joy node for 8BitDo controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': joy_dev,
            'deadzone': 0.05,
            'autorepeat_rate': 50.0,
        }],
        output='screen'
    )

    # Cartesian teleop node
    teleop_node = Node(
        package='ur5e_teleop',
        executable='cartesian_teleop_node.py',
        name='cartesian_teleop_node',
        parameters=[{
            'config_file': config_file,
            'twist_topic': twist_topic,
            'joy_topic': '/joy',
            'gripper_action': '/gripper_action_controller/gripper_cmd',
            'base_frame': base_frame,
        }],
        output='screen'
    )

    return [joy_node, teleop_node]


def generate_launch_description():
    """Generate launch description for UR5e teleoperation"""

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='8bitdo_ultimate_2c.yaml',
        description='Controller configuration file name'
    )

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    twist_topic_arg = DeclareLaunchArgument(
        'twist_topic',
        default_value='/servo_node/delta_twist_cmds',
        description='Topic to publish twist commands to'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame for twist commands'
    )

    return LaunchDescription([
        config_file_arg,
        joy_dev_arg,
        twist_topic_arg,
        base_frame_arg,
        OpaqueFunction(function=launch_setup)
    ])
