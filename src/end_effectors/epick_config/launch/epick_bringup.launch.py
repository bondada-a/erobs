from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    parent_arg = DeclareLaunchArgument("parent", default_value="tool0")
    port_arg = DeclareLaunchArgument("usb_port", default_value="/tmp/ttyUR")
    fake_arg = DeclareLaunchArgument("use_fake_hardware", default_value="false")

    usb_port = LaunchConfiguration("usb_port")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    controller = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "usb_port": usb_port,
                "use_fake_hardware": use_fake_hardware,
            }
        ],
    )

    return LaunchDescription([
        parent_arg,
        port_arg,
        fake_arg,
        controller,
    ])
