from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch MoveIt Servo for UR5e teleoperation"""

    # Get path to our config file
    servo_yaml = os.path.join(
        get_package_share_directory('ur5e_teleop'),
        'config',
        'servo',
        'ur5e_servo.yaml'
    )

    # Servo node with params-file argument
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        emulate_tty=True,
        parameters=[servo_yaml]
    )

    return LaunchDescription([
        servo_node
    ])
