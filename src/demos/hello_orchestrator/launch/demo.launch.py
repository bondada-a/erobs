"""Launch file for hello_orchestrator demo"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Print action server
        Node(
            package='hello_orchestrator',
            executable='print_server',
            name='print_server',
            output='screen',
        ),

        # Move action server
        Node(
            package='hello_orchestrator',
            executable='move_server',
            name='move_server',
            output='screen',
        ),

        # Orchestrator action server
        Node(
            package='hello_orchestrator',
            executable='orchestrator_server',
            name='orchestrator_server',
            output='screen',
        ),
    ])
