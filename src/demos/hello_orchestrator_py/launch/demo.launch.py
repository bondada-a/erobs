"""Launch file for hello_orchestrator_py demo.

Prerequisites:
- MoveIt must be running (provides /robot_description topic)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the demo.

    Launches:
    1. Print action server
    2. Move action server (with MTC/OMPL params)
    3. Orchestrator action server
    """

    # OMPL parameters for MTC PipelinePlanner (only needed by move_server)
    ompl_args = [
        '--ros-args',
        '-p', 'ompl.planning_plugin:=ompl_interface/OMPLPlanner',
        '-p', 'ompl.request_adapters:=default_planner_request_adapters/AddTimeOptimalParameterization',
    ]

    # Kinematics parameters for MTC (only needed by move_server)
    kinematics_params = {
        'robot_description_kinematics': {
            'ur_arm': {
                'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                'kinematics_solver_search_resolution': 0.001,
                'kinematics_solver_timeout': 1.0,
                'kinematics_solver_attempts': 10
            }
        }
    }

    return LaunchDescription([
        # Print action server (no MTC)
        Node(
            package='hello_orchestrator_py',
            executable='print_server.py',
            name='print_server_py',
            output='screen',
        ),

        # Move action server (uses MTC)
        Node(
            package='hello_orchestrator_py',
            executable='move_server.py',
            name='move_server_py',
            output='screen',
            parameters=[kinematics_params],
            arguments=ompl_args,
        ),

        # Orchestrator action server (no MTC - just dispatches)
        Node(
            package='hello_orchestrator_py',
            executable='orchestrator_server.py',
            name='orchestrator_server_py',
            output='screen',
        ),
    ])
