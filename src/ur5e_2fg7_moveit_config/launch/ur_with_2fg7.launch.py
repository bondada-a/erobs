import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("ur_type", default_value="ur5e"))
    declared_arguments.append(DeclareLaunchArgument("description_package", default_value="ur5e_2fg7_robot_description"))
    declared_arguments.append(DeclareLaunchArgument("description_file", default_value="ur_with_2fg7.xacro"))
    declared_arguments.append(DeclareLaunchArgument("moveit_config_package", default_value="ur5e_2fg7_moveit_config"))
    declared_arguments.append(DeclareLaunchArgument("moveit_config_file", default_value="srdf/ur.srdf"))
    declared_arguments.append(DeclareLaunchArgument("moveit_controllers_config", default_value="config/moveit_controllers.yaml"))
    # Launch configurations
    launch_rviz = LaunchConfiguration("launch_rviz")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")

    # Robot description (URDF via xacro command)
    robot_description = {
    "robot_description": launch_ros.descriptions.ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare(description_package),
                "urdf",
                description_file,
            ]),
            " ",
            "name:=ur ",
            "ur_type:=ur5e"
        ]),
        value_type=str
    )
}


    # Robot description semantic (SRDF)
    robot_description_semantic = {
        "robot_description_semantic": launch_ros.descriptions.ParameterValue(
            Command([
                "cat ",
                PathJoinSubstitution([
                    FindPackageShare(moveit_config_package),
                    moveit_config_file,
                ])
            ]),
            value_type=str
        )
    }

    moveit_controllers = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare(moveit_config_package),
            LaunchConfiguration("moveit_controllers_config"),
        ]),
        allow_substs=True
    )

    

    # Move Group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_controllers,    
            {"use_sim_time": False},
        ],
    )

    # RViz node
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {"use_sim_time": False},
        ],
    )

    ld = LaunchDescription(declared_arguments)

    ld.add_action(move_group_node)
    ld.add_action(rviz_node)

    return ld
