from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ## Arguments  
    ur_type = DeclareLaunchArgument('ur_type', default_value='ur5e')
    robot_ip = DeclareLaunchArgument('robot_ip', default_value='192.168.56.101')
    use_fake_hardware = DeclareLaunchArgument('use_fake_hardware', default_value='false')
    description_package = DeclareLaunchArgument('description_package', default_value='ur5e_hande_robot_description')
    description_file = DeclareLaunchArgument('description_file', default_value='ur_with_zivid_hande.xacro')
    controllers_file = DeclareLaunchArgument('controllers_file', default_value=os.path.join(get_package_share_directory("ur_zivid_hande_moveit_config"), "config", "ur_hande_controllers.yaml"))

    xacro_args = {"name": "ur", "ur_type": LaunchConfiguration("ur_type"), "tf_prefix": "" }

    ## ur_driver 
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ]),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "launch_rviz": "false",
            "description_package": LaunchConfiguration("description_package"),
            "description_file": LaunchConfiguration("description_file"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "tool_voltage": "24",
        }.items()
    )

    # Load MoveIt! configuration (same as robot_bringup.launch.py)
    moveit_config = (
        MoveItConfigsBuilder("ur_moveit", package_name="ur_zivid_hande_moveit_config")
        .robot_description(file_path=os.path.join(get_package_share_directory("ur5e_hande_robot_description"), "urdf", "ur_with_zivid_hande.xacro"), mappings=xacro_args)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])  # This is the key configuration!
        .to_moveit_configs()
    )

    # Load ExecuteTaskSolutionCapability so we can execute found solutions
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # HandE controller spawner
    hande_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "-c", "/controller_manager"],
    )

    # MTC Orchestrator Action Server (gets MoveIt config from move_group)
    mtc_orchestrator_action_server = Node(
        package="mtc_pipeline",
        executable="mtc_orchestrator_action_server",
        name="mtc_orchestrator_action_server",
        output="screen",
        parameters=[
            # Don't pass moveit_config here - let it get from move_group
        ]
    )

    # MoveTo Action Server (gets proper planning config)
    moveto_action_server = Node(
        package="mtc_pipeline",
        executable="moveto_action_server",
        name="moveto_action_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),  # Pass the same MoveIt config
            {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization',
                'start_state_max_bounds_error': 0.1
            }
        ]
    )

    return LaunchDescription([
        ## arguments 
        robot_ip,
        ur_type,
        use_fake_hardware,
        description_package,
        description_file,
        controllers_file,

        ## Nodes
        ur_control_launch,
        run_move_group_node,
        static_tf,
        robot_state_publisher,
        hande_controller_spawner,
        mtc_orchestrator_action_server,
        moveto_action_server,
    ])
