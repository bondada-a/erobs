"""Launch file for Octomap with Zivid camera + MoveIt Planning Scene integration.

Includes:
1. Downsampling relay to reduce Zivid's ~5M points to ~10k points
2. Octomap server to build 3D occupancy grid
3. Bridge node to push octomap into MoveIt's planning scene

Usage:
    Terminal 1: ros2 launch beambot beambot_bringup.launch.py
    Terminal 2: ros2 launch beambot octomap_test.launch.py
    Terminal 3: Trigger a capture (via GUI or service call)

View in RViz:
    - Add MarkerArray display → topic: /occupied_cells_vis_array
    - Add PointCloud2 display → topic: /octomap_point_cloud_centers
    - Planning Scene display should show octomap as collision geometry

Data flow:
    Zivid (/points/xyz, ~5M points)
        → pointcloud_relay (voxel downsample)
            → /points/xyz_relayed (~10k points)
                → octomap_server
                    → /octomap_binary
                        → octomap_to_planning_scene
                            → /planning_scene (MoveIt)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch argument: path to saved octomap (empty = start fresh)
    declare_octomap_path = DeclareLaunchArgument(
        'octomap_path',
        default_value='',
        description='Path to saved .bt octomap file. Empty = start with empty map.'
    )

    # Voxel downsampling relay - reduces ~5M points to ~10k for octomap
    pointcloud_relay = Node(
        package='beambot',
        executable='pointcloud_relay.py',
        name='pointcloud_relay',
        parameters=[{
            'input_topic': '/points/xyz',
            'output_topic': '/points/xyz_relayed',
            'enable_downsampling': True,
            'voxel_size': 0.01,  # 1cm voxels (matches octomap resolution)
        }],
        output='screen',
    )

    # Octomap server node
    # Subscribes to relayed point cloud, builds and publishes octomap
    #
    # The point cloud comes in zivid_optical_frame, octomap_server
    # will transform it to frame_id (base_link) using TF.
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
            'resolution': 0.01,  # 1cm voxel resolution
            'frame_id': 'base_link',  # Octomap is built in robot base frame
            'sensor_model.max_range': 2.0,  # Max sensor range in meters
            'sensor_model.hit': 0.7,  # Probability update on hit (occupied)
            'sensor_model.miss': 0.4,  # Probability update on miss (free)
            'filter_ground': False,  # Keep ground/table surface
            'base_frame_id': 'base_link',
            'latch': False,  # Don't latch (we want fresh data)
            'transform_tolerance': 5.0,  # TF timing slack for Zivid capture
            'octomap_path': LaunchConfiguration('octomap_path'),
            # Visualization settings
            'use_height_map': True,  # Color by height (red=low, blue=high)
            'publish_free_space': False,  # Don't show free space (green) voxels
        }],
        remappings=[
            # Subscribe to relayed topic (BEST_EFFORT compatible)
            ('cloud_in', '/points/xyz_relayed'),
        ],
        output='screen',
    )

    # Bridge node: octomap → MoveIt planning scene
    # Subscribes to /octomap_binary and publishes to /planning_scene
    octomap_to_planning_scene = Node(
        package='beambot',
        executable='octomap_to_planning_scene.py',
        name='octomap_to_planning_scene',
        parameters=[{
            'octomap_topic': '/octomap_binary',
            'planning_scene_topic': '/planning_scene',
            'min_update_interval': 0.5,  # Throttle to avoid overwhelming MoveIt
            'log_updates': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_octomap_path,
        pointcloud_relay,
        octomap_server,
        octomap_to_planning_scene,
    ])
