#!/usr/bin/env python3
"""Bridge node: Octomap to MoveIt Planning Scene.

Subscribes to /octomap_binary from octomap_server and publishes to
/planning_scene for MoveIt collision avoidance.

This bridge is needed because MoveIt's native PointCloudOctomapUpdater
uses a tf2 MessageFilter with hardcoded timing that doesn't work well
with single-shot cameras like Zivid. The standalone octomap_server
has a configurable transform_tolerance (5.0s) that handles this.

Data flow:
    octomap_server (/octomap_binary)
        → this bridge node
            → MoveIt move_group (/planning_scene)

Usage:
    ros2 run beambot octomap_to_planning_scene.py

    Or via launch file (octomap_test.launch.py)
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import Octomap, OctomapWithPose


class OctomapToPlanningScene(Node):
    """Bridges octomap_server output to MoveIt's planning scene."""

    def __init__(self):
        super().__init__('octomap_to_planning_scene')

        # Parameters
        self.declare_parameter('octomap_topic', '/octomap_binary')
        self.declare_parameter('planning_scene_topic', '/planning_scene')
        self.declare_parameter('min_update_interval', 0.5)  # seconds
        self.declare_parameter('log_updates', True)

        octomap_topic = self.get_parameter('octomap_topic').value
        planning_scene_topic = self.get_parameter('planning_scene_topic').value
        self._min_interval = self.get_parameter('min_update_interval').value
        self._log_updates = self.get_parameter('log_updates').value

        # Throttling state
        self._last_update_time = 0.0
        self._updates_sent = 0
        self._updates_throttled = 0

        # QoS: Match MoveGroup's subscription (RELIABLE + VOLATILE)
        scene_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscriber to octomap_server output
        # octomap_server publishes with RELIABLE QoS
        self._octomap_sub = self.create_subscription(
            Octomap,
            octomap_topic,
            self._octomap_callback,
            scene_qos
        )

        # Publisher to MoveIt planning scene
        self._scene_pub = self.create_publisher(
            PlanningScene,
            planning_scene_topic,
            scene_qos
        )

        self.get_logger().info(
            f'Octomap→PlanningScene bridge started: '
            f'{octomap_topic} → {planning_scene_topic} '
            f'(throttle: {self._min_interval}s)'
        )

    def _octomap_callback(self, octomap_msg: Octomap):
        """Process incoming octomap and forward to planning scene."""
        now = time.time()

        # Throttle updates to avoid overwhelming MoveIt
        if now - self._last_update_time < self._min_interval:
            self._updates_throttled += 1
            return

        # Build PlanningScene message
        scene_msg = PlanningScene()
        scene_msg.is_diff = True  # Differential update

        # Wrap octomap in OctomapWithPose
        octomap_with_pose = OctomapWithPose()
        octomap_with_pose.header = octomap_msg.header
        octomap_with_pose.octomap = octomap_msg

        # Origin is identity (octomap is already in the correct frame)
        octomap_with_pose.origin = Pose()
        octomap_with_pose.origin.orientation.w = 1.0

        # Assign to planning scene world
        scene_msg.world.octomap = octomap_with_pose

        # Publish
        self._scene_pub.publish(scene_msg)
        self._last_update_time = now
        self._updates_sent += 1

        if self._log_updates:
            # Calculate octomap size for logging
            data_size_kb = len(octomap_msg.data) / 1024
            self.get_logger().info(
                f'Published octomap to planning scene '
                f'(frame: {octomap_msg.header.frame_id}, '
                f'res: {octomap_msg.resolution:.3f}m, '
                f'size: {data_size_kb:.1f}KB, '
                f'updates: {self._updates_sent}, '
                f'throttled: {self._updates_throttled})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OctomapToPlanningScene()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutting down. Total updates: {node._updates_sent}, '
            f'throttled: {node._updates_throttled}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
