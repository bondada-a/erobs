#!/usr/bin/env python3
"""
Shared planning scene publisher for EROBS beamline
Reads obstacles from YAML and publishes to /planning_scene topic
All MoveIt configs automatically receive the scene
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
import yaml
import math
from ament_index_python.packages import get_package_share_directory
import os


class SharedPlanningScenePublisher(Node):
    def __init__(self):
        super().__init__('shared_planning_scene_publisher')

        # Declare parameter for scene config file
        self.declare_parameter('scene_config', 'beamline_scene.yaml')

        # Publisher for planning scene
        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        # Load scene from YAML
        self.obstacles = self.load_scene_config()

        # Wait briefly for subscribers
        self.get_logger().info('Waiting for planning scene subscribers...')
        rclpy.spin_once(self, timeout_sec=1.0)

        # Publish initial scene
        self.publish_scene()

        # Publish periodically to ensure all configs receive it
        self.timer = self.create_timer(5.0, self.publish_scene)

        self.get_logger().info(f'Shared planning scene publisher ready with {len(self.obstacles)} obstacles')

    def load_scene_config(self):
        """Load obstacle definitions from YAML file"""
        scene_config = self.get_parameter('scene_config').value

        # Try to find the config file
        try:
            package_share = get_package_share_directory('erobs_planning_scene')
            config_path = os.path.join(package_share, 'config', scene_config)
        except:
            # Fallback for development
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'config',
                scene_config
            )

        self.get_logger().info(f'Loading scene from: {config_path}')

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        return config.get('obstacles', [])

    def create_collision_object(self, obstacle_config):
        """Create a CollisionObject from YAML config"""
        collision_object = CollisionObject()
        collision_object.id = obstacle_config['name']
        collision_object.header.frame_id = obstacle_config.get('frame', 'map')

        # Create pose
        pose = Pose()
        pose_config = obstacle_config['pose']
        pose.position.x = float(pose_config.get('x', 0.0))
        pose.position.y = float(pose_config.get('y', 0.0))
        pose.position.z = float(pose_config.get('z', 0.0))

        # Convert RPY to quaternion
        roll = float(pose_config.get('roll', 0.0))
        pitch = float(pose_config.get('pitch', 0.0))
        yaw = float(pose_config.get('yaw', 0.0))
        quat = self.euler_to_quaternion(roll, pitch, yaw)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        # Create primitive shape
        primitive = SolidPrimitive()
        obj_type = obstacle_config['type'].lower()

        if obj_type == 'box':
            primitive.type = SolidPrimitive.BOX
            size = obstacle_config['size']
            primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        elif obj_type == 'cylinder':
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [
                float(obstacle_config['height']),
                float(obstacle_config['radius'])
            ]
        elif obj_type == 'sphere':
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [float(obstacle_config['radius'])]
        else:
            self.get_logger().warning(f"Unknown obstacle type: {obj_type}")
            return None

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        return collision_object

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]

    def publish_scene(self):
        """Publish the shared planning scene with all obstacles"""
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True

        # Add all obstacles from config
        for obstacle_config in self.obstacles:
            collision_obj = self.create_collision_object(obstacle_config)
            if collision_obj:
                scene_msg.world.collision_objects.append(collision_obj)

        self.scene_pub.publish(scene_msg)
        self.get_logger().debug(f'Published planning scene with {len(self.obstacles)} obstacles')


def main(args=None):
    rclpy.init(args=args)
    node = SharedPlanningScenePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
