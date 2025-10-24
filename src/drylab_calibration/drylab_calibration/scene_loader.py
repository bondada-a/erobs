#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import os
import time

class SceneLoaderNode(Node):
    def __init__(self):
        super().__init__('scene_loader_node')
        self.declare_parameter("scene_file", "")
        scene_path = self.get_parameter("scene_file").get_parameter_value().string_value

        self.publisher = self.create_publisher(CollisionObject, "/collision_object", 10)

        # Wait a moment to allow RViz and MoveIt to be fully ready
        time.sleep(2.0)
        self.load_scene(scene_path)

    def load_scene(self, file_path):
        if not os.path.exists(file_path):
            self.get_logger().error(f"Scene file not found: {file_path}")
            return

        with open(file_path, "r") as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]

        idx = 0
        while idx < len(lines):
            if lines[idx].startswith("*"):
                try:
                    name = lines[idx][2:].strip()
                    pos = list(map(float, lines[idx+1].split()))
                    quat = list(map(float, lines[idx+2].split()))
                    shape = lines[idx+4].strip()
                    dims = list(map(float, lines[idx+5].split()))

                    if shape != "box":
                        self.get_logger().warn(f"Skipping unsupported shape: {shape}")
                        idx += 1
                        continue

                    from geometry_msgs.msg import Pose
                    from shape_msgs.msg import SolidPrimitive
                    from moveit_msgs.msg import CollisionObject
                    from std_msgs.msg import Header

                    co = CollisionObject()
                    co.id = name
                    co.header = Header()
                    co.header.frame_id = "map"
                    co.operation = CollisionObject.ADD

                    solid = SolidPrimitive()
                    solid.type = SolidPrimitive.BOX
                    solid.dimensions = dims

                    pose = Pose()
                    pose.position.x = pos[0]
                    pose.position.y = pos[1]
                    pose.position.z = pos[2]
                    pose.orientation.x = quat[0]
                    pose.orientation.y = quat[1]
                    pose.orientation.z = quat[2]
                    pose.orientation.w = quat[3]

                    co.primitives.append(solid)
                    co.primitive_poses.append(pose)

                    self.publisher.publish(co)
                    self.get_logger().info(f"Published box object: {name}")
                except Exception as e:
                    self.get_logger().error(f"Failed to parse object at line {idx}: {e}")

            idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = SceneLoaderNode()
    rclpy.spin_once(node, timeout_sec=2.0)  # Short spin to allow final messages
    node.destroy_node()
    rclpy.shutdown()
