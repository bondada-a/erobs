#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import os
import math

import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower_moveit')
        moveit_commander.roscpp_initialize([])

        self.robot = moveit_commander.RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "ur_manipulator"  # Update if your group name differs
        self.move_group = MoveGroupCommander(self.group_name)

        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.2)

        self.waypoints = self.load_waypoints()

    def load_waypoints(self):
        filename = os.path.join(os.getcwd(), "waypoints.json")
        try:
            with open(filename, "r") as f:
                waypoints = json.load(f)
                self.get_logger().info(f"Loaded waypoints: {list(waypoints.keys())}")
                return waypoints
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return {}

    def move_through_waypoints(self):
        sorted_labels = sorted(self.waypoints.keys(), key=lambda x: int(x.split('_')[-1]))

        for label in sorted_labels:
            pose_deg = self.waypoints[label]
            pose_rad = [math.radians(j) for j in pose_deg]

            self.get_logger().info(f"Moving to {label}: {pose_deg}")
            success = self.move_group.go(pose_rad, wait=True)
            self.move_group.stop()

            if success:
                self.get_logger().info(f"Reached {label}")
            else:
                self.get_logger().warn(f"Failed to reach {label}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()

    if node.waypoints:
        node.move_through_waypoints()
    else:
        node.get_logger().error("No waypoints loaded. Exiting.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
