#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import os
import math
import time
import subprocess

from ur_msgs.srv import SetPayload
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pdf_beamtime_interfaces.srv import GripperControlMsg


class WaypointExecutor(Node):
    def __init__(self):
        super().__init__('waypoint_executor')
        self.publisher = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.gripper_client = self.create_client(GripperControlMsg, '/gripper_service')

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

    def load_waypoints(self, filename):
        try:
            with open(filename, "r") as f:
                waypoints = json.load(f)
                self.get_logger().info(f"Loaded waypoints from {filename}")
                return waypoints
        except Exception as e:
            self.get_logger().error(f"Failed to load {filename}: {e}")
            return {}

    def execute_waypoints(self, waypoints: dict, reverse=False, gripper_actions=None):
        """Executes a dictionary of waypoints. Optionally send gripper actions at specific keys."""
        keys = sorted(waypoints.keys(), key=lambda k: list(waypoints).index(k))  # keep order
        if reverse:
            keys.reverse()

        for key in keys:
            waypoint_deg = waypoints[key]
            waypoint_rad = [math.radians(j) for j in waypoint_deg]

            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = waypoint_rad
            point.time_from_start.sec = 2

            traj_msg.points = [point]

            self.get_logger().info(f"Moving to {key}: {waypoint_deg}")
            self.publisher.publish(traj_msg)
            self.wait_until_reached(waypoint_rad)
            time.sleep(1)

            # Gripper action at specific waypoint
            if gripper_actions and key in gripper_actions:
                cmd, grip = gripper_actions[key]
                self.call_gripper_service(cmd, grip)

    def wait_until_reached(self, target_positions, threshold=0.001, timeout=30.0):
        from sensor_msgs.msg import JointState
        current_positions = [None] * len(target_positions)
        joint_state_received = False

        def joint_state_cb(msg):
            nonlocal current_positions, joint_state_received
            name_to_pos = dict(zip(msg.name, msg.position))
            current_positions = [name_to_pos[name] for name in self.joint_names]
            joint_state_received = True

        sub = self.create_subscription(JointState, '/joint_states', joint_state_cb, 10)
        start_time = self.get_clock().now().seconds_nanoseconds()[0]

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if not joint_state_received:
                continue
            all_close = all(abs(c - t) < threshold for c, t in zip(current_positions, target_positions))
            if all_close:
                self.get_logger().info("Target reached.")
                break
            if self.get_clock().now().seconds_nanoseconds()[0] - start_time > timeout:
                self.get_logger().warn("Timeout waiting for robot to reach target.")
                break

        self.destroy_subscription(sub)

    def set_payload(self, mass: float):
        client = self.create_client(SetPayload, '/io_and_status_controller/set_payload')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetPayload service not available")
            return

        req = SetPayload.Request()
        req.mass = mass
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Payload set to {mass} kg")
        else:
            self.get_logger().error("Failed to set payload")

    def call_gripper_service(self, command: str, grip: int):
        if not self.gripper_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Gripper service not available")
            return

        req = GripperControlMsg.Request()
        req.command = command
        req.grip = grip

        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Gripper {command} success, result: {future.result().results}")
        else:
            self.get_logger().error("Gripper command failed")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointExecutor()

    # === Phase 1: Tool exchange ===
    tool_sequence = [
        {"file": "dock_2_gripper.json", "reverse": True, "payload": 1.80},
        {"file": "dock_1_gripper.json", "reverse": False, "payload": 1.80},
    ]

    for step in tool_sequence:
        waypoints = node.load_waypoints(step["file"])
        if not waypoints:
            continue
        if "payload" in step:
            node.set_payload(step["payload"])
        node.execute_waypoints(waypoints, reverse=step.get("reverse", False))

    # === Phase 2: Pick and place with gripper ===
    node.get_logger().info("Launching gripper service node...")
    gripper_proc = subprocess.Popen(["ros2", "run", "gripper_service", "gripper_service"])
    time.sleep(3)  # Allow time for it to initialize

    pickplace_file = "pick_place_sequence.json"
    wp = node.load_waypoints(pickplace_file)

    if wp:
        node.execute_waypoints({"pickup_approach": wp["pickup_approach"]})
        node.execute_waypoints({"pickup_approach": wp["pickup_approach"]})
        node.execute_waypoints({"pickup": wp["pickup"]})
        # time.sleep(1)
        node.call_gripper_service("CLOSE", 100)
        time.sleep(1)
        node.execute_waypoints({"pickup_approach": wp["pickup_approach"]})  # back to approach

        node.execute_waypoints({"place_approach": wp["place_approach"]})
        node.execute_waypoints({"place": wp["place"]})
        node.call_gripper_service("OPEN", 100)
        node.execute_waypoints({"place_approach": wp["place_approach"]})  # back to approach


    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
