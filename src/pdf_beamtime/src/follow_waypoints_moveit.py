#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import os
import math
import time
from ur_msgs.srv import SetPayload
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.publisher import Publisher


class WaypointExecutor(Node):
    def __init__(self):
        super().__init__('waypoint_executor')
        self.publisher = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

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

    
    def execute_waypoints(self):
        sorted_keys = sorted(self.waypoints.keys(), key=lambda k: int(k.split('_')[-1]))

        for idx, key in enumerate(sorted_keys):
            waypoint_deg = self.waypoints[key]
            waypoint_rad = [math.radians(j) for j in waypoint_deg]

            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = waypoint_rad
            point.time_from_start.sec = 2

            traj_msg.points = [point]

            self.get_logger().info(f"Sending {key}: {waypoint_deg}")
            self.publisher.publish(traj_msg)

            self.wait_until_reached(waypoint_rad)
            time.sleep(1)

            if idx == 1:  # After reaching the second waypoint
                self.set_payload(1.80)



    def wait_until_reached(self, target_positions, threshold=0.001, timeout=30.0):
        """Waits until the robot's joint positions are within a threshold of the target."""
        from sensor_msgs.msg import JointState

        current_positions = [None] * len(target_positions)
        joint_state_received = False

        def joint_state_cb(msg):
            nonlocal current_positions, joint_state_received
            name_to_pos = dict(zip(msg.name, msg.position))
            current_positions = [name_to_pos[name] for name in self.joint_names]
            joint_state_received = True

        # Create a temporary subscription to joint states
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

            now = self.get_clock().now().seconds_nanoseconds()[0]
            if now - start_time > timeout:
                self.get_logger().warn("Timeout waiting for robot to reach target.")
                break

        self.destroy_subscription(sub)

    def set_payload(self, mass: float):
        from ur_msgs.srv import SetPayload

        client = self.create_client(SetPayload, '/io_and_status_controller/set_payload')

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetPayload service not available")
            return

        req = SetPayload.Request()
        req.mass = mass
        

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Payload set to {mass} kg successfully")
        else:
            self.get_logger().error("Failed to set payload")




def main(args=None):
    rclpy.init(args=args)
    node = WaypointExecutor()

    if node.waypoints:
        node.execute_waypoints()
    else:
        node.get_logger().error("No waypoints loaded.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
