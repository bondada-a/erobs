#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math
import json

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self.joint_positions = None
        self.last_update_time = 0
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        self.joint_positions = msg.position
        self.last_update_time = time.time()

    def record_waypoint(self, waypoint_index: int):
        self.get_logger().info(f"Move the robot to the position for Waypoint {waypoint_index} and press Enter.")
        input("Press Enter to record the current pose...")

        old_time = self.last_update_time
        timeout_end = time.time() + 3.0

        while time.time() < timeout_end and self.last_update_time <= old_time:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.last_update_time <= old_time:
            self.get_logger().error("Failed to receive a new joint state message!")
            return None

        if self.joint_positions is None:
            self.get_logger().error("No joint state received yet!")
            return None

        reordered_positions_rad = [self.joint_positions[-1]] + list(self.joint_positions[:-1])
        pose_in_degrees = [round(math.degrees(x), 2) for x in reordered_positions_rad]

        self.get_logger().info(f"Recorded Waypoint {waypoint_index}: {pose_in_degrees}")
        return pose_in_degrees

def main(args=None):
    rclpy.init(args=args)
    recorder = WaypointRecorder()

    # Wait briefly for initial messages
    for _ in range(10):
        rclpy.spin_once(recorder, timeout_sec=0.1)

    if recorder.joint_positions is None:
        recorder.get_logger().error("No joint states received. Is the robot running?")
        recorder.destroy_node()
        rclpy.shutdown()
        return

    poses = {}
    for i in range(1, 6):  # Record 5 waypoints
        pose = recorder.record_waypoint(i)
        if pose is not None:
            poses[f"waypoint_{i}"] = pose
        else:
            recorder.get_logger().warn(f"Failed to record Waypoint {i}.")

    recorder.get_logger().info("All recorded waypoints:")
    for label, pose in poses.items():
        print(f"{label}: {pose}")

    filename = input("Enter filename to save waypoints (e.g., 'dock_gripper.json'): ")
    with open(filename, "w") as f:
        json.dump(poses, f, indent=4)
    print("Waypoints saved to 'waypoints.json'.")

    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
