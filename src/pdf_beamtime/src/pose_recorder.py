#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')
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
        self.last_update_time = time.time()  # Record when we last got a message

    def record_pose(self, label: str):
        """Record the current robot pose."""
        self.get_logger().info(f"Move the robot to the desired pose for '{label}' and press Enter.")
        input("Press Enter to record the current pose...")

        # Get fresh joint data by spinning for a while
        old_time = self.last_update_time
        timeout_end = time.time() + 3.0  # 3 second timeout
        
        # Spin until we get a new message or timeout
        while time.time() < timeout_end and self.last_update_time <= old_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.last_update_time <= old_time:
            self.get_logger().error("Failed to receive a new joint state message!")
            return None

        if self.joint_positions is None:
            self.get_logger().error("No joint state received yet!")
            return None

        # Reorder joints: move the last entry to the start
        reordered_positions_rad = [self.joint_positions[-1]] + list(self.joint_positions[:-1])

        # Convert radians to degrees using math.degrees()
        pose_in_degrees = [round(math.degrees(x), 2) for x in reordered_positions_rad]

        self.get_logger().info(f"Recorded pose for '{label}': {pose_in_degrees}")
        return pose_in_degrees

def main(args=None):
    rclpy.init(args=args)
    recorder = PoseRecorder()

    # Spin a few times to start receiving messages
    for _ in range(10):
        rclpy.spin_once(recorder, timeout_sec=0.1)
    
    if recorder.joint_positions is None:
        recorder.get_logger().error("No joint states received. Is the robot running?")
        recorder.destroy_node()
        rclpy.shutdown()
        return

    poses = {}
    labels = ['pickup_approach', 'pickup', 'place_approach', 'place']

    for label in labels:
        pose = recorder.record_pose(label)
        if pose is not None:
            poses[label] = pose
        else:
            recorder.get_logger().warn(f"No pose recorded for '{label}'.")

    recorder.get_logger().info("All recorded poses:")
    for label, pose in poses.items():
        print(f"{label}: {pose}")

    # Save the poses to a file
    save_to_file(poses)

    recorder.destroy_node()
    rclpy.shutdown()

def save_to_file(poses):
    import json
    filename = "recorded_poses.json"
    with open(filename, "w") as f:
        json.dump(poses, f, indent=4)
    print(f"Poses saved to {filename}")

if __name__ == '__main__':
    main()
