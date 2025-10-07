#!/usr/bin/env python3
"""Get current robot pose and save to JSON"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import math
import sys


class PoseCapture(Node):
    def __init__(self):
        super().__init__('pose_capture')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.pose_received = False
        self.current_pose = None

    def joint_state_callback(self, msg):
        # Map joint names to positions
        joint_dict = dict(zip(msg.name, msg.position))

        # UR robot joint order for JSON: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        joint_order = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Convert radians to degrees and round to 2 decimal places
        pose_deg = [round(math.degrees(joint_dict[j]), 2) for j in joint_order]

        self.current_pose = pose_deg
        self.pose_received = True
        self.get_logger().info(f'Current pose (degrees): {pose_deg}')


def main():
    rclpy.init()
    node = PoseCapture()

    print("Getting current robot pose...")

    # Spin once to get the pose
    while not node.pose_received:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.current_pose:
        # Create the JSON structure
        json_data = {
            "start_gripper": "hande",
            "poses": {
                "dock_approach": node.current_pose
            },
            "tasks": [
                {
                    "task_type": "moveto",
                    "target": "dock_approach",
                    "planning_type": "joint"
                }
            ]
        }

        # Save to file
        output_file = '/home/aditya/work/github_ws/erobs/beamline_test.json'
        with open(output_file, 'w') as f:
            json.dump(json_data, f, indent=2)

        print(f"\n✓ Saved pose to {output_file}")
        print(f"  dock_approach: {node.current_pose}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
