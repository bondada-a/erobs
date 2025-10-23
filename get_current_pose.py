#!/usr/bin/env python3
"""Get current robot pose and save to JSON

Usage:
    # Just print current pose (copy-paste ready):
    python3 get_current_pose.py

    # Save to new file:
    python3 get_current_pose.py -f poses.json -p pickup_approach -g hande

    # Update existing file:
    python3 get_current_pose.py -f beamline_test.json -p new_pose --update
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import math
import sys
import argparse
import os


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


def main(args=None):
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Capture current robot pose and save to JSON')
    parser.add_argument('-f', '--file', type=str,
                        help='Output JSON file path')
    parser.add_argument('-p', '--pose', type=str,
                        help='Name for the captured pose (e.g., "dock_approach", "pickup")')
    parser.add_argument('-g', '--gripper', type=str, default='hande',
                        help='Gripper type (default: hande)')
    parser.add_argument('--update', action='store_true',
                        help='Update existing file instead of creating new one')

    cli_args = parser.parse_args(args)

    # Initialize ROS 2
    rclpy.init()
    node = PoseCapture()

    if cli_args.pose:
        print(f"Getting current robot pose for '{cli_args.pose}'...")
    else:
        print("Getting current robot pose...")

    # Spin once to get the pose
    while not node.pose_received:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.current_pose:
        # If no file specified, just print the pose in JSON format
        if not cli_args.file:
            print(f"\nCurrent pose: {json.dumps(node.current_pose)}")
        else:
            # Pose name is required if file is specified
            if not cli_args.pose:
                print("\nError: --pose (-p) is required when saving to a file")
                node.destroy_node()
                rclpy.shutdown()
                return

            # Check if we should update existing file or create new one
            if cli_args.update and os.path.exists(cli_args.file):
                # Load existing JSON
                with open(cli_args.file, 'r') as f:
                    json_data = json.load(f)

                # Update the pose
                if "poses" not in json_data:
                    json_data["poses"] = {}
                json_data["poses"][cli_args.pose] = node.current_pose

                print(f"\n✓ Updated pose '{cli_args.pose}' in {cli_args.file}")
            else:
                # Create new JSON structure
                json_data = {
                    "start_gripper": cli_args.gripper,
                    "poses": {
                        cli_args.pose: node.current_pose
                    },
                    "tasks": [
                        {
                            "task_type": "moveto",
                            "target": cli_args.pose,
                            "planning_type": "joint"
                        }
                    ]
                }
                print(f"\n✓ Created new file {cli_args.file}")

            # Save to file
            with open(cli_args.file, 'w') as f:
                json.dump(json_data, f, indent=2)

            print(f"  {cli_args.pose}: {node.current_pose}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
