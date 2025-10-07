#!/usr/bin/env python3
"""
Test script for the vision system in simulation.
This script sends vision-based movement commands to the robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mtc_pipeline.action import MTCExecution
import json
import sys
import argparse


class VisionTestClient(Node):
    def __init__(self):
        super().__init__('vision_test_client')
        self.action_client = ActionClient(
            self, MTCExecution, '/mtc_execution_action')

    def send_vision_goal(self, tag_id, approach_distance=0.1, approach_direction="z"):
        """Send a vision-based movement goal to approach a tag."""

        # Create the goal
        goal_msg = MTCExecution.Goal()
        goal_msg.robot_ip = "192.168.56.101"
        goal_msg.start_gripper = "epick"
        goal_msg.poses_json = "{}"

        # Create vision task
        steps = {
            "steps": [
                {
                    "task_type": "vision_moveto",
                    "tag_id": tag_id,
                    "approach_distance": approach_distance,
                    "timeout": 10.0,
                    "approach_direction": approach_direction,
                    "use_preset_height": True,
                    "preset_height": 0.15
                }
            ]
        }

        goal_msg.steps_json = json.dumps(steps)

        self.get_logger().info(f'Sending vision goal: approach tag {tag_id} from {approach_direction} at {approach_distance}m')

        # Wait for server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False

        # Send goal
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success:
            self.get_logger().info('Vision task completed successfully!')
        else:
            self.get_logger().error(f'Vision task failed: {result.error_message}')

        return result.success

    def run_pick_place_demo(self):
        """Run a demo sequence: approach tag 0, pick, move to tag 1, place."""

        self.get_logger().info('Starting pick and place demo with vision...')

        goal_msg = MTCExecution.Goal()
        goal_msg.robot_ip = "192.168.56.101"
        goal_msg.start_gripper = "epick"
        goal_msg.poses_json = "{}"

        # Create pick and place sequence
        steps = {
            "steps": [
                # Move to home position
                {
                    "task_type": "moveto",
                    "location": "home"
                },
                # Approach tag 0 from above
                {
                    "task_type": "vision_moveto",
                    "tag_id": 0,
                    "approach_distance": 0.1,
                    "timeout": 10.0,
                    "approach_direction": "z",
                    "use_preset_height": True,
                    "preset_height": 0.15
                },
                # Move down closer
                {
                    "task_type": "vision_moveto",
                    "tag_id": 0,
                    "approach_distance": 0.03,
                    "timeout": 5.0,
                    "approach_direction": "z",
                    "use_preset_height": False
                },
                # Close gripper
                {
                    "task_type": "endeffector",
                    "command": "close"
                },
                # Lift up
                {
                    "task_type": "moveto",
                    "location": "home"
                },
                # Move to tag 1
                {
                    "task_type": "vision_moveto",
                    "tag_id": 1,
                    "approach_distance": 0.1,
                    "timeout": 10.0,
                    "approach_direction": "z",
                    "use_preset_height": True,
                    "preset_height": 0.15
                },
                # Open gripper
                {
                    "task_type": "endeffector",
                    "command": "open"
                },
                # Return home
                {
                    "task_type": "moveto",
                    "location": "home"
                }
            ]
        }

        goal_msg.steps_json = json.dumps(steps)

        self.get_logger().info('Sending pick and place sequence...')

        # Wait for server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False

        # Send goal
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted, executing sequence...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success:
            self.get_logger().info('Pick and place demo completed successfully!')
        else:
            self.get_logger().error(f'Pick and place demo failed: {result.error_message}')

        return result.success


def main():
    parser = argparse.ArgumentParser(description='Test vision system in simulation')
    parser.add_argument('--tag', type=int, default=0,
                       help='Tag ID to approach (default: 0)')
    parser.add_argument('--distance', type=float, default=0.1,
                       help='Approach distance in meters (default: 0.1)')
    parser.add_argument('--direction', type=str, default='z',
                       choices=['x', '-x', 'y', '-y', 'z', '-z'],
                       help='Approach direction (default: z)')
    parser.add_argument('--demo', action='store_true',
                       help='Run pick and place demo sequence')

    args = parser.parse_args()

    rclpy.init()
    client = VisionTestClient()

    try:
        if args.demo:
            success = client.run_pick_place_demo()
        else:
            success = client.send_vision_goal(
                args.tag, args.distance, args.direction)

        sys.exit(0 if success else 1)

    except KeyboardInterrupt:
        client.get_logger().info('Interrupted by user')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()