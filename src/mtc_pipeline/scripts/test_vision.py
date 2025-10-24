#!/usr/bin/env python3
"""
Simple test script for vision system - detect tag and move to it.
Usage: python3 test_vision.py [tag_id]
"""

import rclpy
from rclpy.action import ActionClient
from mtc_pipeline.action import VisionMoveToAction
import sys

def main():
    # Get tag ID from command line, default to 0
    tag_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0

    rclpy.init()
    node = rclpy.create_node('vision_test_client')

    # Create action client
    client = ActionClient(node, VisionMoveToAction, 'vision_move_to_action')

    print(f"Waiting for vision action server...")
    if not client.wait_for_server(timeout_sec=5.0):
        print("ERROR: Vision action server not available!")
        return 1

    # Create goal
    goal = VisionMoveToAction.Goal()
    goal.tag_id = tag_id
    goal.timeout = 10.0

    print(f"Sending goal: detect and move to tag {tag_id}")

    # Send goal and wait for result
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        print("ERROR: Goal rejected!")
        return 1

    print("Goal accepted, executing...")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result().result

    if result.success:
        print(f"SUCCESS: Moved to tag {tag_id}")
        return 0
    else:
        print(f"FAILED: {result.error_message}")
        return 1

if __name__ == '__main__':
    sys.exit(main())
