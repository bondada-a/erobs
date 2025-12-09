#!/usr/bin/env python3
"""MTC Python Client - Send task sequences to the MTC Orchestrator.

Usage:
    ros2 run mtc_py mtc_py_client.py /path/to/task_sequence.json
"""

import argparse
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from mtc_py.action import MTCExecution


class TaskGoalSender(Node):
    def __init__(self):
        super().__init__('task_goal_sender')
        self._client = ActionClient(self, MTCExecution, 'mtc_execution_py')
        self._goal_handle = None

    def send_goal(self, json_content: str) -> bool:
        """Send the task goal and wait for completion."""

        self.get_logger().info('Waiting for mtc_execution_py action server...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return False

        goal = MTCExecution.Goal()
        goal.full_json = json_content

        self.get_logger().info('Sending task goal...')

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future)

        self._goal_handle = send_future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return False

        self.get_logger().info('Goal accepted, executing...')

        # Wait for result
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'✅ Task completed successfully! '
                f'Steps: {result.result.completed_steps}/{result.result.total_steps}'
            )
            return True
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(f'Task was canceled: {result.result.error_message}')
            return False
        else:
            self.get_logger().error(
                f'❌ Task failed: {result.result.error_message} '
                f'(completed {result.result.completed_steps} steps)'
            )
            return False

    def _feedback_callback(self, feedback_msg):
        """Handle progress feedback."""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[{fb.progress_percentage:.0f}%] Step {fb.current_step}: '
            f'{fb.current_action} | Gripper: {fb.current_gripper} | {fb.status_message}'
        )


def main():
    parser = argparse.ArgumentParser(description='Send task sequence to MTC Orchestrator')
    parser.add_argument('json_file', type=str, help='Path to task sequence JSON file')

    args = parser.parse_args()

    # Read and validate JSON file
    json_path = Path(args.json_file)
    if not json_path.exists():
        print(f'Error: File not found: {json_path}', file=sys.stderr)
        sys.exit(1)

    try:
        with open(json_path) as f:
            json_content = f.read()
        # Validate it's valid JSON
        json.loads(json_content)
    except json.JSONDecodeError as e:
        print(f'Error: Invalid JSON in {json_path}: {e}', file=sys.stderr)
        sys.exit(1)
    except IOError as e:
        print(f'Error reading file: {e}', file=sys.stderr)
        sys.exit(1)

    print(f'Loaded task sequence from: {json_path}')

    # Send the goal
    rclpy.init()
    node = TaskGoalSender()
    exit_code = 1

    try:
        success = node.send_goal(json_content)
        exit_code = 0 if success else 1
    except KeyboardInterrupt:
        print('\nCanceled by user')
        if node._goal_handle:
            try:
                cancel_future = node._goal_handle.cancel_goal_async()
                # Spin to actually send the cancel request (with timeout)
                rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=2.0)
            except KeyboardInterrupt:
                pass  # User hit Ctrl+C again, just exit
        exit_code = 130
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shut down

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
