#!/usr/bin/env python3
"""Client that sends multi-step tasks to the orchestrator"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from hello_orchestrator.action import OrchestratorTask
from ament_index_python.packages import get_package_share_directory
import json
import os
import sys


class TaskClient(Node):
    def __init__(self):
        super().__init__('task_client')
        self._action_client = ActionClient(
            self, OrchestratorTask, 'orchestrator_task'
        )

    def send_task(self, task_dict):
        """Send a task to the orchestrator"""
        goal = OrchestratorTask.Goal()
        goal.task_json = json.dumps(task_dict)

        self.get_logger().info('Waiting for orchestrator action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending task with %d steps' % len(task_dict['tasks']))
        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        if result.success:
            self.get_logger().info(
                f'✅ Task completed successfully! ({result.completed_steps} steps)'
            )
        else:
            self.get_logger().error(
                f'❌ Task failed: {result.error_message} (completed {result.completed_steps} steps)'
            )

        return result.success

    def feedback_callback(self, feedback_msg):
        """Handle feedback from orchestrator"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Step {feedback.current_step}/{feedback.total_steps}: {feedback.current_action}'
        )


def main(args=None):
    rclpy.init(args=args)
    client = TaskClient()

    # Load task from JSON file
    # Default to demo_task.json, or accept file path as command-line argument
    if len(sys.argv) > 1:
        task_file = sys.argv[1]
    else:
        package_share_dir = get_package_share_directory('hello_orchestrator')
        task_file = os.path.join(package_share_dir, 'config', 'demo_task.json')

    try:
        with open(task_file, 'r') as f:
            task = json.load(f)
    except FileNotFoundError:
        client.get_logger().error(f'Task file not found: {task_file}')
        client.destroy_node()
        rclpy.shutdown()
        return 1
    except json.JSONDecodeError as e:
        client.get_logger().error(f'Invalid JSON in task file: {e}')
        client.destroy_node()
        rclpy.shutdown()
        return 1

    client.get_logger().info('=' * 50)
    client.get_logger().info('HELLO ORCHESTRATOR DEMO')
    client.get_logger().info('=' * 50)
    client.get_logger().info(f'Loading task from: {task_file}')
    client.get_logger().info(f'Task contains {len(task.get("tasks", []))} steps')

    success = client.send_task(task)

    client.destroy_node()
    rclpy.shutdown()

    return 0 if success else 1


if __name__ == '__main__':
    main()
