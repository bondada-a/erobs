#!/usr/bin/env python3
import json, math, os, sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cms_beamtime_interfaces.action import PickPlaceControlMsg

class CmsBeamtimeClient(Node):
    def __init__(self):
        super().__init__('cms_beamtime_client')
        self.declare_parameter('action_name', 'cms_beamtime_action_server')
        action_name = self.get_parameter('action_name').value
        self._action_client = ActionClient(self, PickPlaceControlMsg, action_name)

    def send_goal(self):
        # Build an empty goal (server ignores its contents when using JSON param-file)
        goal_msg = PickPlaceControlMsg.Goal()

        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available.')
            return

        self.get_logger().info('Sending goal…')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _feedback_cb(self, feedback_msg):
        pct = int(math.ceil(feedback_msg.feedback.status * 100.0))
        self.get_logger().info(f'Progress: {pct}%')

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return
        self.get_logger().info('Goal accepted, waiting for result…')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('✅ Completed successfully!')
        else:
            self.get_logger().error('❌ Failed or aborted.')

def main():
    rclpy.init()
    node = CmsBeamtimeClient()

    if len(sys.argv) < 2:
        print('Usage: cms_beamtime_client.py <path_to_sequence.json>')
        sys.exit(1)

    # The server reads the SAME JSON file via its sequence_file param,
    # so we just invoke send_goal() without re-sending JSON as a field.
    node.send_goal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
