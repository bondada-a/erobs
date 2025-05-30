"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""

import math
import time
import json
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from pdf_beamtime_interfaces.action import PickPlaceControlMsg


class SimpleClient(Node):
    """Send a simple goal request to the action server."""

    def __init__(self):
        """Python init."""
        super().__init__("pdf_beamtime_client")
        self._action_client = ActionClient(self, PickPlaceControlMsg, "pdf_beamtime_action_server")
        self._goal_handle = None

        # Load positions from recorded_poses.json
        self.positions = self.load_positions()

    def load_positions(self):
        """Load positions from recorded_poses.json."""
        current_directory = os.getcwd()
        filename = os.path.join(current_directory, "recorded_poses.json")

        try:
            with open(filename, "r") as f:
                positions = json.load(f)
                self.get_logger().info(f"Loaded positions from {filename}: {positions}")
                return positions
        except FileNotFoundError:
            self.get_logger().error(f"File {filename} not found!")
            return None
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON from {filename}!")
            return None

    def send_pickup_goal(self):
        """Send a working goal."""
        if not self.positions:
            self.get_logger().error("No positions loaded. Cannot send pickup goal.")
            return

        goal_msg = PickPlaceControlMsg.Goal()

        # Use positions loaded from recorded_poses.json
        goal_msg.pickup_approach = [x / 180 * math.pi for x in self.positions["pickup_approach"]]
        goal_msg.pickup = [x / 180 * math.pi for x in self.positions["pickup"]]
        goal_msg.place_approach = [x / 180 * math.pi for x in self.positions["place_approach"]]
        goal_msg.place = [x / 180 * math.pi for x in self.positions["place"]]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        """Display the feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info("Completion percentage: {0} %".format(math.ceil(feedback.status * 100)))

    def goal_response_callback(self, future):
        """Send a cancellation after 15 seconds."""
        self._goal_handle = future.result()
        time.sleep(15.0)
        self.get_logger().warn("********** Goal Canceling Now *********")
        self._goal_handle.cancel_goal_async()

    def send_return_sample_goal(self):
        """Send a working goal."""
        if not self.positions:
            self.get_logger().error("No positions loaded. Cannot send return sample goal.")
            return

        goal_msg = PickPlaceControlMsg.Goal()

        # Example: Modify this if you want to use different positions for return sample
        goal_msg.pickup_approach = [x / 180 * math.pi for x in self.positions["pickup_approach"]]
        
        # Uncomment and modify these if needed:
        # goal_msg.pickup = [x / 180 * math.pi for x in self.positions["pickup"]]
        # goal_msg.place_approach = [x / 180 * math.pi for x in self.positions["place_approach"]]
        # goal_msg.place = [x / 180 * math.pi for x in self.positions["place"]]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)


def main(args=None):
    """Python main."""
    rclpy.init(args=args)

    client = SimpleClient()

    # Example: Send pickup or return sample goals based on your requirements
    client.send_pickup_goal()
    # client.send_return_sample_goal()

    rclpy.spin(client)

    client.destroy_node()


if __name__ == "__main__":
    main()
