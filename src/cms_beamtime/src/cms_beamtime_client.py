#!/usr/bin/env python3
"""
cms_beamtime_client.py

Sends a JSON-driven sequence (with joint angles in degrees) to cms_beamtime_server,
converting all "poses" values from degrees to radians before sending. Expects a file
named recorded_poses.json in the current working directory with the following structure:

{
  "poses": {
    "<pose_name>": [6-joint-array-in-degrees],
    ...
  },
  "sequence": [
    { "type":"move", "pose_waypoints":["A","B","C"], "speed":0.5 },
    { "type":"end_effector", "device":"gripper", "action":"open" },
    ...
  ]
}

The client:
1. Reads recorded_poses.json,
2. Converts each pose array from degrees → radians,
3. Re-serializes the modified dict to a JSON string,
4. Sends that JSON string as goal.json_string to the action server.
Feedback (0–100%) prints to the console. If the goal is accepted, it waits for completion or cancellation.
"""

import json
import os
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from cms_beamtime_interfaces.action import PickPlaceControlMsg


class SimpleClient(Node):
    """Client to send a JSON sequence (degrees → radians) to the cms_beamtime action server."""

    def __init__(self):
        super().__init__("cms_beamtime_client")
        self._action_client = ActionClient(self, PickPlaceControlMsg, "cms_beamtime_action_server")
        self._goal_handle = None

    def load_and_convert_json(self, filename: str) -> str:
        """
        Load JSON from filename, convert every joint angle in 'poses' from degrees to radians,
        re-serialize, and return the resulting JSON string. If any error, returns an empty string.
        """
        if not os.path.isfile(filename):
            self.get_logger().error(f"File '{filename}' not found.")
            return ""

        try:
            with open(filename, "r") as f:
                data = json.load(f)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON from '{filename}': {e}")
            return ""
        except Exception as e:
            self.get_logger().error(f"Error reading '{filename}': {e}")
            return ""

        # Ensure top-level keys exist
        if "poses" not in data or not isinstance(data["poses"], dict):
            self.get_logger().error(f"JSON missing 'poses' dict.")
            return ""
        if "sequence" not in data or not isinstance(data["sequence"], list):
            self.get_logger().error(f"JSON missing 'sequence' list.")
            return ""

        # Convert each pose's joint array from degrees to radians
        try:
            for pose_name, angles_deg in data["poses"].items():
                if not isinstance(angles_deg, list) or len(angles_deg) != 6:
                    raise ValueError(f"Pose '{pose_name}' does not have a 6-element list.")
                # Convert in-place
                data["poses"][pose_name] = [math.radians(angle) for angle in angles_deg]
        except Exception as e:
            self.get_logger().error(f"Error converting poses to radians: {e}")
            return ""

        # Re-serialize to JSON string
        try:
            converted_raw = json.dumps(data)
            self.get_logger().info(f"Loaded and converted JSON from {filename}")
            return converted_raw
        except Exception as e:
            self.get_logger().error(f"Error serializing converted JSON: {e}")
            return ""

    def send_json_sequence(self, json_filepath: str):
        """Read a JSON file, convert poses from degrees to radians, and send its contents as goal.json_string."""
        raw_converted_json = self.load_and_convert_json(json_filepath)
        if not raw_converted_json:
            return

        goal_msg = PickPlaceControlMsg.Goal()
        goal_msg.json_string = raw_converted_json

        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server available. Sending goal.")

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Display feedback percentage."""
        feedback = feedback_msg.feedback
        pct = math.ceil(feedback.status * 100)
        self.get_logger().info(f"Completion percentage: {pct} %")

    def goal_response_callback(self, future):
        """Called when the server accepts or rejects the goal."""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("Goal rejected by server.")
            return
        self.get_logger().info("Goal accepted. Waiting for result...")
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Called when the action is complete (succeeded or aborted)."""
        result = future.result().result
        if result.success:
            self.get_logger().info("Sequence completed successfully!")
        else:
            self.get_logger().error("Sequence failed or was aborted.")


def main(args=None):
    rclpy.init(args=args)
    client = SimpleClient()
    # Send the JSON sequence using the same filename as before
    client.send_json_sequence("recorded_poses.json")
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
