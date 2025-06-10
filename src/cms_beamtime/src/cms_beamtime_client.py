#!/usr/bin/env python3
import json
import os
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from cms_beamtime_interfaces.action import PickPlaceControlMsg


class SimpleClient(Node):
    def __init__(self):
        super().__init__("cms_beamtime_client")
        self._action_client = ActionClient(self, PickPlaceControlMsg, "cms_beamtime_action_server")
        self._goal_handle = None

    def load_and_convert_json(self, filename: str) -> str:
        
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

        try:
            for pose_name, angles_deg in data["poses"].items():
                if not isinstance(angles_deg, list) or len(angles_deg) != 6:
                    raise ValueError(f"Pose '{pose_name}' does not have a 6-element list.")
                # Convert angles from degrees to radians
                data["poses"][pose_name] = [math.radians(angle) for angle in angles_deg]
        except Exception as e:
            self.get_logger().error(f"Error converting poses to radians: {e}")
            return ""
        # Convert back to JSON string
        try:
            converted_raw = json.dumps(data)
            self.get_logger().info(f"Loaded JSON from {filename}")
            return converted_raw
        except Exception as e:
            self.get_logger().error(f"Error converting JSON: {e}")
            return ""

    def send_json_sequence(self, json_filepath: str):
        
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
        feedback = feedback_msg.feedback
        pct = math.ceil(feedback.status * 100)
        self.get_logger().info(f"Completion percentage: {pct} %")
        
    def goal_response_callback(self, future):
        
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("Goal rejected by server.")
            return
        self.get_logger().info("Goal accepted. Waiting for result...")
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Sequence completed successfully!")
        else:
            self.get_logger().error("Sequence failed or was aborted.")


def main(args=None):
    rclpy.init(args=args)
    client = SimpleClient()

    if len(sys.argv) < 2:
        print(
            "Error: You must supply a JSON file path.\n"
            "Usage: ros2 run <your_package> cms_beamtime_client <path_to_json_file>"
        )
        sys.exit(1)

    json_file = sys.argv[1]

    client.send_json_sequence(json_file)


    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


