#!/usr/bin/env python3

"""
Vision-based pick with predefined place position example.
Picks an object marked with AprilTag 3 and places it at a predefined position.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mtc_pipeline.action import VisionPickPlaceAction
import json
import sys


class VisionPickPlaceClient(Node):
    def __init__(self):
        super().__init__('vision_pick_place_client')
        self._action_client = ActionClient(
            self,
            VisionPickPlaceAction,
            'vision_pick_place_action'
        )
        self.get_logger().info('Vision Pick Place Client initialized')

    def send_goal(self, pick_tag_id=3, place_position=None, gripper='hande'):
        """
        Send a vision pick and place goal.

        Args:
            pick_tag_id: AprilTag ID to pick from
            place_position: [x, y, z] position for placing, or None for default
            gripper: 'hande' or 'epick'
        """
        goal_msg = VisionPickPlaceAction.Goal()

        # Pick configuration
        goal_msg.pick_tag_id = pick_tag_id

        # Place configuration - use predefined position instead of tag
        goal_msg.place_tag_id = -1  # -1 means use predefined position

        # Gripper type
        goal_msg.gripper = gripper

        # Configure grasp offset (5cm above tag, rotated 180° around pitch)
        # This assumes the tag is on top of the object
        grasp_offset = {
            "x": 0.0,      # No offset in X
            "y": 0.0,      # No offset in Y
            "z": 0.05,     # 5cm above tag
            "rpy": [0, 3.14159, 0]  # Flip gripper to point down
        }
        goal_msg.grasp_offset_json = json.dumps(grasp_offset)

        # Configure place position if provided
        if place_position is not None:
            place_config = {
                "place_position": place_position  # [x, y, z] in meters
            }
            goal_msg.place_poses_json = json.dumps(place_config)
        else:
            # Use default place position (defined in vision_pick_place_stages.cpp)
            # Default is x=0.4m, y=0.3m, z=0.15m
            goal_msg.place_poses_json = ""

        # Approach and retreat offsets
        goal_msg.approach_offset = 0.1   # 10cm above grasp/place position
        goal_msg.retreat_offset = 0.15   # 15cm above grasp/place position

        self.get_logger().info(f'Sending goal:')
        self.get_logger().info(f'  Pick from AprilTag: {pick_tag_id}')
        if place_position:
            self.get_logger().info(f'  Place at position: {place_position}')
        else:
            self.get_logger().info(f'  Place at default position')
        self.get_logger().info(f'  Gripper: {gripper}')

        # Wait for server
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return None

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, executing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: {feedback.current_operation} '
            f'({feedback.progress_percentage:.1f}%)'
        )

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('✓ Vision pick and place completed successfully!')
        else:
            self.get_logger().error(f'✗ Failed: {result.error_message}')

        # Shutdown after receiving result
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    client = VisionPickPlaceClient()

    # Parse command line arguments
    if len(sys.argv) > 1:
        try:
            pick_tag = int(sys.argv[1])
        except ValueError:
            client.get_logger().error(f'Invalid tag ID: {sys.argv[1]}')
            pick_tag = 3
    else:
        pick_tag = 3  # Default to tag 3

    # Optional: specify custom place position
    # Format: x,y,z (in meters)
    place_position = None
    if len(sys.argv) > 2:
        try:
            coords = sys.argv[2].split(',')
            if len(coords) == 3:
                place_position = [float(x) for x in coords]
                client.get_logger().info(f'Using custom place position: {place_position}')
        except (ValueError, IndexError):
            client.get_logger().warn('Invalid place position format. Using default.')

    # Optional: specify gripper type
    gripper = 'hande'  # default
    if len(sys.argv) > 3:
        if sys.argv[3].lower() in ['hande', 'epick']:
            gripper = sys.argv[3].lower()

    # Send the goal
    future = client.send_goal(
        pick_tag_id=pick_tag,
        place_position=place_position,
        gripper=gripper
    )

    if future is None:
        client.get_logger().error('Failed to send goal')
        rclpy.shutdown()
        return

    # Keep the node spinning
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Interrupted by user')

    client.destroy_node()


if __name__ == '__main__':
    print("\n" + "="*60)
    print("Vision Pick and Predefined Place Client")
    print("="*60)
    print("\nUsage:")
    print("  ros2 run mtc_pipeline vision_pick_predefined_place.py [tag_id] [x,y,z] [gripper]")
    print("\nExamples:")
    print("  # Pick tag 3, place at default position")
    print("  ros2 run mtc_pipeline vision_pick_predefined_place.py 3")
    print("\n  # Pick tag 3, place at x=0.4, y=0.2, z=0.1")
    print("  ros2 run mtc_pipeline vision_pick_predefined_place.py 3 0.4,0.2,0.1")
    print("\n  # Pick tag 5, custom place, use epick gripper")
    print("  ros2 run mtc_pipeline vision_pick_predefined_place.py 5 0.3,0.3,0.15 epick")
    print("="*60 + "\n")

    main()