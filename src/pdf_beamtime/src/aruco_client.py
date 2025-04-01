"""ArUco Tag-based Pick and Place Client for ROS2."""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from pdf_beamtime_interfaces.action import PickPlaceControlMsg


class ArUcoPickPlaceClient(Node):
    """Client for pick and place operations using ArUco tags."""

    def __init__(self):
        """Initialize the ArUco tag-based pick-and-place client."""
        super().__init__("aruco_pick_place_client")

        # Action client for pick and place
        self._action_client = ActionClient(self, PickPlaceControlMsg, "pdf_beamtime_action_server")

        # Subscribe to ArUco marker poses topic
        self.marker_subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',  # Topic published by your ArUco detection node
            self.marker_callback,
            10
        )

        # Initialize variables
        self.marker_detected = False
        self.marker_pose = None

        # Fixed place position (goal position) - set your desired joint angles here
        self.fixed_place_position = {
            "place_approach": [-90.0, -90.0, 90.0, 0.0, 90.0, 0.0],
            "place": [-90.0, -110.0, 110.0, 0.0, 90.0, 0.0]
        }

        self.get_logger().info("ArUco Pick and Place Client initialized")

    def marker_callback(self, msg):
        """Process detected ArUco markers."""
        if len(msg.poses) > 0:
            # Use the first detected marker (you can modify this logic if needed)
            self.marker_pose = msg.poses[0]
            self.marker_detected = True

            self.get_logger().info(f"Detected marker at position: "
                                   f"[{self.marker_pose.position.x:.3f}, "
                                   f"{self.marker_pose.position.y:.3f}, "
                                   f"{self.marker_pose.position.z:.3f}]")
        else:
            self.marker_detected = False

    def calculate_pickup_positions(self):
        """
        Calculate pickup and approach positions based on the marker pose.

        Note: This method needs to be customized based on your robot's kinematics.
        """
        if not self.marker_pose:
            self.get_logger().error("No marker pose available for pickup calculation!")
            return None

        # Get the marker position
        x = self.marker_pose.position.x
        y = self.marker_pose.position.y
        z = self.marker_pose.position.z

        # Calculate approach position (a few centimeters above the marker)
        approach_z = z + 0.05  # Adjust standoff distance as needed

        # Example joint angles (replace with your robot's IK solution)
        pickup_approach_joints = [math.atan2(y, x) * 180 / math.pi, -45.0 + (approach_z * 10), 90.0, 0.0, 90.0, 0.0]
        pickup_joints = [math.atan2(y, x) * 180 / math.pi, -45.0 + (z * 10), 90.0, 0.0, 90.0, 0.0]

        return {
            "pickup_approach": pickup_approach_joints,
            "pickup": pickup_joints
        }

    def wait_for_marker(self, timeout=60):
        """Wait for an ArUco marker to be detected."""
        start_time = time.time()
        self.get_logger().info("Waiting for an ArUco marker...")

        while not self.marker_detected and (time.time() - start_time) < timeout:
            rclpy.spin_once(self)

        if self.marker_detected:
            self.get_logger().info("ArUco marker detected!")
            return True
        else:
            self.get_logger().error(f"No marker detected after {timeout} seconds.")
            return False

    def send_pickup_place_goal(self):
        """Send the pick-and-place goal once a marker is detected."""
        # Wait for a marker to be detected
        if not self.wait_for_marker():
            return

        # Calculate positions based on the detected marker pose
        positions = self.calculate_pickup_positions()
        if not positions:
            return

        positions.update(self.fixed_place_position)

        # Create and send the goal message
        goal_msg = PickPlaceControlMsg.Goal()
        
        goal_msg.pickup_approach = [x / 180 * math.pi for x in positions["pickup_approach"]]
        goal_msg.pickup = [x / 180 * math.pi for x in positions["pickup"]]
        goal_msg.place_approach = [x / 180 * math.pi for x in positions["place_approach"]]
        goal_msg.place = [x / 180 * math.pi for x in positions["place"]]

        self.get_logger().info("Sending pick-and-place goal...")
        
        self._action_client.wait_for_server()
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response to the goal request."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected!")
            return
        
        self.get_logger().info("Goal accepted!")
        
    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Completion percentage: {feedback.status * 100:.2f}%")

    def get_result_callback(self, future):
        """Handle the result of the action."""
        result = future.result().result
        
        if result.success:
            self.get_logger().info("Pick-and-place operation completed successfully!")
        else:
            self.get_logger().error("Pick-and-place operation failed!")


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    client = ArUcoPickPlaceClient()
    
    try:
        client.send_pickup_place_goal()
        
    except KeyboardInterrupt:
        client.get_logger().info("Operation interrupted by user.")
    
    finally:
        client.destroy_node()
        
    rclpy.shutdown()


if __name__ == "__main__":
    main()
