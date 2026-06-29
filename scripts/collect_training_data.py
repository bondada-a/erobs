#!/usr/bin/env python3
"""Collect training images from Zivid camera via beambot-mcp-server's cached image.

Usage:
    source install/setup.bash
    python3 scripts/collect_training_data.py [output_dir]

Press Enter to trigger capture + save. Type 'q' to quit.
Uses the same ROS2 bridge as the MCP server for reliable Zivid capture.
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

try:
    import cv2
    from cv_bridge import CvBridge
except ImportError:
    print("Need: pip install opencv-python")
    sys.exit(1)


class Collector(Node):
    def __init__(self, output_dir):
        super().__init__('training_data_collector')
        self.output_dir = output_dir
        self.bridge = CvBridge()
        self.last_image = None

        # Subscribe to Zivid color image with RELIABLE + VOLATILE
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.sub = self.create_subscription(
            Image, '/color/image_color', self._on_image, qos)

        # Capture trigger service client
        self.capture_client = self.create_client(Trigger, '/capture')

        self.get_logger().info(f'Ready. Saving to {output_dir}')

    def _on_image(self, msg):
        self.last_image = msg

    def trigger_and_save(self, filepath):
        """Trigger Zivid capture, wait for image, save."""
        self.last_image = None

        # Trigger capture
        if not self.capture_client.wait_for_service(timeout_sec=3.0):
            print("  /capture service not available")
            return False

        future = self.capture_client.call_async(Trigger.Request())

        # Wait for capture to complete
        start = time.time()
        while not future.done() and time.time() - start < 10:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not future.done():
            print("  Capture trigger timed out")
            return False

        # Wait for image callback
        start = time.time()
        while self.last_image is None and time.time() - start < 10:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.last_image is None:
            print("  No image received")
            return False

        # Save
        img = self.bridge.imgmsg_to_cv2(self.last_image, 'bgr8')
        cv2.imwrite(filepath, img)
        return True


def main():
    output_dir = sys.argv[1] if len(sys.argv) > 1 else "src/beambot/data/spincoater_training"
    os.makedirs(output_dir, exist_ok=True)

    existing = sorted([f for f in os.listdir(output_dir) if f.endswith('.jpg')])
    count = len(existing) + 1

    rclpy.init()
    node = Collector(output_dir)

    # Spin briefly to let subscriptions connect
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"Saving to: {output_dir}")
    print(f"Starting at image #{count} ({len(existing)} existing)")
    print("Press Enter to capture, 'q' to quit\n")

    try:
        while True:
            user = input(f"[{count:03d}] Enter=capture, q=quit: ").strip()
            if user.lower() == 'q':
                break

            filename = f"{count:03d}.jpg"
            filepath = os.path.join(output_dir, filename)

            if node.trigger_and_save(filepath):
                print(f"  Saved {filename}")
                count += 1
            else:
                print("  Failed — try again")
    except KeyboardInterrupt:
        pass

    new_count = count - (len(existing) + 1)
    total = len([f for f in os.listdir(output_dir) if f.endswith('.jpg')])
    print(f"\nDone. Captured {new_count} new images. Total: {total}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
