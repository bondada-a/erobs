#!/usr/bin/env python3
"""Collect 2D flash-lit images for YOLO training dataset.

Triggers /capture_2d repeatedly, saving each frame with a sequential filename.
Spin the chuck between captures to get varied sample orientations.

Usage:
    python3 collect_dataset.py <output_dir> [--interval 3] [--count 50]

    output_dir:  Directory to save images (created if needed)
    --interval:  Seconds between captures (default 3, gives you time to spin chuck)
    --count:     Number of images to capture (default 50, Ctrl+C to stop early)

Example:
    python3 collect_dataset.py ~/datasets/spincoater_sample --interval 4 --count 100
"""
import argparse
import os
import sys
import time
import subprocess
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class DatasetCollector(Node):
    def __init__(self, output_dir, interval, count):
        super().__init__('dataset_collector')
        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.interval = interval
        self.max_count = count
        self.captured = 0
        self.pending = False
        self.sub = self.create_subscription(Image, '/color/image_color', self.cb, 10)

    def cb(self, msg):
        if not self.pending:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        fname = os.path.join(self.output_dir, f'img_{self.captured:04d}.png')
        cv2.imwrite(fname, img)
        self.captured += 1
        self.pending = False
        print(f'  [{self.captured}/{self.max_count}] Saved {fname}')

    def trigger_capture(self):
        subprocess.run(
            ['ros2', 'service', 'call', '/capture_2d', 'std_srvs/srv/Trigger'],
            capture_output=True, timeout=15
        )


def main():
    parser = argparse.ArgumentParser(description='Collect 2D images for YOLO dataset')
    parser.add_argument('output_dir', help='Directory to save images')
    parser.add_argument('--interval', type=float, default=3.0,
                        help='Seconds between captures (default 3)')
    parser.add_argument('--count', type=int, default=50,
                        help='Number of images to capture (default 50)')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    existing = len([f for f in os.listdir(args.output_dir) if f.endswith('.png')])
    if existing > 0:
        print(f'Note: {existing} images already in {args.output_dir}')

    rclpy.init()
    node = DatasetCollector(args.output_dir, args.interval, args.count)

    print(f'=== Dataset Collection ===')
    print(f'Output: {args.output_dir}')
    print(f'Interval: {args.interval}s | Target: {args.count} images')
    print(f'Spin the chuck between captures for varied orientations.')
    print(f'Press Ctrl+C to stop early.\n')

    try:
        while node.captured < node.max_count:
            node.pending = True
            threading.Thread(target=node.trigger_capture, daemon=True).start()

            # Wait for image
            t0 = time.time()
            while node.pending and time.time() - t0 < 15:
                rclpy.spin_once(node, timeout_sec=0.1)

            if node.pending:
                print('  WARNING: capture timed out, retrying...')
                node.pending = False
                continue

            # Wait interval before next capture
            if node.captured < node.max_count:
                print(f'  Waiting {args.interval}s... (spin the chuck now)')
                time.sleep(args.interval)

    except KeyboardInterrupt:
        print(f'\n\nStopped early.')

    print(f'\nDone! Captured {node.captured} images in {args.output_dir}')
    print(f'Next: annotate with bounding boxes (e.g. Roboflow, CVAT, or Label Studio)')
    print(f'      then train with: yolo detect train data=dataset.yaml model=yolov8n.pt epochs=100')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
