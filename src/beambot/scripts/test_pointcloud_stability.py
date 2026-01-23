#!/usr/bin/env python3
"""
Test Zivid point cloud stability.

Takes 10 consecutive captures and checks if the 3D position at the ArUco marker
varies between captures. This isolates whether the bimodal issue is in the
point cloud data itself.
"""

import os
import time

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import struct
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2

from zivid_interfaces.srv import CaptureAndDetectMarkers


class PointCloudStabilityTest(Node):
    def __init__(self):
        super().__init__('pointcloud_stability_test')

        # Service client
        self.client = self.create_client(
            CaptureAndDetectMarkers,
            '/capture_and_detect_markers'
        )

        # QoS to match Zivid
        self.zivid_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Storage
        self.received_image = None
        self.received_cloud = None

        self.get_logger().info("PointCloud Stability Test initialized")

    def capture_once(self) -> Tuple[Optional[np.ndarray], Optional[PointCloud2]]:
        """Trigger one capture and return image + point cloud."""
        self.received_image = None
        self.received_cloud = None

        # Create temporary subscriptions
        def on_image(msg):
            self.received_image = msg

        def on_cloud(msg):
            self.received_cloud = msg

        image_sub = self.create_subscription(
            Image, '/color/image_color', on_image, self.zivid_qos
        )
        cloud_sub = self.create_subscription(
            PointCloud2, '/points/xyzrgba', on_cloud, self.zivid_qos
        )

        try:
            # Wait for subscriptions to connect
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.received_image or self.received_cloud:
                    break

            # Clear stale data
            self.received_image = None
            self.received_cloud = None

            # Trigger capture
            if not self.client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("Zivid service not available")
                return None, None

            request = CaptureAndDetectMarkers.Request()
            request.marker_ids = [0]  # Tag 0
            request.marker_dictionary = "aruco4x4_50"

            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

            if not future.done():
                self.get_logger().error("Service call timed out")
                return None, None

            # Wait for data
            for i in range(200):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.received_image and self.received_cloud:
                    break

            if not self.received_image or not self.received_cloud:
                self.get_logger().error("Failed to receive image or point cloud")
                return None, None

            # Convert image
            img_msg = self.received_image
            if img_msg.encoding == 'rgba8':
                img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                    (img_msg.height, img_msg.width, 4))
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            else:
                self.get_logger().error(f"Unsupported encoding: {img_msg.encoding}")
                return None, None

            return img, self.received_cloud

        finally:
            self.destroy_subscription(image_sub)
            self.destroy_subscription(cloud_sub)

    def get_3d_at_pixel(self, cloud: PointCloud2, px: int, py: int) -> Optional[Tuple[float, float, float]]:
        """Get 3D position from point cloud at pixel (px, py)."""
        if px < 0 or px >= cloud.width or py < 0 or py >= cloud.height:
            return None

        offset = py * cloud.row_step + px * cloud.point_step
        x, y, z = struct.unpack_from('<fff', cloud.data, offset)

        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            return None

        return (x, y, z)

    def detect_marker_corners(self, img: np.ndarray, marker_id: int = 0) -> Optional[np.ndarray]:
        """Detect ArUco marker and return corner pixels."""
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(img)

        if ids is None or marker_id not in ids.flatten():
            return None

        idx = list(ids.flatten()).index(marker_id)
        return corners[idx][0]  # Shape: (4, 2)

    def run_test(self, num_captures: int = 10, marker_id: int = 0):
        """Run the stability test."""
        self.get_logger().info(f"Starting {num_captures} consecutive captures...")

        results = []

        for i in range(num_captures):
            self.get_logger().info(f"Capture {i+1}/{num_captures}...")

            img, cloud = self.capture_once()
            if img is None or cloud is None:
                self.get_logger().warn(f"Capture {i+1} failed")
                continue

            # Detect marker
            corners = self.detect_marker_corners(img, marker_id)
            if corners is None:
                self.get_logger().warn(f"Marker {marker_id} not detected in capture {i+1}")
                continue

            # Get 3D at each corner
            corner_xyz = []
            for px, py in corners:
                xyz = self.get_3d_at_pixel(cloud, int(round(px)), int(round(py)))
                if xyz:
                    corner_xyz.append(xyz)

            if len(corner_xyz) != 4:
                self.get_logger().warn(f"Only {len(corner_xyz)}/4 corners have depth")
                continue

            # Average corners
            center_x = sum(c[0] for c in corner_xyz) / 4.0 * 1000  # mm
            center_y = sum(c[1] for c in corner_xyz) / 4.0 * 1000
            center_z = sum(c[2] for c in corner_xyz) / 4.0 * 1000

            # Also store pixel center for reference
            pixel_center = np.mean(corners, axis=0)

            results.append({
                'capture': i + 1,
                'x': center_x,
                'y': center_y,
                'z': center_z,
                'pixel_x': pixel_center[0],
                'pixel_y': pixel_center[1],
            })

            self.get_logger().info(
                f"  Center: ({center_x:.2f}, {center_y:.2f}, {center_z:.2f}) mm, "
                f"Pixel: ({pixel_center[0]:.1f}, {pixel_center[1]:.1f})"
            )

            # Small delay between captures
            time.sleep(0.5)

        return results


def analyze_and_plot(results: List[dict], output_path: str):
    """Analyze results and create plot."""
    if len(results) < 2:
        print("Not enough results to analyze")
        return

    x = np.array([r['x'] for r in results])
    y = np.array([r['y'] for r in results])
    z = np.array([r['z'] for r in results])
    px = np.array([r['pixel_x'] for r in results])
    py = np.array([r['pixel_y'] for r in results])

    print("\n" + "="*60)
    print("POINT CLOUD STABILITY TEST RESULTS")
    print("="*60)

    print(f"\nRaw values (mm) - {len(results)} captures:")
    print(f"  {'#':<3} {'X':>10} {'Y':>10} {'Z':>10} {'PixelX':>10} {'PixelY':>10}")
    for r in results:
        print(f"  {r['capture']:<3} {r['x']:>10.2f} {r['y']:>10.2f} {r['z']:>10.2f} "
              f"{r['pixel_x']:>10.1f} {r['pixel_y']:>10.1f}")

    print(f"\n3D Position Statistics (camera frame):")
    print(f"  X: mean={np.mean(x):.2f}mm, σ={np.std(x):.3f}mm, range={np.ptp(x):.2f}mm")
    print(f"  Y: mean={np.mean(y):.2f}mm, σ={np.std(y):.3f}mm, range={np.ptp(y):.2f}mm")
    print(f"  Z: mean={np.mean(z):.2f}mm, σ={np.std(z):.3f}mm, range={np.ptp(z):.2f}mm")

    print(f"\nPixel Detection Statistics:")
    print(f"  Pixel X: σ={np.std(px):.3f}px, range={np.ptp(px):.2f}px")
    print(f"  Pixel Y: σ={np.std(py):.3f}px, range={np.ptp(py):.2f}px")

    # Check bimodal
    for axis_name, data in [('X', x), ('Y', y), ('Z', z)]:
        sorted_data = np.sort(data)
        gaps = np.diff(sorted_data)
        max_gap = np.max(gaps) if len(gaps) > 0 else 0
        bimodal = "YES" if max_gap > 0.5 else "NO"
        print(f"\n  {axis_name} bimodal (gap > 0.5mm): {bimodal} (max gap={max_gap:.2f}mm)")

    # Conclusion
    print("\n" + "="*60)
    print("CONCLUSION")
    print("="*60)

    total_3d_std = np.std(x) + np.std(y) + np.std(z)
    if total_3d_std < 0.5:
        print("  ✓ Point cloud is STABLE (total σ < 0.5mm)")
        print("  → Problem is NOT in point cloud data")
        print("  → Problem must be in TF transform chain")
    else:
        y_gap = np.max(np.diff(np.sort(y)))
        if y_gap > 1.0:
            print("  ✗ Point cloud shows BIMODAL pattern!")
            print("  → Problem IS in the point cloud/camera itself")
        else:
            print("  ? Point cloud has some variation but not clearly bimodal")
            print("  → Further investigation needed")

    # Create plot
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))

    # 3D positions
    for col, (data, label) in enumerate([(x, 'X'), (y, 'Y'), (z, 'Z')]):
        ax = axes[0, col]
        ax.plot(range(1, len(data)+1), data, 'o-', markersize=10, color='blue')
        ax.axhline(np.mean(data), color='r', linestyle='--', alpha=0.7)
        ax.set_xlabel('Capture #')
        ax.set_ylabel(f'{label} (mm)')
        ax.set_title(f'Camera Frame {label}: σ={np.std(data):.3f}mm, range={np.ptp(data):.2f}mm')
        ax.grid(True, alpha=0.3)

    # Pixel positions
    ax = axes[1, 0]
    ax.plot(range(1, len(px)+1), px, 'o-', markersize=10, color='green', label='Pixel X')
    ax.axhline(np.mean(px), color='r', linestyle='--', alpha=0.7)
    ax.set_xlabel('Capture #')
    ax.set_ylabel('Pixel X')
    ax.set_title(f'Pixel X: σ={np.std(px):.3f}px')
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(range(1, len(py)+1), py, 'o-', markersize=10, color='green', label='Pixel Y')
    ax.axhline(np.mean(py), color='r', linestyle='--', alpha=0.7)
    ax.set_xlabel('Capture #')
    ax.set_ylabel('Pixel Y')
    ax.set_title(f'Pixel Y: σ={np.std(py):.3f}px')
    ax.grid(True, alpha=0.3)

    # Y vs capture scatter with histogram
    ax = axes[1, 2]
    ax.hist(y, bins=20, color='blue', alpha=0.7, edgecolor='black')
    ax.set_xlabel('Y (mm)')
    ax.set_ylabel('Count')
    ax.set_title('Y Distribution (check for bimodal)')
    ax.grid(True, alpha=0.3)

    plt.suptitle('Zivid Point Cloud Stability Test\n(10 consecutive captures, same scene)',
                 fontweight='bold', fontsize=14)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"\nPlot saved: {output_path}")
    plt.close()


def main():
    rclpy.init()

    node = PointCloudStabilityTest()

    try:
        results = node.run_test(num_captures=10, marker_id=0)

        if results:
            output_path = '/home/aditya/work/github_ws/experimental/recorded_bags/analysis_new/pointcloud_stability_test.png'
            analyze_and_plot(results, output_path)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
