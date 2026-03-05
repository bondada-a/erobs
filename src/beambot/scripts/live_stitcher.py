#!/usr/bin/env python3
"""
Live Environment Stitcher

Subscribes to Zivid point clouds, transforms each to the robot base frame
using TF, and accumulates into a single stitched point cloud.

Publishes the growing map on /stitched_cloud for RViz visualization.
Saves to PLY on shutdown (Ctrl+C).

Usage:
    Terminal 1: ros2 launch beambot beambot_bringup.launch.py
    Terminal 2: python3 src/beambot/scripts/live_stitcher.py
    Terminal 3: ros2 service call /capture std_srvs/srv/Trigger "{}"
    RViz: Add PointCloud2 display → /stitched_cloud
"""

import struct
import signal
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import tf2_ros
from scipy.spatial.transform import Rotation


class LiveStitcher(Node):
    def __init__(self):
        super().__init__('live_stitcher')

        # Parameters
        self.voxel_size = 0.005  # 5mm voxel downsampling
        self.output_dir = Path.home() / 'work/github_ws/experimental/scan_data'
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # TF2
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Accumulated point cloud
        self.accumulated_pcd = o3d.geometry.PointCloud()
        self.capture_count = 0

        # QoS matching Zivid's RELIABLE publisher
        zivid_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # Subscribe to Zivid point cloud
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/points/xyzrgba',
            self.cloud_callback,
            zivid_qos
        )

        # Publisher for accumulated cloud (for RViz)
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            '/stitched_cloud',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Republish timer (every 3 seconds)
        self.republish_timer = self.create_timer(3.0, self.publish_accumulated)

        self.get_logger().info('='*50)
        self.get_logger().info('  LIVE STITCHER READY')
        self.get_logger().info('  Trigger captures → map builds in RViz')
        self.get_logger().info(f'  Voxel size: {self.voxel_size*1000:.0f}mm')
        self.get_logger().info(f'  Output dir: {self.output_dir}')
        self.get_logger().info('  Ctrl+C to save and exit')
        self.get_logger().info('='*50)

    def cloud_callback(self, msg: PointCloud2):
        """Process each incoming Zivid point cloud."""
        # 1. Look up transform: base_link → zivid_optical_frame
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id or 'zivid_optical_frame',
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return

        # 2. Convert PointCloud2 → Open3D
        pcd = self.pointcloud2_to_open3d(msg)
        if pcd is None or len(pcd.points) == 0:
            self.get_logger().warning('Empty point cloud received, skipping')
            return

        incoming_points = len(pcd.points)

        # 3. Build 4x4 transform matrix
        T = self.transform_to_matrix(transform_stamped.transform)

        # 4. Transform to base frame
        pcd.transform(T)

        # 5. Downsample the incoming cloud before merging
        pcd = pcd.voxel_down_sample(self.voxel_size)

        # 6. Remove statistical outliers
        if len(pcd.points) > 100:
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=20, std_ratio=2.0
            )

        # 7. Accumulate
        self.accumulated_pcd += pcd

        # 8. Downsample accumulated cloud to keep size manageable
        self.accumulated_pcd = self.accumulated_pcd.voxel_down_sample(self.voxel_size)

        self.capture_count += 1
        self.get_logger().info(
            f'✓ Capture #{self.capture_count}: '
            f'{incoming_points:,} pts in → '
            f'{len(pcd.points):,} after filter → '
            f'{len(self.accumulated_pcd.points):,} total in map'
        )

        # Publish immediately after new capture
        self.publish_accumulated()

    def publish_accumulated(self):
        """Publish accumulated cloud for RViz."""
        if len(self.accumulated_pcd.points) == 0:
            return

        msg = self.open3d_to_pointcloud2(self.accumulated_pcd)
        self.cloud_pub.publish(msg)

    def save_cloud(self):
        """Save accumulated cloud to PLY file."""
        if len(self.accumulated_pcd.points) == 0:
            self.get_logger().warning('No points to save')
            return

        filepath = self.output_dir / 'stitched_environment.ply'
        o3d.io.write_point_cloud(str(filepath), self.accumulated_pcd)
        self.get_logger().info(f'Saved {len(self.accumulated_pcd.points):,} points → {filepath}')

        # Also save a higher-res version without the aggressive downsampling
        filepath_obj = self.output_dir / 'stitched_environment.pcd'
        o3d.io.write_point_cloud(str(filepath_obj), self.accumulated_pcd)
        self.get_logger().info(f'Saved PCD → {filepath_obj}')

    # ─── Conversion utilities ─────────────────────────────────

    def pointcloud2_to_open3d(self, msg: PointCloud2) -> o3d.geometry.PointCloud:
        """Convert ROS2 PointCloud2 (XYZRGBA) to Open3D PointCloud."""
        # Parse fields to find offsets
        field_map = {f.name: f for f in msg.fields}

        if 'x' not in field_map:
            self.get_logger().error('PointCloud2 missing xyz fields')
            return None

        # Read raw data
        data = np.frombuffer(msg.data, dtype=np.uint8)

        # Compute point count
        if msg.height > 1:
            # Organized cloud
            point_count = msg.width * msg.height
        else:
            point_count = msg.width

        point_step = msg.point_step

        if len(data) < point_count * point_step:
            self.get_logger().error(
                f'Data size mismatch: {len(data)} < {point_count * point_step}'
            )
            return None

        # Reshape to (N, point_step)
        data = data[:point_count * point_step].reshape(point_count, point_step)

        # Extract XYZ (float32)
        x_off = field_map['x'].offset
        y_off = field_map['y'].offset
        z_off = field_map['z'].offset

        xyz = np.zeros((point_count, 3), dtype=np.float32)
        xyz[:, 0] = np.frombuffer(data[:, x_off:x_off+4].tobytes(), dtype=np.float32)
        xyz[:, 1] = np.frombuffer(data[:, y_off:y_off+4].tobytes(), dtype=np.float32)
        xyz[:, 2] = np.frombuffer(data[:, z_off:z_off+4].tobytes(), dtype=np.float32)

        # Remove NaN points
        valid = np.isfinite(xyz).all(axis=1)
        xyz = xyz[valid]

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))

        # Extract RGB if available
        if 'rgba' in field_map or 'rgb' in field_map:
            color_field = field_map.get('rgba', field_map.get('rgb'))
            c_off = color_field.offset

            # RGBA is packed as uint32 (BGRA byte order on little-endian)
            rgba_bytes = data[valid, c_off:c_off+4]
            b = rgba_bytes[:, 0].astype(np.float64) / 255.0
            g = rgba_bytes[:, 1].astype(np.float64) / 255.0
            r = rgba_bytes[:, 2].astype(np.float64) / 255.0

            colors = np.column_stack([r, g, b])
            pcd.colors = o3d.utility.Vector3dVector(colors)

        return pcd

    def open3d_to_pointcloud2(self, pcd: o3d.geometry.PointCloud) -> PointCloud2:
        """Convert Open3D PointCloud to ROS2 PointCloud2 for RViz."""
        points = np.asarray(pcd.points, dtype=np.float32)
        has_colors = pcd.has_colors()

        if has_colors:
            colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)

        # Build PointCloud2 message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.height = 1
        msg.width = len(points)

        if has_colors:
            msg.point_step = 16  # xyz (12) + rgba (4)
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
            ]

            buffer = bytearray(len(points) * 16)
            for i, (pt, col) in enumerate(zip(points, colors)):
                offset = i * 16
                struct.pack_into('fff', buffer, offset, pt[0], pt[1], pt[2])
                rgba = struct.pack('BBBB', col[2], col[1], col[0], 255)  # BGRA
                buffer[offset+12:offset+16] = rgba
        else:
            msg.point_step = 12  # xyz only
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            buffer = points.tobytes()

        msg.row_step = msg.point_step * msg.width
        msg.data = bytes(buffer)
        msg.is_bigendian = False
        msg.is_dense = True

        return msg

    def transform_to_matrix(self, transform) -> np.ndarray:
        """Convert geometry_msgs/Transform to 4x4 numpy matrix."""
        t = transform.translation
        r = transform.rotation

        # Quaternion to rotation matrix
        rot = Rotation.from_quat([r.x, r.y, r.z, r.w])
        matrix = np.eye(4)
        matrix[:3, :3] = rot.as_matrix()
        matrix[:3, 3] = [t.x, t.y, t.z]

        return matrix


def main():
    rclpy.init()
    node = LiveStitcher()

    # Handle Ctrl+C gracefully — save before exit
    def shutdown_handler(sig, frame):
        print('\n\nShutting down — saving point cloud...')
        node.save_cloud()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_cloud()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
