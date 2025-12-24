#!/usr/bin/env python3
"""Point cloud relay node with voxel downsampling.

Downsamples large point clouds (e.g., 5M points from Zivid) to a manageable
size for octomap processing. Without downsampling, octomap would take seconds
to process each cloud, causing dropped messages and high CPU usage.

Data flow:
    Zivid (/points/xyz, ~5M points)
        → voxel downsample (1cm grid)
            → publish (/points/xyz_relayed, ~10k points)
                → octomap_server

Note: QoS bridging is NOT needed here. RELIABLE publisher (Zivid) can send
to BEST_EFFORT subscriber (octomap) directly. This relay exists purely for
the downsampling functionality.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

# Optional Open3D import for voxel downsampling
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False


class PointCloudRelay(Node):
    def __init__(self):
        super().__init__('pointcloud_relay')

        # Declare parameters
        self.declare_parameter('input_topic', '/points/xyz')
        self.declare_parameter('output_topic', '/points/xyz_relayed')
        self.declare_parameter('voxel_size', 0.02)  # 2cm default (matches octomap resolution)
        self.declare_parameter('enable_downsampling', True)
        self.declare_parameter('enable_noise_filter', False)  # Statistical outlier removal
        self.declare_parameter('noise_nb_neighbors', 20)  # Neighbors for outlier detection
        self.declare_parameter('noise_std_ratio', 2.0)  # Std dev threshold

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.enable_downsampling = self.get_parameter('enable_downsampling').value
        self.enable_noise_filter = self.get_parameter('enable_noise_filter').value
        self.noise_nb_neighbors = self.get_parameter('noise_nb_neighbors').value
        self.noise_std_ratio = self.get_parameter('noise_std_ratio').value

        # Check Open3D availability if downsampling is requested
        if self.enable_downsampling and not OPEN3D_AVAILABLE:
            self.get_logger().warn(
                'Downsampling requested but Open3D not available. '
                'Install with: pip install open3d. Disabling downsampling.'
            )
            self.enable_downsampling = False

        # QoS settings - match Zivid's RELIABLE QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5  # Buffer a few messages during processing
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.callback,
            qos
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            qos
        )

        modes = []
        if self.enable_downsampling:
            modes.append(f"voxel={self.voxel_size}m")
        if self.enable_noise_filter:
            modes.append(f"noise_filter(k={self.noise_nb_neighbors}, std={self.noise_std_ratio})")
        mode_str = ", ".join(modes) if modes else "passthrough"
        self.get_logger().info(
            f'Relaying {input_topic} -> {output_topic} ({mode_str})'
        )

    def callback(self, msg: PointCloud2):
        original_count = msg.width * msg.height

        if self.enable_downsampling:
            # Convert to Open3D, downsample, convert back
            downsampled_msg = self._downsample_pointcloud(msg)
            if downsampled_msg is not None:
                self.publisher.publish(downsampled_msg)
                new_count = downsampled_msg.width * downsampled_msg.height
                reduction = (1 - new_count / original_count) * 100
                self.get_logger().info(
                    f'Relayed {original_count:,} -> {new_count:,} points '
                    f'({reduction:.1f}% reduction)'
                )
            else:
                # Fallback to original if downsampling fails
                self.publisher.publish(msg)
                self.get_logger().warn(
                    f'Downsampling failed, relayed original {original_count:,} points'
                )
        else:
            self.publisher.publish(msg)
            self.get_logger().info(f'Relayed {original_count:,} points (passthrough)')

    def _downsample_pointcloud(self, msg: PointCloud2) -> PointCloud2:
        """Downsample point cloud using Open3D voxel grid filter."""
        try:
            # Read raw data buffer directly for maximum performance
            # Zivid /points/xyz format: x, y, z as FLOAT32 (may have padding to 16 bytes)
            data = np.frombuffer(msg.data, dtype=np.uint8)

            # Determine point layout from message
            point_step = msg.point_step  # Bytes per point (typically 12 or 16 for XYZ)
            num_points = msg.width * msg.height

            # Find X, Y, Z field offsets
            x_offset = y_offset = z_offset = None
            for field in msg.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset

            if x_offset is None or y_offset is None or z_offset is None:
                self.get_logger().error('Missing x, y, or z fields in point cloud')
                return None

            # Reshape data to (num_points, point_step) for easy field extraction
            data_reshaped = data.reshape(num_points, point_step)

            # Extract X, Y, Z as float32 arrays (view into raw bytes)
            x = data_reshaped[:, x_offset:x_offset+4].view(np.float32).flatten()
            y = data_reshaped[:, y_offset:y_offset+4].view(np.float32).flatten()
            z = data_reshaped[:, z_offset:z_offset+4].view(np.float32).flatten()

            # Filter out NaN points
            valid_mask = ~(np.isnan(x) | np.isnan(y) | np.isnan(z))
            x = x[valid_mask]
            y = y[valid_mask]
            z = z[valid_mask]

            if len(x) == 0:
                self.get_logger().warn('No valid points in cloud after NaN filtering')
                return None

            # Stack into Nx3 array for Open3D (needs float64)
            points_np = np.column_stack([x, y, z]).astype(np.float64)

            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_np)

            # Apply statistical outlier removal if enabled (removes noise/isolated points)
            if self.enable_noise_filter:
                pcd, inlier_indices = pcd.remove_statistical_outlier(
                    nb_neighbors=self.noise_nb_neighbors,
                    std_ratio=self.noise_std_ratio
                )

            # Voxel downsample
            pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            points_down = np.asarray(pcd_down.points, dtype=np.float32)

            # Convert back to PointCloud2
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]

            cloud_msg = point_cloud2.create_cloud(header, fields, points_down)
            return cloud_msg

        except Exception as e:
            self.get_logger().error(f'Downsampling error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
