#!/usr/bin/env python3
"""
Offline stitcher — reprocess a rosbag with adjustable quality settings.

Reads point clouds + TF from a recorded rosbag, transforms each to base frame,
and produces a high-quality stitched point cloud.

Usage:
    python3 stitch_from_bag.py <bag_path> [--voxel SIZE] [--no-downsample]

Examples:
    # Default 2mm voxel (good balance of quality + size)
    python3 stitch_from_bag.py recorded_bags/data/beamline/scene_capture/capture_r01

    # Full resolution (no downsampling — large file)
    python3 stitch_from_bag.py recorded_bags/data/beamline/scene_capture/capture_r01 --no-downsample

    # Custom voxel size (1mm = very detailed)
    python3 stitch_from_bag.py recorded_bags/data/beamline/scene_capture/capture_r01 --voxel 0.001
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

# ROS2 bag reading
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def read_bag_messages(bag_path: str, topics: list):
    """Read messages from a rosbag for specified topics."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    # Set filter for specific topics
    filter_ = rosbag2_py.StorageFilter(topics=topics)
    reader.set_filter(filter_)

    # Get topic type map
    topic_types = {}
    for topic_info in reader.get_all_topics_and_types():
        topic_types[topic_info.name] = topic_info.type

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp


def pointcloud2_to_open3d(msg) -> o3d.geometry.PointCloud:
    """Convert PointCloud2 (XYZRGBA) to Open3D PointCloud."""
    field_map = {f.name: f for f in msg.fields}
    point_count = msg.width * msg.height
    point_step = msg.point_step

    data = np.frombuffer(msg.data, dtype=np.uint8)
    if len(data) < point_count * point_step:
        return None
    data = data[:point_count * point_step].reshape(point_count, point_step)

    # Extract XYZ
    x_off = field_map['x'].offset
    y_off = field_map['y'].offset
    z_off = field_map['z'].offset

    xyz = np.zeros((point_count, 3), dtype=np.float32)
    xyz[:, 0] = np.frombuffer(data[:, x_off:x_off+4].tobytes(), dtype=np.float32)
    xyz[:, 1] = np.frombuffer(data[:, y_off:y_off+4].tobytes(), dtype=np.float32)
    xyz[:, 2] = np.frombuffer(data[:, z_off:z_off+4].tobytes(), dtype=np.float32)

    # Remove NaNs
    valid = np.isfinite(xyz).all(axis=1)
    xyz = xyz[valid]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))

    # Extract RGB
    if 'rgba' in field_map or 'rgb' in field_map:
        color_field = field_map.get('rgba', field_map.get('rgb'))
        c_off = color_field.offset
        rgba_bytes = data[valid, c_off:c_off+4]
        colors = np.column_stack([
            rgba_bytes[:, 2].astype(np.float64) / 255.0,  # R
            rgba_bytes[:, 1].astype(np.float64) / 255.0,  # G
            rgba_bytes[:, 0].astype(np.float64) / 255.0,  # B
        ])
        pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def transform_to_matrix(transform) -> np.ndarray:
    """Convert TFMessage transform to 4x4 matrix."""
    t = transform.translation
    r = transform.rotation
    rot = Rotation.from_quat([r.x, r.y, r.z, r.w])
    matrix = np.eye(4)
    matrix[:3, :3] = rot.as_matrix()
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def build_tf_lookup(bag_path: str):
    """
    Read all TF messages and build a lookup structure.
    Returns dict: {(parent, child): [(timestamp_ns, 4x4 matrix), ...]}
    """
    print("Reading TF data from bag...")
    tf_data = {}  # (parent, child) -> [(ts, matrix)]

    for topic, msg, timestamp in read_bag_messages(bag_path, ['/tf', '/tf_static']):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            key = (parent, child)

            ts = (transform.header.stamp.sec * 1_000_000_000 +
                  transform.header.stamp.nanosec)
            matrix = transform_to_matrix(transform.transform)

            if key not in tf_data:
                tf_data[key] = []
            tf_data[key].append((ts, matrix))

    # Sort by timestamp
    for key in tf_data:
        tf_data[key].sort(key=lambda x: x[0])

    print(f"  Loaded {len(tf_data)} transform chains")
    for key, values in tf_data.items():
        print(f"    {key[0]} → {key[1]}: {len(values)} samples")

    return tf_data


def lookup_transform(tf_data: dict, parent: str, child: str,
                     timestamp_ns: int) -> np.ndarray:
    """
    Find the closest transform for a given timestamp.
    Uses binary search for efficiency.
    """
    key = (parent, child)
    if key not in tf_data:
        return None

    entries = tf_data[key]
    timestamps = [e[0] for e in entries]

    # Binary search for closest timestamp
    idx = np.searchsorted(timestamps, timestamp_ns)
    if idx == 0:
        return entries[0][1]
    if idx >= len(entries):
        return entries[-1][1]

    # Pick whichever is closer
    before = entries[idx - 1]
    after = entries[idx]
    if abs(timestamp_ns - before[0]) <= abs(timestamp_ns - after[0]):
        return before[1]
    return after[1]


def compute_chain_transform(tf_data: dict, timestamp_ns: int,
                            target_frame: str = 'base_link',
                            source_frame: str = 'zivid_optical_frame'):
    """
    Compute full transform chain from target_frame to source_frame.

    For UR5e with Zivid:
    base_link → base → ... (robot joints) → tool0 → zivid_optical_frame

    We use base → tool0 from dynamic TF, and tool0 → zivid_optical_frame from static TF.
    """
    # The TF tree for UR5e:
    # base_link → base (static)
    # base → shoulder_link → upper_arm_link → ... → tool0 (dynamic, from joints)
    # tool0 → zivid_optical_frame (static, from hand-eye calibration)

    # Strategy: walk the chain by finding all connected transforms
    # For simplicity, find base→tool0 chain and tool0→zivid_optical_frame

    # Collect all static transforms (they don't change)
    static_transforms = {}
    for (parent, child), entries in tf_data.items():
        if len(entries) <= 5:  # Static transforms have very few entries
            static_transforms[(parent, child)] = entries[0][1]

    # Collect dynamic transform at timestamp
    dynamic_transforms = {}
    for (parent, child), entries in tf_data.items():
        if len(entries) > 5:  # Dynamic transforms have many entries
            dynamic_transforms[(parent, child)] = lookup_transform(
                tf_data, parent, child, timestamp_ns
            )

    # Build the full chain by BFS/DFS from target_frame to source_frame
    all_transforms = {}
    all_transforms.update(static_transforms)
    all_transforms.update(dynamic_transforms)

    # BFS to find path from target_frame to source_frame
    from collections import deque
    visited = {target_frame: np.eye(4)}
    queue = deque([target_frame])

    while queue:
        current = queue.popleft()
        if current == source_frame:
            return visited[source_frame]

        for (parent, child), matrix in all_transforms.items():
            if parent == current and child not in visited:
                visited[child] = visited[current] @ matrix
                queue.append(child)

    return None


def main():
    parser = argparse.ArgumentParser(description='Stitch point clouds from rosbag')
    parser.add_argument('bag_path', help='Path to rosbag directory')
    parser.add_argument('--voxel', type=float, default=0.002,
                        help='Voxel size in meters (default: 0.002 = 2mm)')
    parser.add_argument('--no-downsample', action='store_true',
                        help='Skip voxel downsampling (full resolution)')
    parser.add_argument('--output', type=str, default=None,
                        help='Output PLY file path')
    parser.add_argument('--visualize', action='store_true',
                        help='Show result in Open3D viewer after stitching')
    args = parser.parse_args()

    bag_path = args.bag_path
    output_path = args.output or str(
        Path('scan_data') / 'stitched_environment.ply'
    )
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)

    # 1. Build TF lookup from bag
    tf_data = build_tf_lookup(bag_path)

    # 2. Process point clouds
    print(f"\nProcessing point clouds from {bag_path}...")
    accumulated = o3d.geometry.PointCloud()
    count = 0

    for topic, msg, timestamp in read_bag_messages(bag_path, ['/points/xyzrgba']):
        # Get timestamp from message header
        msg_ts = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # Compute full transform chain
        T = compute_chain_transform(tf_data, msg_ts)
        if T is None:
            print(f"  ✗ Capture {count+1}: TF chain not found, skipping")
            continue

        # Convert to Open3D
        pcd = pointcloud2_to_open3d(msg)
        if pcd is None or len(pcd.points) == 0:
            continue

        incoming = len(pcd.points)

        # Transform to base frame
        pcd.transform(T)

        # Downsample individual cloud to reduce memory during accumulation
        if not args.no_downsample:
            pcd = pcd.voxel_down_sample(args.voxel)

        # Remove outliers
        if len(pcd.points) > 100:
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        accumulated += pcd
        count += 1

        print(f"  ✓ Capture {count}: {incoming:,} pts → "
              f"{len(pcd.points):,} filtered → "
              f"{len(accumulated.points):,} total")

    if count == 0:
        print("No point clouds processed!")
        sys.exit(1)

    # 3. Final downsample of accumulated cloud
    if not args.no_downsample:
        before = len(accumulated.points)
        accumulated = accumulated.voxel_down_sample(args.voxel)
        print(f"\nFinal downsample: {before:,} → {len(accumulated.points):,} points")

    # 4. Optional: compute normals (useful for mesh reconstruction later)
    print("Computing normals...")
    accumulated.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
    )

    # 5. Save
    o3d.io.write_point_cloud(output_path, accumulated)
    print(f"\n{'='*50}")
    print(f"  DONE: {count} captures stitched")
    print(f"  Points: {len(accumulated.points):,}")
    print(f"  Saved: {output_path}")
    print(f"{'='*50}")

    # 6. Visualize if requested
    if args.visualize:
        print("\nOpening viewer... (close window to exit)")
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='Stitched Environment')
        vis.add_geometry(accumulated)
        opt = vis.get_render_option()
        opt.point_size = 1.0
        opt.background_color = [0.1, 0.1, 0.1]
        vis.run()
        vis.destroy_window()


if __name__ == '__main__':
    main()
