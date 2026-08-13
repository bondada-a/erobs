#!/usr/bin/env python3
"""Merge survey point clouds into one voxel-downsampled PLY."""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy.serialization
import rosbag2_py
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_matrix, quaternion_matrix, rotation_matrix

CLOUD_TOPIC = "/points/xyzrgba"


def xyz_rpy_matrix(xyz, rpy):
    matrix = euler_matrix(*rpy, axes="sxyz")
    matrix[:3, 3] = xyz
    return matrix


class UrdfFkSolver:
    """Compute transforms from a URDF and joint positions for legacy bags."""

    def __init__(self, path):
        self.parent_joint = {}
        for element in ET.parse(path).getroot().findall("joint"):
            origin = element.find("origin")
            axis = element.find("axis")
            xyz = (
                [float(value) for value in origin.get("xyz", "0 0 0").split()]
                if origin is not None
                else [0, 0, 0]
            )
            rpy = (
                [float(value) for value in origin.get("rpy", "0 0 0").split()]
                if origin is not None
                else [0, 0, 0]
            )
            child = element.find("child").get("link")
            self.parent_joint[child] = {
                "name": element.get("name"),
                "type": element.get("type"),
                "parent": element.find("parent").get("link"),
                "origin": xyz_rpy_matrix(xyz, rpy),
                "axis": np.asarray(
                    [float(value) for value in axis.get("xyz", "1 0 0").split()]
                    if axis is not None
                    else [1, 0, 0]
                ),
            }

    def chain_to_root(self, link):
        chain = []
        seen = set()
        while link in self.parent_joint:
            if link in seen:
                raise ValueError(f"cycle in URDF at {link}")
            seen.add(link)
            joint = self.parent_joint[link]
            chain.append(joint)
            link = joint["parent"]
        return reversed(chain)

    def joint_matrix(self, joint, angles):
        if joint["type"] not in {"revolute", "continuous", "prismatic"}:
            return joint["origin"]
        if joint["name"] not in angles:
            raise ValueError(f"joint {joint['name']!r} missing from /joint_states")

        value = angles[joint["name"]]
        if joint["type"] == "prismatic":
            motion = np.eye(4)
            motion[:3, 3] = joint["axis"] * value
        else:
            motion = rotation_matrix(value, joint["axis"])
        return joint["origin"] @ motion

    def world_matrix(self, link, angles):
        matrix = np.eye(4)
        for joint in self.chain_to_root(link):
            matrix = matrix @ self.joint_matrix(joint, angles)
        return matrix

    def lookup(self, target, source, angles):
        roots = {joint["parent"] for joint in self.parent_joint.values()}
        for frame in (target, source):
            if frame not in self.parent_joint and frame not in roots:
                raise ValueError(f"frame {frame!r} not found in URDF")
        return np.linalg.inv(self.world_matrix(target, angles)) @ self.world_matrix(
            source, angles
        )


def open_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    return reader, topic_types


def transform_matrix(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def extract_points(cloud):
    fields = {field.name for field in cloud.fields}
    color_field = "rgba" if "rgba" in fields else "rgb" if "rgb" in fields else None
    names = ["x", "y", "z"] + ([color_field] if color_field else [])
    points = point_cloud2.read_points(
        cloud, field_names=names, skip_nans=True, reshape_organized_cloud=False
    )
    xyz = np.column_stack(
        [
            points["x"].astype(np.float64),
            points["y"].astype(np.float64),
            points["z"].astype(np.float64),
        ]
    )
    if color_field is None or not len(xyz):
        return xyz, None

    packed = np.ascontiguousarray(points[color_field]).view(np.uint32).reshape(-1)
    rgb = np.column_stack(
        [
            ((packed >> 16) & 0xFF).astype(np.uint8),
            ((packed >> 8) & 0xFF).astype(np.uint8),
            (packed & 0xFF).astype(np.uint8),
        ]
    )
    return xyz, rgb


def voxel_downsample(xyz, rgb, leaf):
    if leaf <= 0 or not len(xyz):
        return xyz, rgb

    cells = np.floor(xyz / leaf).astype(np.int64)
    _, groups, counts = np.unique(
        cells, axis=0, return_inverse=True, return_counts=True
    )
    xyz_sum = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(xyz_sum, groups, xyz)
    xyz = xyz_sum / counts[:, None]

    if rgb is not None:
        rgb_sum = np.zeros((len(counts), 3), dtype=np.float64)
        np.add.at(rgb_sum, groups, rgb)
        rgb = np.clip(rgb_sum / counts[:, None], 0, 255).astype(np.uint8)
    return xyz, rgb


def write_ply(path, xyz, rgb):
    fields = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
    header = [
        "ply",
        "format binary_little_endian 1.0",
        f"element vertex {len(xyz)}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if rgb is not None:
        fields += [("r", "u1"), ("g", "u1"), ("b", "u1")]
        header += ["property uchar red", "property uchar green", "property uchar blue"]
    header += ["end_header", ""]

    points = np.empty(len(xyz), dtype=np.dtype(fields))
    points["x"], points["y"], points["z"] = xyz.T
    if rgb is not None:
        points["r"], points["g"], points["b"] = rgb.T

    with open(path, "wb") as output:
        output.write("\n".join(header).encode("ascii"))
        output.write(points.tobytes())


def self_check():
    xyz = np.array([[0.0, 0, 0], [0.4, 0, 0], [1.1, 0, 0]])
    rgb = np.array([[0, 0, 0], [100, 100, 100], [200, 200, 200]], dtype=np.uint8)
    sampled_xyz, sampled_rgb = voxel_downsample(xyz, rgb, 1.0)
    assert np.allclose(sampled_xyz, [[0.2, 0, 0], [1.1, 0, 0]])
    assert np.array_equal(sampled_rgb, [[50, 50, 50], [200, 200, 200]])
    assert np.allclose(xyz_rpy_matrix([1, 2, 3], [0, 0, 0])[:3, 3], [1, 2, 3])
    print("self-check passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag")
    parser.add_argument("--out", default="survey_map.ply")
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument("--voxel", type=float, default=0.005)
    parser.add_argument("--max-range", type=float, default=2.0)
    parser.add_argument(
        "--urdf",
        default="",
        help="reconstruct transforms from /joint_states when legacy bags lack arm TF",
    )
    parser.add_argument("--self-check", action="store_true")
    args, _ = parser.parse_known_args()

    if args.self_check:
        self_check()
        return 0
    if not args.bag:
        parser.error("--bag is required")
    if args.voxel < 0 or args.max_range < 0:
        parser.error("--voxel and --max-range cannot be negative")

    bag_path = Path(args.bag)
    if not bag_path.is_dir():
        print(f"ERROR: bag directory not found: {bag_path}", file=sys.stderr)
        return 1

    reader, topic_types = open_bag(bag_path)
    if CLOUD_TOPIC not in topic_types:
        print(f"ERROR: {CLOUD_TOPIC} is not present in the bag", file=sys.stderr)
        return 1

    fk_solver = None
    if args.urdf:
        urdf_path = Path(args.urdf)
        if not urdf_path.is_file():
            print(f"ERROR: URDF not found: {urdf_path}", file=sys.stderr)
            return 1
        try:
            fk_solver = UrdfFkSolver(urdf_path)
        except (OSError, ET.ParseError, ValueError) as exc:
            print(f"ERROR: cannot load URDF: {exc}", file=sys.stderr)
            return 1

    tf_buffer = None if fk_solver else tf2_ros.Buffer(cache_time=Duration(seconds=3600))
    latest_angles = None
    clouds = []
    colors = []
    all_have_color = True

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic in {"/tf", "/tf_static"} and fk_solver is None:
            message = rclpy.serialization.deserialize_message(data, TFMessage)
            for transform in message.transforms:
                if topic == "/tf_static":
                    tf_buffer.set_transform_static(transform, "bag")
                else:
                    tf_buffer.set_transform(transform, "bag")
            continue

        if topic == "/joint_states" and fk_solver is not None:
            message = rclpy.serialization.deserialize_message(data, JointState)
            latest_angles = dict(zip(message.name, message.position))
            continue

        if topic != CLOUD_TOPIC:
            continue

        cloud = rclpy.serialization.deserialize_message(data, PointCloud2)
        if fk_solver is not None:
            if latest_angles is None:
                print("WARN: skipping cloud recorded before any joint state")
                continue
            try:
                matrix = fk_solver.lookup(
                    args.target_frame, cloud.header.frame_id, latest_angles
                )
            except ValueError as exc:
                print(f"WARN: skipping cloud: {exc}")
                continue
        else:
            try:
                transform = tf_buffer.lookup_transform(
                    args.target_frame,
                    cloud.header.frame_id,
                    Time.from_msg(cloud.header.stamp),
                    timeout=Duration(seconds=0),
                )
            except tf2_ros.TransformException as exc:
                print(f"WARN: skipping cloud without transform: {exc}")
                continue
            matrix = transform_matrix(transform)

        xyz, rgb = extract_points(cloud)
        if args.max_range > 0 and len(xyz):
            keep = np.einsum("ij,ij->i", xyz, xyz) <= args.max_range**2
            xyz = xyz[keep]
            if rgb is not None:
                rgb = rgb[keep]
        if not len(xyz):
            continue

        xyz = xyz @ matrix[:3, :3].T + matrix[:3, 3]
        raw_count = len(xyz)
        xyz, rgb = voxel_downsample(xyz, rgb, args.voxel)
        clouds.append(xyz)
        if rgb is None:
            all_have_color = False
            colors.clear()
        elif all_have_color:
            colors.append(rgb)
        print(f"Cloud {len(clouds)}: {raw_count:,} -> {len(xyz):,} points")

    if not clouds:
        print("ERROR: no clouds could be merged", file=sys.stderr)
        return 1

    xyz = np.vstack(clouds)
    rgb = np.vstack(colors) if all_have_color else None
    xyz, rgb = voxel_downsample(xyz, rgb, args.voxel)
    write_ply(args.out, xyz, rgb)
    print(f"Wrote {args.out}: {len(xyz):,} points from {len(clouds)} clouds")
    return 0


if __name__ == "__main__":
    sys.exit(main())
