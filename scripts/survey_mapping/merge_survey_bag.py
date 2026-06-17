#!/usr/bin/env python3
"""Merge the point clouds in a survey bag into one cloud in base_link.

Step 3 of 3 in the survey-mapping workflow
------------------------------------------
    teach_survey_poses.py   -> record viewpoints into survey_poses.yaml
    run_survey.py           -> drive to each pose + trigger Zivid + record a bag
    merge_survey_bag.py      <- you are here (offline: bag -> merged cloud)

What this does (pure offline, no robot, no ROS graph)
-----------------------------------------------------
Opens the rosbag and replays it in time order, feeding every /tf and /tf_static
message into a tf2 buffer. For each /points/xyzrgba message it:

    1. looks up base_link <- <cloud frame> AT THE CLOUD'S OWN TIMESTAMP
       (the bag carries the full TF history, so this is exact — this is the
        whole reason we recorded TF alongside the clouds)
    2. transforms every point from the camera optical frame into base_link
    3. accumulates them

After all clouds are merged it voxel-downsamples the union and writes a single
PLY. Because the arm base is world-fixed, base_link is the natural common frame;
the wrist-mounted camera's pose in base_link is fully determined by joint FK +
the static hand-eye calibration, both of which are in the bag's TF.

No open3d required
------------------
Transform, voxel-downsample and PLY writing are pure numpy. If open3d IS
installed you can additionally pass --denoise (statistical outlier removal) and
--mesh (Poisson surface reconstruction -> .ply mesh).

Why it usually needs no ICP
---------------------------
The 2026-03-27 hand-eye calibration residuals are <1.53mm / <0.33deg, so a
straight TF-based merge is often clean enough. If you see ghosting/seams,
that's where ICP refinement would come in — out of scope here; this gives you
the TF-merged baseline to judge from.

Usage
-----
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 scripts/survey_mapping/merge_survey_bag.py \
        --bag survey_session --out survey_map.ply --voxel 0.005

    # with open3d extras:
    python3 scripts/survey_mapping/merge_survey_bag.py \
        --bag survey_session --out survey_map.ply --denoise --mesh survey_mesh.ply

Options
-------
    --bag PATH           rosbag2 directory from run_survey.py (required)
    --out PATH           output merged PLY (default: survey_map.ply)
    --target-frame NAME  frame to merge into (default: base_link)
    --voxel METERS       voxel-grid leaf size; 0 disables (default: 0.005 = 5mm)
    --cloud-topic NAME   point cloud topic (default: /points/xyzrgba)
    --max-range METERS   drop points farther than this from the camera (default: 2.0)
    --denoise            statistical outlier removal (needs open3d)
    --mesh PATH          also write a Poisson mesh PLY (needs open3d)
"""

import argparse
import os
import sys

import numpy as np

import rclpy.serialization
import rosbag2_py
from rclpy.duration import Duration
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage
import tf2_ros
from tf_transformations import quaternion_matrix


# ---------------------------------------------------------------------------
# Forward-kinematics fallback for bags missing the moving arm /tf
# ---------------------------------------------------------------------------
# Symptom this fixes: "Could not find a connection between 'base_link' and
# 'zivid_optical_frame' ... Tf has two or more unconnected trees." That happens
# when `ros2 bag record` auto-detected /tf QoS from a TRANSIENT_LOCAL publisher
# (tcp_pose_broadcaster) and silently dropped the VOLATILE robot_state_publisher
# messages — so the bag has /tf_static and /joint_states, but NOT the revolute
# arm transforms that bridge base->wrist.
#
# Given the URDF + /joint_states we compute the FULL target<-source transform
# ourselves, directly with numpy (NO tf2 buffer). The URDF is a complete
# kinematic tree from its root down to zivid_optical_frame (it includes the
# fixed flange->tool0->zivid calibration too), so we never touch the bag's /tf
# at all on this path. Per cloud we pick the nearest /joint_states sample and
# compose ~12 4x4 matrices — ~77 lookups for a whole survey, vs millions of
# buffer inserts the previous tf2-injection approach needed.


def _xyz_rpy_matrix(xyz, rpy) -> np.ndarray:
    """4x4 homogeneous matrix from a URDF <origin xyz=... rpy=...>."""
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    mat[:3, 3] = xyz
    return mat


class UrdfFkSolver:
    """Whole-tree forward kinematics from a URDF, computed directly in numpy.

    Parses every joint into parent/child links. For a target/source frame pair
    we walk each frame's path up to the kinematic root and compose:

        T(target <- source) = world(target)^-1 @ world(source)

    where world(link) is the product of joint transforms from root to that link.
    Fixed joints contribute a constant matrix; revolute/continuous joints
    contribute origin @ R(axis, angle). Angles come from /joint_states.
    """

    def __init__(self, urdf_path: str):
        import xml.etree.ElementTree as ET

        root = ET.parse(urdf_path).getroot()
        # child_link -> joint record. In a URDF tree each link has exactly one
        # parent joint (except the root), so this is a clean upward index.
        self._parent_joint = {}
        for j in root.findall("joint"):
            origin = j.find("origin")
            xyz = (
                [float(v) for v in (origin.get("xyz", "0 0 0")).split()]
                if origin is not None
                else [0.0, 0.0, 0.0]
            )
            rpy = (
                [float(v) for v in (origin.get("rpy", "0 0 0")).split()]
                if origin is not None
                else [0.0, 0.0, 0.0]
            )
            axis_el = j.find("axis")
            axis = (
                [float(v) for v in (axis_el.get("xyz", "1 0 0")).split()]
                if axis_el is not None
                else [1.0, 0.0, 0.0]
            )
            child = j.find("child").get("link")
            self._parent_joint[child] = {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "origin": _xyz_rpy_matrix(xyz, rpy),
                "axis": np.array(axis, dtype=float),
                "moving": j.get("type") in ("revolute", "continuous", "prismatic"),
            }

    def _chain_to_root(self, link: str):
        """Joints from root down to `link` (root-first order)."""
        chain = []
        seen = set()
        while link in self._parent_joint:
            if link in seen:
                raise ValueError(f"cycle in URDF at link {link}")
            seen.add(link)
            j = self._parent_joint[link]
            chain.append(j)
            link = j["parent"]
        chain.reverse()
        return chain

    def _joint_matrix(self, j, angles: dict) -> np.ndarray:
        if not j["moving"]:
            return j["origin"]
        angle = angles.get(j["name"], 0.0)
        if j["type"] == "prismatic":
            slide = np.eye(4)
            slide[:3, 3] = j["axis"] * angle
            return j["origin"] @ slide
        rot = np.eye(4)
        rot[:3, :3] = Rotation.from_rotvec(j["axis"] * angle).as_matrix()
        return j["origin"] @ rot

    def _world(self, link: str, angles: dict) -> np.ndarray:
        """T(root <- link): product of joint transforms from root to link."""
        mat = np.eye(4)
        for j in self._chain_to_root(link):
            mat = mat @ self._joint_matrix(j, angles)
        return mat

    def lookup(self, target: str, source: str, angles: dict) -> np.ndarray:
        """4x4 transform mapping a point in `source` frame into `target` frame."""
        for frame in (target, source):
            if frame not in self._parent_joint and not self._is_root(frame):
                raise ValueError(f"frame '{frame}' not in URDF")
        return np.linalg.inv(self._world(target, angles)) @ self._world(source, angles)

    def _is_root(self, frame: str) -> bool:
        """True if `frame` is some joint's parent but never a child (the root)."""
        parents = {j["parent"] for j in self._parent_joint.values()}
        return frame in parents


# The 6 UR arm revolute joints, in the URDF naming the bag's /joint_states uses.
_ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def open_bag(path: str):
    """Open a rosbag2 dir for sequential reading; return (reader, {topic: type})."""
    storage = rosbag2_py.StorageOptions(uri=path, storage_id="")
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, topic_types


def transform_to_matrix(tf_stamped) -> np.ndarray:
    """TransformStamped -> 4x4 homogeneous matrix (rotation + translation)."""
    t = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation
    mat = quaternion_matrix([q.x, q.y, q.z, q.w])  # 4x4, rotation only
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z
    return mat


def extract_xyz_rgb(cloud: PointCloud2):
    """Return (xyz Nx3 float64, rgb Nx3 uint8 or None) with NaNs dropped.

    Handles Zivid's xyzrgba layout where color is a packed 32-bit field. Color
    extraction is best-effort: if there's no recognizable color field we return
    geometry only.
    """
    field_names = {f.name for f in cloud.fields}
    color_field = (
        "rgba" if "rgba" in field_names else ("rgb" if "rgb" in field_names else None)
    )

    want = ["x", "y", "z"] + ([color_field] if color_field else [])
    # structured=True keeps per-field dtypes (color stays packed for us to unpack)
    pts = point_cloud2.read_points(
        cloud, field_names=want, skip_nans=True, reshape_organized_cloud=False
    )

    xyz = np.column_stack(
        [
            pts["x"].astype(np.float64),
            pts["y"].astype(np.float64),
            pts["z"].astype(np.float64),
        ]
    )

    rgb = None
    if color_field is not None and xyz.shape[0] > 0:
        # Packed color arrives as float32 or uint32 bits; reinterpret to uint32
        # and unpack the 0xAARRGGBB / 0x00RRGGBB byte order Zivid/PCL use.
        packed = np.ascontiguousarray(pts[color_field])
        as_u32 = packed.view(np.uint32).reshape(-1)
        r = ((as_u32 >> 16) & 0xFF).astype(np.uint8)
        g = ((as_u32 >> 8) & 0xFF).astype(np.uint8)
        b = (as_u32 & 0xFF).astype(np.uint8)
        rgb = np.column_stack([r, g, b])

    return xyz, rgb


def voxel_downsample(xyz: np.ndarray, rgb, leaf: float):
    """Average points (and colors) within each leaf-sized voxel cell.

    Pure-numpy voxel grid: quantize to integer cell coords, group by cell, take
    the centroid per cell. Much smaller, evenly-sampled cloud — same idea as the
    open3d voxel_down_sample used in pointcloud_relay.py.
    """
    if leaf <= 0 or xyz.shape[0] == 0:
        return xyz, rgb

    cells = np.floor(xyz / leaf).astype(np.int64)
    # Map each unique cell to a group id; inverse lets us scatter-add per group.
    _, inverse, counts = np.unique(
        cells, axis=0, return_inverse=True, return_counts=True
    )
    inverse = inverse.reshape(-1)
    n_groups = counts.shape[0]

    summed = np.zeros((n_groups, 3), dtype=np.float64)
    np.add.at(summed, inverse, xyz)
    centroids = summed / counts[:, None]

    rgb_out = None
    if rgb is not None:
        csum = np.zeros((n_groups, 3), dtype=np.float64)
        np.add.at(csum, inverse, rgb.astype(np.float64))
        rgb_out = np.clip(csum / counts[:, None], 0, 255).astype(np.uint8)

    return centroids.astype(np.float64), rgb_out


def write_ply(path: str, xyz: np.ndarray, rgb):
    """Write a binary little-endian PLY (xyz float32, optional rgb uchar)."""
    n = xyz.shape[0]
    has_color = rgb is not None
    header = [
        "ply",
        "format binary_little_endian 1.0",
        f"element vertex {n}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if has_color:
        header += ["property uchar red", "property uchar green", "property uchar blue"]
    header += ["end_header", ""]

    with open(path, "wb") as handle:
        handle.write("\n".join(header).encode("ascii"))
        if has_color:
            dtype = np.dtype(
                [
                    ("x", "<f4"),
                    ("y", "<f4"),
                    ("z", "<f4"),
                    ("r", "u1"),
                    ("g", "u1"),
                    ("b", "u1"),
                ]
            )
            arr = np.empty(n, dtype=dtype)
            arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            arr["r"], arr["g"], arr["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        else:
            dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
            arr = np.empty(n, dtype=dtype)
            arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        handle.write(arr.tobytes())


def maybe_denoise_and_mesh(xyz, rgb, args):
    """Optional open3d post-processing. Returns possibly-filtered (xyz, rgb)."""
    if not args.denoise and not args.mesh:
        return xyz, rgb
    try:
        import open3d as o3d
    except ImportError:
        print(
            "WARN: open3d not installed — skipping --denoise/--mesh "
            "(pip install open3d). Wrote the raw TF-merged cloud only."
        )
        return xyz, rgb

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    if rgb is not None:
        pcd.colors = o3d.utility.Vector3dVector(rgb.astype(np.float64) / 255.0)

    if args.denoise:
        before = len(pcd.points)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        print(f"  denoise: {before:,} -> {len(pcd.points):,} points")
        xyz = np.asarray(pcd.points)
        rgb = (
            (np.asarray(pcd.colors) * 255).astype(np.uint8)
            if pcd.has_colors()
            else None
        )

    if args.mesh:
        print("  meshing (Poisson)...")
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30)
        )
        mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=9
        )
        o3d.io.write_triangle_mesh(args.mesh, mesh)
        print(f"  mesh written to {args.mesh}")

    return xyz, rgb


def main():
    parser = argparse.ArgumentParser(
        description="Merge a survey bag's point clouds into one cloud in base_link."
    )
    parser.add_argument(
        "--bag", required=True, help="rosbag2 directory from run_survey.py"
    )
    parser.add_argument("--out", default="survey_map.ply", help="output merged PLY")
    parser.add_argument("--target-frame", default="base_link", help="merge frame")
    parser.add_argument(
        "--voxel", type=float, default=0.005, help="voxel leaf m (0=off)"
    )
    parser.add_argument("--cloud-topic", default="/points/xyzrgba", help="cloud topic")
    parser.add_argument(
        "--max-range",
        type=float,
        default=2.0,
        help="drop points beyond this distance from camera (m)",
    )
    parser.add_argument(
        "--urdf",
        default="",
        help="URDF path: reconstruct missing arm /tf from "
        "/joint_states via FK (use when the merge warns "
        "'unconnected trees'). Dump it live with: ros2 param "
        "get /robot_state_publisher robot_description",
    )
    parser.add_argument(
        "--denoise", action="store_true", help="statistical outlier removal (open3d)"
    )
    parser.add_argument(
        "--mesh", default="", help="also write Poisson mesh PLY (open3d)"
    )
    args, _ = parser.parse_known_args()

    if not os.path.isdir(args.bag):
        print(f"ERROR: bag directory not found: {args.bag}")
        sys.exit(1)

    reader, topic_types = open_bag(args.bag)
    if args.cloud_topic not in topic_types:
        print(
            f"ERROR: cloud topic {args.cloud_topic} not in bag. "
            f"Topics: {sorted(topic_types)}"
        )
        sys.exit(1)

    # Large cache so TF from a minutes-long survey is all retained. Static
    # transforms (hand-eye calib) persist regardless; this covers the dynamic
    # joint TF history. Only used when NOT in FK mode.
    tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=3600))

    # FK fallback: if --urdf is given, compute target<-source directly from the
    # URDF + /joint_states, bypassing the bag's /tf entirely (for bags whose /tf
    # dropped robot_state_publisher — see the UrdfFkSolver comment above). The
    # arm is stopped at each capture, so the most recent /joint_states before a
    # cloud equals the pose at capture — we just track the latest sample.
    fk_solver = None
    fk_used = 0
    latest_angles = None  # dict joint_name -> radians, most recent /joint_states
    if args.urdf:
        if not os.path.isfile(args.urdf):
            print(f"ERROR: --urdf file not found: {args.urdf}")
            sys.exit(1)
        fk_solver = UrdfFkSolver(args.urdf)
        print(
            f"FK fallback ON: computing {args.target_frame}<-cloud directly "
            f"from /joint_states + {args.urdf} (bypassing bag /tf)"
        )

    merged_xyz = []
    merged_rgb = []
    any_color = True
    n_clouds = 0

    print(f"Reading {args.bag} ...")
    while reader.has_next():
        topic, data, _stamp = reader.read_next()

        # In FK mode we ignore the bag's /tf completely (it's the broken part);
        # otherwise feed it into the buffer for time-aware lookups.
        if topic in ("/tf", "/tf_static") and fk_solver is None:
            tf_msg = rclpy.serialization.deserialize_message(data, TFMessage)
            is_static = topic == "/tf_static"
            for transform in tf_msg.transforms:
                if is_static:
                    tf_buffer.set_transform_static(transform, "bag")
                else:
                    tf_buffer.set_transform(transform, "bag")
            continue

        if topic == "/joint_states" and fk_solver is not None:
            js = rclpy.serialization.deserialize_message(data, JointState)
            latest_angles = dict(zip(js.name, js.position))
            continue

        if topic == args.cloud_topic:
            cloud = rclpy.serialization.deserialize_message(data, PointCloud2)
            src_frame = cloud.header.frame_id
            stamp = Time.from_msg(cloud.header.stamp)

            if fk_solver is not None:
                if latest_angles is None:
                    print("  WARN: cloud before any /joint_states; skipping")
                    continue
                try:
                    mat = fk_solver.lookup(args.target_frame, src_frame, latest_angles)
                except ValueError as exc:
                    print(f"  WARN: FK lookup failed ({exc}); skipping a cloud")
                    continue
                fk_used += 1
            else:
                try:
                    tf = tf_buffer.lookup_transform(
                        args.target_frame,
                        src_frame,
                        stamp,
                        timeout=Duration(seconds=0.0),
                    )
                except tf2_ros.TransformException as exc:
                    print(
                        f"  WARN: no TF {args.target_frame}<-{src_frame} "
                        f"at {stamp.nanoseconds * 1e-9:.3f}s ({exc}); skipping a cloud"
                    )
                    continue
                mat = transform_to_matrix(tf)

            xyz_cam, rgb = extract_xyz_rgb(cloud)
            if xyz_cam.shape[0] == 0:
                continue

            # Range gate in camera frame (distance from optical origin) before
            # transforming — kills far-wall/background noise cheaply.
            if args.max_range > 0:
                dist = np.linalg.norm(xyz_cam, axis=1)
                keep = dist <= args.max_range
                xyz_cam = xyz_cam[keep]
                if rgb is not None:
                    rgb = rgb[keep]
            if xyz_cam.shape[0] == 0:
                continue

            # Apply 4x4 (computed above, FK or tf2): p_base = R * p_cam + t
            homog = np.hstack([xyz_cam, np.ones((xyz_cam.shape[0], 1))])
            xyz_base = (homog @ mat.T)[:, :3]
            raw_pts = xyz_base.shape[0]

            # Downsample THIS cloud immediately (before accumulating), so peak
            # memory is ~one raw cloud + the growing downsampled set — not all
            # raw clouds at once. Critical at full-res: 76 x 5M raw points would
            # be ~10GB held + ~20GB at the final vstack, which OOM-kills. A final
            # voxel pass after the loop still merges cross-cloud overlap.
            if args.voxel > 0:
                xyz_base, rgb = voxel_downsample(xyz_base, rgb, args.voxel)

            merged_xyz.append(xyz_base)
            if rgb is None:
                any_color = False
            else:
                merged_rgb.append(rgb)
            n_clouds += 1
            kept = xyz_base.shape[0]
            if args.voxel > 0:
                print(
                    f"  merged cloud #{n_clouds} from {src_frame}: "
                    f"{raw_pts:,} -> {kept:,} pts (voxel)"
                )
            else:
                print(f"  merged cloud #{n_clouds} from {src_frame}: {kept:,} pts")

    del reader

    if fk_solver is not None:
        print(f"FK: computed {fk_used} cloud transforms directly from /joint_states")

    if not merged_xyz:
        print("ERROR: no clouds merged. Did run_survey.py record any captures?")
        if fk_solver is None:
            print(
                "HINT: if the warnings say 'unconnected trees', the bag's /tf "
                "dropped the arm transforms. Re-run with "
                "--urdf /path/to/robot.urdf to reconstruct them from "
                "/joint_states."
            )
        sys.exit(1)

    xyz = np.vstack(merged_xyz)
    rgb = np.vstack(merged_rgb) if (any_color and merged_rgb) else None
    print(
        f"Stacked {n_clouds} cloud(s): {xyz.shape[0]:,} points"
        f"{' (per-cloud voxel applied)' if args.voxel > 0 else ''}"
        f"{' (with color)' if rgb is not None else ''}"
    )

    if args.voxel > 0:
        # Second pass: merge overlap BETWEEN clouds (each was downsampled alone).
        xyz, rgb = voxel_downsample(xyz, rgb, args.voxel)
        print(f"Final voxel merge ({args.voxel * 1000:.1f}mm): {xyz.shape[0]:,} points")

    args.mesh = args.mesh or None
    xyz, rgb = maybe_denoise_and_mesh(xyz, rgb, args)

    write_ply(args.out, xyz, rgb)
    print(f"\nWrote merged cloud: {args.out} ({xyz.shape[0]:,} points)")
    print("View with: ros2 run pcl_ros ... , CloudCompare, MeshLab, or open3d.")


if __name__ == "__main__":
    main()
