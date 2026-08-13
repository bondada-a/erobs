#!/usr/bin/env python3
"""Record survey viewpoints from /joint_states while the robot is in freedrive."""

import argparse
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
DEFAULT_OUTPUT = Path(__file__).with_name("survey_poses.yaml")


class PoseTeacher(Node):
    def __init__(self):
        super().__init__("survey_pose_teacher")
        self._latest = None
        self._lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)

    def _on_joints(self, msg):
        with self._lock:
            self._latest = dict(zip(msg.name, msg.position))

    def snapshot_degrees(self):
        with self._lock:
            latest = self._latest
        if latest is None:
            return None
        try:
            return [round(math.degrees(latest[name]), 2) for name in ARM_JOINTS]
        except KeyError as exc:
            self.get_logger().error(f"Joint {exc} missing from /joint_states")
            return None


def load_existing(path):
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")

    poses = {}
    for name, joints in data.items():
        if (
            not isinstance(name, str)
            or not isinstance(joints, list)
            or len(joints) != 6
        ):
            raise ValueError(f"invalid pose {name!r} in {path}")
        try:
            poses[name] = [float(value) for value in joints]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid pose {name!r} in {path}") from exc
    return poses


def format_poses(poses):
    lines = [
        "# Survey viewpoints for point-cloud mapping.",
        "# Joint angles in degrees: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3",
    ]
    for name, joints in poses.items():
        lines.append(f"{name}: [{', '.join(f'{value:.2f}' for value in joints)}]")
    return "\n".join(lines) + "\n"


def next_index(poses, prefix):
    index = 1
    while f"{prefix}_{index}" in poses:
        index += 1
    return index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--prefix", default="survey")
    parser.add_argument("--append", action="store_true")
    args, _ = parser.parse_known_args()

    try:
        poses = load_existing(args.output) if args.append else {}
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    rclpy.init()
    node = PoseTeacher()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        print("Waiting for /joint_states ...")
        deadline = time.monotonic() + 10
        while node.snapshot_degrees() is None:
            if time.monotonic() >= deadline:
                print(
                    "ERROR: no /joint_states received within 10 seconds",
                    file=sys.stderr,
                )
                return 1
            time.sleep(0.05)

        print("Enter=save, u=undo, q=finish")
        while True:
            command = input(f"[{len(poses)} saved] > ").strip().lower()
            if command == "q":
                break
            if command == "u":
                if poses:
                    name, _ = poses.popitem()
                    print(f"Removed {name}")
                else:
                    print("Nothing to undo")
                continue
            if command:
                print("Enter=save, u=undo, q=finish")
                continue

            joints = node.snapshot_degrees()
            if joints is None:
                print("No complete joint state available")
                continue
            name = f"{args.prefix}_{next_index(poses, args.prefix)}"
            poses[name] = joints
            print(f"Saved {name}: {joints}")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not poses:
        print("No poses saved")
        return 0

    try:
        args.output.write_text(format_poses(poses))
    except OSError as exc:
        print(f"ERROR: cannot write {args.output}: {exc}", file=sys.stderr)
        return 1
    print(f"Wrote {len(poses)} poses to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
