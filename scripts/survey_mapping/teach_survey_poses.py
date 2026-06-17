#!/usr/bin/env python3
"""Teach survey poses by hand: move the arm, press Enter, it saves the pose.

Step 1 of 3 in the survey-mapping workflow
------------------------------------------
    teach_survey_poses.py   <- you are here (record viewpoints)
    run_survey.py           -> drive to each pose + trigger Zivid + record a bag
    merge_survey_bag.py      -> offline: bag -> merged point cloud (base_link)

What this does
--------------
Subscribes to /joint_states and runs a tiny interactive loop:

    move the arm to a viewpoint (by hand, in freedrive)  ->  press Enter
    move it somewhere else                               ->  press Enter
    ...                                                  ->  type 'q' to finish

Each Enter snapshots the current 6 arm-joint angles, converts them to DEGREES
(the convention used by src/cms/poses.yaml and the orchestrator), and appends
them under an auto-incrementing name (survey_1, survey_2, ...). On 'q' it
writes them all to survey_poses.yaml.

The robot is NEVER commanded to move by this script — YOU move it. Put the
UR into freedrive / teach mode (teach pendant button, or however the cell is
set up) so you can physically pose the wrist.

Output schema (matches the pose registry exactly so it's reusable everywhere):

    survey_1: [j1_deg, j2_deg, j3_deg, j4_deg, j5_deg, j6_deg]
    survey_2: [ ... ]

Usage
-----
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash   # for joint-state message types (std msgs only here)
    python3 scripts/survey_mapping/teach_survey_poses.py

    # custom output path / pose name prefix:
    python3 scripts/survey_mapping/teach_survey_poses.py \
        --output /tmp/my_survey.yaml --prefix view

    # append to an existing file instead of overwriting:
    python3 scripts/survey_mapping/teach_survey_poses.py --append
"""

import argparse
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# UR arm joints, in the canonical order the pose registry stores them.
ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# rad -> deg without importing math (kept dependency-light)
_RAD2DEG = 57.29577951308232

# Default output sits next to this script so the whole workflow is self-contained.
_DEFAULT_OUTPUT = os.path.join(os.path.dirname(__file__), "survey_poses.yaml")


class PoseTeacher(Node):
    """Holds the latest /joint_states so the main thread can snapshot on Enter."""

    def __init__(self):
        super().__init__("survey_pose_teacher")
        self._latest = None  # dict: joint_name -> position (radians)
        self._lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)

    def _on_joints(self, msg: JointState):
        # /joint_states may include non-arm joints (gripper, etc.) and arrive in
        # any order, so index by name rather than assuming a fixed layout.
        with self._lock:
            self._latest = dict(zip(msg.name, msg.position))

    def snapshot_degrees(self):
        """Return the current 6 arm joints in DEGREES, or None if not ready.

        Returns None when no joint state has arrived yet, or when the message
        is missing one of the arm joints (shouldn't happen on a healthy bringup,
        but we fail loud rather than write a garbage pose).
        """
        with self._lock:
            latest = self._latest
        if latest is None:
            return None
        try:
            return [latest[name] * _RAD2DEG for name in ARM_JOINTS]
        except KeyError as missing:
            self.get_logger().error(f"Joint {missing} not in /joint_states")
            return None


def _format_yaml(poses: dict) -> str:
    """Serialize {name: [floats]} to inline-list YAML matching poses.yaml style.

    Hand-rolled (no yaml dep) so the file looks exactly like src/cms/poses.yaml:
    one pose per line, 2-decimal degrees, inline bracket list.
    """
    lines = []
    for name, joints in poses.items():
        joints_str = ", ".join(f"{v:.2f}" for v in joints)
        lines.append(f"{name}: [{joints_str}]")
    return "\n".join(lines) + "\n"


def _load_existing(path: str) -> dict:
    """Best-effort parse of an existing survey YAML so --append can extend it.

    Only understands the inline-list format this script writes; anything it
    can't parse is skipped with a warning rather than aborting.
    """
    poses = {}
    if not os.path.isfile(path):
        return poses
    with open(path) as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            name, _, rest = line.partition(":")
            rest = rest.strip().strip("[]")
            try:
                poses[name.strip()] = [float(v) for v in rest.split(",")]
            except ValueError:
                print(f"  (skipping unparseable line: {line!r})")
    return poses


def _next_index(poses: dict, prefix: str) -> int:
    """First free N such that '{prefix}_{N}' isn't already taken."""
    n = 1
    while f"{prefix}_{n}" in poses:
        n += 1
    return n


def main():
    parser = argparse.ArgumentParser(
        description="Teach survey poses by hand (move arm, press Enter, it saves)."
    )
    parser.add_argument(
        "--output",
        default=_DEFAULT_OUTPUT,
        help=f"Output YAML path (default: {_DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--prefix",
        default="survey",
        help="Pose name prefix; poses are <prefix>_1, <prefix>_2, ... (default: survey)",
    )
    parser.add_argument(
        "--append",
        action="store_true",
        help="Append to --output if it exists instead of overwriting.",
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = PoseTeacher()

    # Spin in a background thread so the foreground can block on input().
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    poses = _load_existing(args.output) if args.append else {}
    if poses:
        print(f"Loaded {len(poses)} existing pose(s) from {args.output}")

    # Wait for the first joint state so the very first Enter can't fail silently.
    print("Waiting for /joint_states ...")
    ready_deadline = node.get_clock().now().nanoseconds + int(10e9)
    while node.snapshot_degrees() is None:
        if node.get_clock().now().nanoseconds > ready_deadline:
            print("ERROR: No /joint_states received in 10s. Is the robot driver up?")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    print(
        "\n"
        "================ TEACH SURVEY POSES ================\n"
        "  Put the arm in FREEDRIVE and move it to a viewpoint.\n"
        "  [Enter]      save current pose\n"
        "  u [Enter]    undo last saved pose\n"
        "  q [Enter]    finish and write the YAML\n"
        "====================================================\n"
    )

    try:
        while True:
            cmd = input(f"[{len(poses)} saved] pose> ").strip().lower()

            if cmd == "q":
                break

            if cmd == "u":
                if poses:
                    removed = next(reversed(poses))
                    del poses[removed]
                    print(f"  removed {removed}")
                else:
                    print("  nothing to undo")
                continue

            if cmd != "":
                print("  (press Enter to save, 'u' to undo, 'q' to finish)")
                continue

            joints = node.snapshot_degrees()
            if joints is None:
                print("  WARN: no fresh joint state — not saved, try again")
                continue

            name = f"{args.prefix}_{_next_index(poses, args.prefix)}"
            poses[name] = joints
            joints_str = ", ".join(f"{v:.2f}" for v in joints)
            print(f"  saved {name}: [{joints_str}]")

    except (KeyboardInterrupt, EOFError):
        print()  # tidy the prompt line on Ctrl-C / Ctrl-D

    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not poses:
        print("No poses saved — nothing written.")
        return

    with open(args.output, "w") as handle:
        handle.write(
            "# Survey viewpoints for point-cloud mapping.\n"
            "# Joint angles in DEGREES, order: "
            "shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3\n"
            "# Generated by scripts/survey_mapping/teach_survey_poses.py\n"
        )
        handle.write(_format_yaml(poses))

    print(f"\nWrote {len(poses)} pose(s) to {args.output}")
    print("Next: run_survey.py --poses %s" % args.output)


if __name__ == "__main__":
    main()
