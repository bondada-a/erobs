#!/usr/bin/env python3
"""Interactive hand-eye calibration pose saver.

Free-drive the robot, hit Enter to append the current joint pose to
handeye_cal_poses.yaml. Reads /joint_states directly (rclpy) — no rosbridge,
no beambot stack needed, just the UR driver up.

Usage:
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash              # for sensor_msgs (stock, always there)
    python3 scripts/save_cal_poses.py      # writes to src/cms/handeye_cal_poses.yaml
    python3 scripts/save_cal_poses.py --selftest   # verify joint reordering, no ROS

Commands at the prompt:
    <Enter> save current pose | u undo last | q quit
"""
import math
import os
import sys
import threading

# Canonical UR order — /joint_states publishes joints in arbitrary order AND
# includes the epick "gripper" joint, so we MUST map by name, never by index.
ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


def reorder_deg(name_to_pos_rad):
    """{joint_name: radians} -> [deg, ...] in canonical arm order (matches poses.yaml)."""
    missing = [j for j in ARM_JOINTS if j not in name_to_pos_rad]
    if missing:
        raise KeyError(f"missing joints in /joint_states: {missing}")
    return [round(math.degrees(name_to_pos_rad[j]), 2) for j in ARM_JOINTS]


def _selftest():
    # Deliberately scrambled order + extra gripper joint, like the real topic.
    d = {
        "elbow_joint": 0.0, "gripper": 0.0,
        "shoulder_lift_joint": -math.pi / 2, "shoulder_pan_joint": math.pi / 4,
        "wrist_1_joint": -math.pi / 2, "wrist_2_joint": -3 * math.pi / 2,
        "wrist_3_joint": math.radians(-162.5),
    }
    assert reorder_deg(d) == [45.0, -90.0, 0.0, -90.0, -270.0, -162.5], reorder_deg(d)
    try:
        reorder_deg({"shoulder_pan_joint": 0.0})  # missing joints must raise
    except KeyError:
        pass
    else:
        raise AssertionError("expected KeyError on missing joints")
    print("selftest ok")


def count_poses(path):
    return sum(1 for ln in open(path).read().splitlines() if ln.strip().startswith("- ["))


def append_pose(path, joints_deg, idx):
    txt = open(path).read()
    if "poses: []" in txt:  # first save: turn empty list into a block list
        open(path, "w").write(txt.replace("poses: []", "poses:"))
    with open(path, "a") as f:
        f.write(f"  - [{', '.join(f'{v:.2f}' for v in joints_deg)}]  # cal_{idx:02d}\n")


def undo_last(path):
    lines = open(path).read().splitlines(keepends=True)
    for i in range(len(lines) - 1, -1, -1):
        if lines[i].strip().startswith("- ["):
            removed = lines.pop(i)
            open(path, "w").writelines(lines)
            return removed.strip()
    return None


def main():
    if "--selftest" in sys.argv:
        _selftest()
        return

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_out = os.path.join(repo_root, "src", "cms", "handeye_cal_poses.yaml")
    out = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else default_out
    if not os.path.exists(out):
        sys.exit(f"pose file not found: {out}")

    import rclpy
    from sensor_msgs.msg import JointState

    latest = {"msg": None}
    rclpy.init()
    node = rclpy.create_node("save_cal_pose")
    node.create_subscription(JointState, "/joint_states", lambda m: latest.__setitem__("msg", m), 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    n = count_poses(out)
    print(f"Saving to: {out}")
    print(f"(existing poses: {n})")
    print("<Enter> save | u undo last | q quit")
    try:
        while True:
            cmd = input(f"[{n} saved] > ").strip().lower()
            if cmd == "q":
                break
            if cmd == "u":
                r = undo_last(out)
                if r:
                    n -= 1
                    print(f"  undid {r}  (now {n})")
                else:
                    print("  nothing to undo")
                continue
            if cmd != "":
                print("  (Enter=save, u=undo, q=quit)")
                continue
            m = latest["msg"]
            if m is None:
                print("  no /joint_states yet — is the UR driver up?")
                continue
            try:
                deg = reorder_deg(dict(zip(m.name, m.position)))
            except KeyError as e:
                print(f"  {e}")
                continue
            append_pose(out, deg, n)
            print(f"  saved cal_{n:02d}: {deg}")
            n += 1
    finally:
        rclpy.shutdown()
        print(f"Done. {n} poses in {out}")


if __name__ == "__main__":
    main()
