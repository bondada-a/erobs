#!/usr/bin/env python3
"""Plot joint-space trajectories from Pilz PTP, RRTstar, LBTRRT for one move.

Captures each planner's path from the LIVE move_group via /plan_kinematic_path
(no execution, planning only), then plots each joint's angle vs normalized path
progress so you can see how RRT*/LBTRRT detour around obstacles while Pilz PTP
takes the straight joint-space line.

REQUIRES the beambot stack running (so move_group is up AND the obstacle scene
from beamline_scene.yaml is loaded — without the scene, RRT* plans in empty
space and every path is a straight line). Run:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 src/beambot/scripts/compare_planners.py

Output: /tmp/planner_comparison.png
"""
import math

import matplotlib
matplotlib.use("Agg")  # headless — save PNG, no display server needed
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from moveit_msgs.srv import GetMotionPlan
from rclpy.node import Node

# UR joint order — matches poses.yaml column order.
JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
START_DEG = [13.38, -112.45, -65.22, -90.98, -267.33, -166.94]   # sample_scan_1
GOAL_DEG = [152.33, -110.63, -100.17, -59.12, -270.43, -207.7]   # safe_tool_exchange

# Live sampling planners (captured only when the stack is up). PTP is excluded —
# it collides here, so its path is the offline straight-line, not a live plan.
# (label, pipeline_id, planner_id=yaml config KEY, color).
PLANNERS = [
    ("RRTConnect", "ompl", "RRTConnectkConfigDefault", "tab:red"),
    ("RRTstar", "ompl", "RRTstar", "tab:blue"),
    ("LBTRRT", "ompl", "LBTRRT", "tab:orange"),
    # STOMP has one planner, no yaml config key -> empty planner_id. Needs
    # "stomp" in the move_group launch pipelines list (robot_bringup.launch.py).
    ("STOMP", "stomp", "", "tab:purple"),
]
PLAN_TIME = 10.0  # RRT*/LBTRRT are anytime — give them time to converge.


def deg2rad(v):
    return [math.radians(x) for x in v]


def plan(node, client, pipeline_id, planner_id, plan_time=PLAN_TIME):
    """Return an (N, 6) array of joint positions (rad) in JOINTS order, or None."""
    req = GetMotionPlan.Request()
    m = MotionPlanRequest()
    m.group_name = "ur_arm"
    m.pipeline_id = pipeline_id
    m.planner_id = planner_id
    m.allowed_planning_time = plan_time
    m.num_planning_attempts = 1
    m.start_state.joint_state.name = JOINTS
    m.start_state.joint_state.position = deg2rad(START_DEG)
    c = Constraints()
    for name, pos in zip(JOINTS, deg2rad(GOAL_DEG)):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = pos
        jc.tolerance_above = 1e-3
        jc.tolerance_below = 1e-3
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    m.goal_constraints.append(c)
    req.motion_plan_request = m

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=plan_time + 15.0)
    if not future.done():
        node.get_logger().error(f"{planner_id}: service call timed out")
        return None
    resp = future.result()
    if resp.motion_plan_response.error_code.val != 1:  # 1 == SUCCESS
        node.get_logger().warn(
            f"{planner_id}: planning failed (error {resp.motion_plan_response.error_code.val})"
        )
        return None
    jt = resp.motion_plan_response.trajectory.joint_trajectory
    # Reorder response columns into canonical JOINTS order.
    idx = [jt.joint_names.index(j) for j in JOINTS]
    return np.array([[pt.positions[i] for i in idx] for pt in jt.points])


def analytic_ptp():
    """PTP joint-space path = straight line between start and goal (may collide)."""
    s = np.radians(START_DEG)
    g = np.radians(GOAL_DEG)
    t = np.linspace(0, 1, 50)[:, None]
    return s + t * (g - s)


def joint_path_length(traj):
    """Total joint-space arc length (rad, summed over joints)."""
    return float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1)))


def main():
    results = {}

    # Pilz PTP: always the analytic straight line (it collides, so we don't plan
    # it live) — the baseline RRT*/LBTRRT detour around. Offline, no stack.
    ptp = analytic_ptp()
    results["Pilz PTP (straight-line)"] = (ptp, "tab:green")
    print(f"Pilz PTP (analytic): joint-path length "
          f"{math.degrees(joint_path_length(ptp)):.1f} deg")

    # RRTstar / LBTRRT: live capture from move_group, only if the stack is up.
    rclpy.init()
    node = Node("compare_planners")
    client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    if client.wait_for_service(timeout_sec=5.0):
        for label, pipeline, pid, color in PLANNERS:
            traj = plan(node, client, pipeline, pid)
            if traj is not None:
                results[label] = (traj, color)
                print(f"{label}: {len(traj)} pts, joint-path length "
                      f"{math.degrees(joint_path_length(traj)):.1f} deg")
    else:
        node.get_logger().warn(
            "move_group /plan_kinematic_path unavailable (stack down). Plotting "
            "Pilz PTP analytic only; re-run with the stack up for RRT*/LBTRRT."
        )

    # 6 subplots: each joint's angle (deg) vs normalized path progress (0..1).
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    for j, ax in enumerate(axes.flat):
        for label, (traj, color) in results.items():
            x = np.linspace(0, 1, len(traj))
            ax.plot(x, np.degrees(traj[:, j]), color=color, label=label, marker=".", ms=3)
        ax.set_title(JOINTS[j])
        ax.set_xlabel("path progress")
        ax.set_ylabel("angle (deg)")
        ax.grid(True, alpha=0.3)
    axes.flat[0].legend(loc="best", fontsize=8)
    fig.suptitle("sample_scan_1 -> safe_tool_exchange : joint-space trajectories")
    fig.tight_layout()
    out = "/tmp/planner_comparison.png"
    fig.savefig(out, dpi=110)
    print(f"saved {out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
