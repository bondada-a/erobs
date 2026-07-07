#!/usr/bin/env python3
"""Multi-run scatter of RRTstar vs LBTRRT joint-space paths (+ PTP straight line).

Plans each sampler N_RUNS times against the live move_group and overlays every
run (faint) plus the mean (bold) per joint, so the run-to-run SCATTER is
visible. RRTstar should spread wider than LBTRRT (which converges tighter). Pilz
PTP is deterministic — one bold green line.

REQUIRES the beambot stack up (move_group + obstacle scene). Run:
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 src/beambot/scripts/compare_planners_scatter.py
Output: /tmp/planner_scatter.png

Reuses the request-building / capture logic from compare_planners.py.
"""
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from moveit_msgs.srv import GetMotionPlan
from rclpy.node import Node

from compare_planners import JOINTS, PLANNERS, analytic_ptp, joint_path_length, plan

N_RUNS = 12      # plans per sampler
PLAN_TIME = 8.0  # per-plan budget — 8s keeps the failure rate low (4s failed ~half)
GRID = 50        # resample every path to this many points so runs align for overlay + mean


def resample(traj, m=GRID):
    """Interpolate a variable-length path onto a fixed m-point normalized grid."""
    src = np.linspace(0, 1, len(traj))
    dst = np.linspace(0, 1, m)
    return np.stack([np.interp(dst, src, traj[:, j]) for j in range(traj.shape[1])], axis=1)


def main():
    rclpy.init()
    node = Node("compare_planners_scatter")
    client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("/plan_kinematic_path unavailable — start the stack.")
        rclpy.shutdown()
        return

    runs = {label: ([], color) for label, _, _, color in PLANNERS}
    lengths = {label: [] for label, _, _, _ in PLANNERS}
    for label, pipeline, pid, color in PLANNERS:
        for i in range(N_RUNS):
            traj = plan(node, client, pipeline, pid, plan_time=PLAN_TIME)
            if traj is None:
                node.get_logger().warn(f"{label} run {i}: failed, skipping")
                continue
            runs[label][0].append(resample(traj))
            lengths[label].append(math.degrees(joint_path_length(traj)))

    ptp = resample(analytic_ptp())
    ptp_len = math.degrees(joint_path_length(analytic_ptp()))

    # Length stats — the scatter, quantified.
    print(f"Pilz PTP (straight): {ptp_len:.1f} deg (deterministic)")
    for label in lengths:
        vals = lengths[label]
        if vals:
            print(f"{label}: n={len(vals)}  min {min(vals):.1f}  mean {np.mean(vals):.1f}  "
                  f"max {max(vals):.1f}  spread {max(vals) - min(vals):.1f}  std {np.std(vals):.1f}")

    x = np.linspace(0, 1, GRID)
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    for j, ax in enumerate(axes.flat):
        ax.plot(x, np.degrees(ptp[:, j]), color="tab:green", lw=2, label="Pilz PTP", zorder=3)
        for label, (rlist, color) in runs.items():
            if not rlist:
                continue
            arr = np.array(rlist)  # (nRuns, GRID, 6)
            for r in arr:  # faint individual runs = the scatter
                ax.plot(x, np.degrees(r[:, j]), color=color, alpha=0.22, lw=0.8)
            ax.plot(x, np.degrees(arr[:, :, j].mean(0)), color=color, lw=2,
                    label=f"{label} (mean of {len(arr)})", zorder=2)
        ax.set_title(JOINTS[j])
        ax.set_xlabel("path progress")
        ax.set_ylabel("angle (deg)")
        ax.grid(True, alpha=0.3)
    axes.flat[0].legend(loc="best", fontsize=7)
    fig.suptitle(
        f"Scatter over {N_RUNS} runs @ {PLAN_TIME}s each — sample_scan_1 -> safe_tool_exchange"
    )
    fig.tight_layout()
    out = "/tmp/planner_scatter.png"
    fig.savefig(out, dpi=110)
    print(f"saved {out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
