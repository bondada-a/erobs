#!/usr/bin/env python3
"""Drive an octomap capture run: move through a run's poses and fire a Zivid
3D capture at each scan pose, so octomap_server builds an occupancy map of the
workspace that can be frozen and reused for collision-aware planning.

Why this script exists
----------------------
The orchestrator's `moveto` tasks don't capture, and a single multi-task goal
runs poses back-to-back without stopping — but a single-shot Zivid must capture
while the arm is *stationary*. So we drive the run pose-by-pose: send each pose
as its own one-task `/beambot_execution` goal, wait for it to settle, then fire
`/capture`. MoveIt is reused across goals (launch_moveit_with_gripper is
idempotent for the same gripper), so the per-pose goals are cheap.

The map itself is built by the separate octomap pipeline
(`ros2 launch beambot octomap_test.launch.py`): each capture publishes
/points/xyz → pointcloud_relay downsamples → octomap_server integrates →
octomap_to_planning_scene pushes it into MoveIt. This script only sequences
move→capture and then saves the accumulated .bt.

Capture trigger: `/capture` (std_srvs/srv/Trigger) — the plain 3D capture, no
markers/detection. (trigger_zivid_capture.py uses /capture_and_detect_markers
for an unrelated rosbag workflow; for raw-cloud mapping /capture is simpler.)

Usage
-----
    # Terminal 1 — bringup (Zivid + orchestrator + MoveIt)
    ros2 launch beambot beambot_bringup.launch.py

    # Terminal 2 — octomap pipeline (relay + octomap_server + planning-scene bridge)
    ros2 launch beambot octomap_test.launch.py

    # Terminal 3 — drive the run and save the map
    ros2 run beambot capture_run.py src/cms/runs/oscan.json \
        --save src/beambot/config/oscan_octomap.bt

    # Later, reuse the frozen map:
    ros2 launch beambot octomap_test.launch.py octomap_path:=.../oscan_octomap.bt

    # Pure logic check, no ROS/hardware needed:
    python3 src/beambot/scripts/capture_run.py --self-check

Options
-------
    --capture-prefix STR  Capture only after moveto targets starting with this
                          (default "oscan"; skips transit poses like
                          safe_sample_transport).
    --capture-service N   Capture trigger service (default /capture).
    --capture-timeout S   Per-capture service timeout (default 30).
    --settle S            Pause after each capture so the relay+octomap_server
                          integrate the cloud before moving (default 2.0).
    --save PATH           Save the accumulated octomap here when done.
    --no-save             Skip saving (just build the live map in octomap_server).
"""

import argparse
import json
import subprocess
import sys

# ROS imports are done lazily inside run()/helpers so --self-check works
# without a sourced ROS environment.


def plan_capture_steps(run: dict, capture_prefix: str):
    """Pure: expand a run dict into an ordered list of (goal_json, do_capture).

    Each step is a one-task `/beambot_execution` goal carrying the run's
    start_gripper + full poses dict + exactly one task. do_capture is True for
    moveto tasks whose target name starts with capture_prefix.
    """
    base = {k: run[k] for k in ("start_gripper", "poses") if k in run}
    run_name = run.get("run_name", "capture_run")
    steps = []
    for task in run.get("tasks", []):
        goal = dict(base)
        goal["tasks"] = [task]
        goal["run_name"] = run_name
        target = task.get("target", "")
        do_capture = (
            task.get("task_type") == "moveto" and target.startswith(capture_prefix)
        )
        steps.append((json.dumps(goal), do_capture))
    return steps


def _send_goal(node, action_client, full_json: str):
    """Send one MTCExecution goal and block until it finishes. Returns (ok, msg)."""
    import rclpy
    from action_msgs.msg import GoalStatus
    from beambot_interfaces.action import MTCExecution

    goal = MTCExecution.Goal()
    goal.full_json = full_json
    send_future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        return False, "goal rejected"
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()
    ok = result.status == GoalStatus.STATUS_SUCCEEDED
    return ok, result.result.error_message


def _capture(node, trigger_client, timeout: float):
    """Fire the Zivid 3D capture. Returns (ok, msg)."""
    import rclpy
    from std_srvs.srv import Trigger

    future = trigger_client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        return False, f"capture timed out after {timeout}s"
    res = future.result()
    return res.success, res.message


def _save_map(path: str):
    """Save octomap_server's current map to a .bt via the standard saver node.

    # ponytail: shells out to octomap_saver_node (the maintainer's documented
    # save path); if its name/behavior differs on this distro, we print the
    # command so the user can run it by hand rather than failing the run.
    """
    cmd = [
        "ros2", "run", "octomap_server", "octomap_saver_node",
        "--ros-args", "-p", f"octomap_path:={path}",
    ]
    print(f"Saving octomap -> {path}")
    try:
        subprocess.run(cmd, timeout=30, check=True)
        print(f"Saved {path}")
    except Exception as exc:  # noqa: BLE001 - save is best-effort
        print(
            f"Auto-save failed ({exc}). Save manually with:\n  {' '.join(cmd)}",
            file=sys.stderr,
        )


def run(args) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from std_srvs.srv import Trigger
    from beambot_interfaces.action import MTCExecution

    with open(args.run_json) as f:
        run_data = json.load(f)
    steps = plan_capture_steps(run_data, args.capture_prefix)
    if not steps:
        print(f"No tasks in {args.run_json}", file=sys.stderr)
        return 1

    rclpy.init()
    node = Node("capture_run")
    action_client = ActionClient(node, MTCExecution, "beambot_execution")
    trigger_client = node.create_client(Trigger, args.capture_service)

    if not action_client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error("beambot_execution action server not available")
        return 1
    if not trigger_client.wait_for_service(timeout_sec=15.0):
        node.get_logger().error(f"capture service {args.capture_service} not available")
        return 1

    n_capture = sum(1 for _, c in steps if c)
    node.get_logger().info(
        f"Capture run: {len(steps)} moves, {n_capture} captures "
        f"(prefix '{args.capture_prefix}')"
    )

    captures = 0
    rc = 0
    for i, (full_json, do_capture) in enumerate(steps, 1):
        node.get_logger().info(f"[{i}/{len(steps)}] moving")
        ok, msg = _send_goal(node, action_client, full_json)
        if not ok:
            node.get_logger().error(f"move {i} failed: {msg} — aborting")
            rc = 1
            break
        if do_capture:
            node.get_logger().info(f"[{i}/{len(steps)}] capture -> {args.capture_service}")
            cok, cmsg = _capture(node, trigger_client, args.capture_timeout)
            if cok:
                captures += 1
            else:
                # Keep going — a missed view just means sparser coverage, not a
                # broken map. Better a partial map than aborting 14 good poses.
                node.get_logger().warning(f"capture {i} failed: {cmsg}")
            _sleep(node, args.settle)

    node.get_logger().info(f"Done: {captures}/{n_capture} captures succeeded")
    node.destroy_node()
    rclpy.shutdown()

    if rc == 0 and not args.no_save and args.save:
        _save_map(args.save)
    return rc


def _sleep(node, seconds: float):
    """Sleep using the node clock so we cooperate with ROS time."""
    import rclpy

    end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def _self_check():
    """Assert the pure planning logic. Runs without ROS."""
    run_data = {
        "start_gripper": "hande",
        "poses": {"safe_sample_transport": [0] * 6, "oscan1": [1] * 6, "oscan2": [2] * 6},
        "tasks": [
            {"task_type": "moveto", "target": "safe_sample_transport", "planning_type": "joint"},
            {"task_type": "moveto", "target": "oscan1", "planning_type": "joint"},
            {"task_type": "moveto", "target": "oscan2", "planning_type": "joint"},
        ],
        "run_name": "oscan",
    }
    steps = plan_capture_steps(run_data, "oscan")
    assert [c for _, c in steps] == [False, True, True], "capture only at oscan* poses"
    for goal_json, _ in steps:
        g = json.loads(goal_json)
        assert g["start_gripper"] == "hande"
        assert len(g["tasks"]) == 1, "one task per goal"
        assert "oscan1" in g["poses"], "full poses dict carried on every goal"
        assert g["run_name"] == "oscan"
    # empty run -> no steps
    assert plan_capture_steps({"tasks": []}, "oscan") == []
    print("self-check OK")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run_json", nargs="?", help="Path to the run JSON (poses + moveto tasks)")
    parser.add_argument("--capture-prefix", default="oscan")
    parser.add_argument("--capture-service", default="/capture")
    parser.add_argument("--capture-timeout", type=float, default=30.0)
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--save", default="")
    parser.add_argument("--no-save", action="store_true")
    parser.add_argument("--self-check", action="store_true", help="Run logic self-check and exit")
    args, _ = parser.parse_known_args()

    if args.self_check:
        _self_check()
        return 0
    if not args.run_json:
        parser.error("run_json is required (or pass --self-check)")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
