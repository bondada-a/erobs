#!/usr/bin/env python3
"""Sequence robot motion and Zivid captures for Octomap mapping.

Requires robot/camera bringup and octomap_mapping.launch.py running separately.
Use --self-check to verify task sequencing without ROS or hardware.
"""

import argparse
import json
import subprocess
import sys

# Keep ROS imports local so --self-check works without ROS.


def plan_capture_steps(run: dict, capture_prefix: str):
    """Return single-task goals and capture flags for matching moveto targets."""
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
    """Wait for one MTCExecution goal; return (success, message)."""
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
    """Request a Zivid 3D capture; return (success, message)."""
    import rclpy
    from std_srvs.srv import Trigger

    future = trigger_client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        return False, f"capture timed out after {timeout}s"
    res = future.result()
    return res.success, res.message


def _save_map(path: str):
    """Save the current map with octomap_saver_node."""
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
    node = None
    try:
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
                    # Continue after capture failures; the map may be incomplete.
                    node.get_logger().warning(f"capture {i} failed: {cmsg}")
                _sleep(node, args.settle)

        node.get_logger().info(f"Done: {captures}/{n_capture} captures succeeded")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

    if rc == 0 and not args.no_save and args.save:
        _save_map(args.save)
    return rc


def _sleep(node, seconds: float):
    """Wait on the node clock while processing callbacks."""
    import rclpy

    end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def _self_check():
    """Check task sequencing without ROS."""
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
