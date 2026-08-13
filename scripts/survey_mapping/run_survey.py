#!/usr/bin/env python3
"""Move through survey poses, capture one Zivid cloud per pose, and record a bag."""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import yaml
from beambot_interfaces.action import MoveToAction
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

MOVE_ACTION = "beambot_moveto"
CAPTURE_SERVICE = "/capture"
CLOUD_TOPIC = "/points/xyzrgba"
CAMERA_NODE = "zivid_camera"
SETTINGS_FILE = "manufacturing_specular.yml"
BAG_TOPICS = [CLOUD_TOPIC, "/tf", "/tf_static", "/joint_states"]

ZIVID_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_POSES = SCRIPT_DIR / "survey_poses.yaml"
DEFAULT_QOS_OVERRIDE = SCRIPT_DIR / "tf_qos_override.yaml"


def default_settings_file():
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = (
            Path(get_package_share_directory("beambot")) / "config" / SETTINGS_FILE
        )
        if installed.is_file():
            return installed
    except Exception:  # Package may not be built or sourced yet.
        pass
    return SCRIPT_DIR.parents[1] / "src" / "beambot" / "config" / SETTINGS_FILE


def load_poses(path):
    data = yaml.safe_load(Path(path).read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")

    poses = {}
    for name, joints in data.items():
        if (
            not isinstance(name, str)
            or not isinstance(joints, list)
            or len(joints) != 6
        ):
            print(f"WARN: skipping invalid pose {name!r}", file=sys.stderr)
            continue
        try:
            values = [float(value) for value in joints]
            if not all(math.isfinite(value) for value in values):
                raise ValueError
            poses[name] = values
        except (TypeError, ValueError):
            print(f"WARN: skipping invalid pose {name!r}", file=sys.stderr)
    return poses


class SurveyRunner(Node):
    def __init__(self, args):
        super().__init__("survey_runner")
        self.args = args
        self.move_client = ActionClient(self, MoveToAction, MOVE_ACTION)
        self.capture_client = self.create_client(Trigger, CAPTURE_SERVICE)
        self.cloud_count = 0
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self._on_cloud, ZIVID_QOS)

    def _on_cloud(self, _message):
        self.cloud_count += 1

    def wait_for_servers(self):
        self.get_logger().info(f"Waiting for {MOVE_ACTION}")
        if not self.move_client.wait_for_server(timeout_sec=15):
            self.get_logger().error(f"{MOVE_ACTION} is unavailable")
            return False
        if not self.args.dry_run and not self.capture_client.wait_for_service(
            timeout_sec=15
        ):
            self.get_logger().error(f"{CAPTURE_SERVICE} is unavailable")
            return False
        return True

    def apply_settings(self):
        path = Path(self.args.settings_file).resolve()
        if not path.is_file():
            self.get_logger().warning(f"Capture settings not found: {path}")
            return

        client = self.create_client(SetParameters, f"/{CAMERA_NODE}/set_parameters")
        if not client.wait_for_service(timeout_sec=10):
            self.get_logger().warning(
                f"Cannot configure {CAMERA_NODE}; keeping current settings"
            )
            return

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="settings_file_path",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=str(path),
                ),
            ),
            Parameter(
                name="settings_yaml",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=""
                ),
            ),
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        result = future.result() if future.done() else None
        if result and all(item.successful for item in result.results):
            self.get_logger().info(f"Applied capture settings: {path.name}")
        else:
            self.get_logger().warning(
                "Could not apply capture settings; keeping current settings"
            )

    def move_to(self, name, joints):
        goal = MoveToAction.Goal()
        goal.target = name
        goal.planning_type = "joint"
        goal.poses_json = json.dumps({name: joints})

        self.get_logger().info(f"Moving to {name}")
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            self.get_logger().error(f"Move to {name} was rejected")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.args.move_timeout
        )
        if not result_future.done():
            self.get_logger().error(f"Move to {name} timed out; cancelling")
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5)
            return False

        wrapped_result = result_future.result()
        result = wrapped_result.result if wrapped_result else None
        if result is None or not result.success:
            message = result.error_message if result else "no result"
            self.get_logger().error(f"Move to {name} failed: {message}")
            return False
        return True

    def capture(self, name):
        count_before = self.cloud_count
        future = self.capture_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.args.capture_timeout
        )
        result = future.result() if future.done() else None
        if result is None:
            self.get_logger().error(f"Capture timed out at {name}")
            return False
        if not result.success:
            self.get_logger().warning(
                f"Capture reported failure at {name}: {result.message}"
            )

        deadline = time.monotonic() + self.args.cloud_timeout
        while self.cloud_count == count_before:
            if time.monotonic() >= deadline:
                self.get_logger().error(f"No cloud received at {name}")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def run(self, poses):
        completed = 0
        total = len(poses)
        for index, (name, joints) in enumerate(poses.items(), start=1):
            self.get_logger().info(f"[{index}/{total}] {name}")
            if not self.move_to(name, joints):
                return False
            if self.args.settle > 0:
                time.sleep(self.args.settle)
            if self.args.dry_run or self.capture(name):
                completed += 1

        self.get_logger().info(f"Survey complete: {completed}/{total}")
        return completed == total


def start_bag(output, qos_override):
    command = ["ros2", "bag", "record", "-o", output]
    if qos_override:
        command += ["--qos-profile-overrides-path", qos_override]
    command += BAG_TOPICS
    print(f"[bag] {' '.join(command)}")
    return subprocess.Popen(command, start_new_session=True)


def stop_bag(process):
    if process is None or process.poll() is not None:
        return
    print("[bag] finalizing")
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=15)
    except ProcessLookupError:
        pass
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses", default=str(DEFAULT_POSES))
    parser.add_argument("--bag-out", default="survey_session")
    parser.add_argument("--no-bag", action="store_true")
    parser.add_argument("--settle", type=float, default=1.5)
    parser.add_argument("--move-timeout", type=float, default=120)
    parser.add_argument("--capture-timeout", type=float, default=25)
    parser.add_argument("--cloud-timeout", type=float, default=20)
    parser.add_argument("--settings-file", default=str(default_settings_file()))
    parser.add_argument("--no-set-settings", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--max-poses", type=int, default=0)
    parser.add_argument("--qos-overrides", default=str(DEFAULT_QOS_OVERRIDE))
    args, _ = parser.parse_known_args()

    if (
        args.settle < 0
        or min(args.move_timeout, args.capture_timeout, args.cloud_timeout) <= 0
        or args.max_poses < 0
    ):
        parser.error(
            "timeouts must be positive; --settle and --max-poses cannot be negative"
        )

    try:
        poses = load_poses(args.poses)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"ERROR: cannot load poses: {exc}", file=sys.stderr)
        return 1
    if not poses:
        print("ERROR: no valid survey poses", file=sys.stderr)
        return 1
    if args.max_poses > 0:
        poses = dict(list(poses.items())[: args.max_poses])

    if (
        not args.no_bag
        and not args.dry_run
        and args.qos_overrides
        and not Path(args.qos_overrides).is_file()
    ):
        print(f"ERROR: QoS override not found: {args.qos_overrides}", file=sys.stderr)
        return 1
    if not args.no_bag and not args.dry_run and Path(args.bag_out).exists():
        print(f"ERROR: bag output already exists: {args.bag_out}", file=sys.stderr)
        return 1

    rclpy.init()
    node = SurveyRunner(args)
    bag_process = None
    success = False
    try:
        if not node.wait_for_servers():
            return 1
        if not args.dry_run and not args.no_set_settings:
            node.apply_settings()
        if not args.dry_run and not args.no_bag:
            bag_process = start_bag(args.bag_out, args.qos_overrides)
            time.sleep(2)
            if bag_process.poll() is not None:
                print("ERROR: bag recorder failed to start", file=sys.stderr)
                return 1
        success = node.run(poses)
    except KeyboardInterrupt:
        print("\nSurvey interrupted", file=sys.stderr)
    finally:
        stop_bag(bag_process)
        node.destroy_node()
        rclpy.shutdown()

    if success and not args.dry_run and not args.no_bag:
        print(f"Bag written to {args.bag_out}")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
