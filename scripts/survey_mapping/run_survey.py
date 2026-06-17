#!/usr/bin/env python3
"""Drive the arm through survey poses, trigger a Zivid capture at each, record a bag.

Step 2 of 3 in the survey-mapping workflow
------------------------------------------
    teach_survey_poses.py   -> record viewpoints into survey_poses.yaml
    run_survey.py            <- you are here (move + capture + record)
    merge_survey_bag.py     -> offline: bag -> merged point cloud (base_link)

What this does
--------------
For each pose in survey_poses.yaml, in order:

    1. send a joint goal to the beambot_moveto action server (same planning
       path the orchestrator uses, so collision-aware MTC planning applies)
    2. wait for the move to finish, then SETTLE (let vibration die)
    3. trigger a Zivid 3D capture
    4. WAIT for the actual point cloud to publish on /points/xyzrgba
       *** this wait is the whole point — see "Timing" below ***
    5. go to the next pose

Meanwhile a `ros2 bag record` of the cloud + TF + joint topics runs in the
background (spawned by this script unless --no-bag), capturing every cloud and
the full transform history so merge_survey_bag.py can reconstruct the scene.

Timing — why we wait for the cloud, not the service
---------------------------------------------------
The Zivid driver is trigger-only and SLOW to deliver: the capture service
returns ~3-4s BEFORE the ~40MB cloud actually lands on the topic, and the
driver stamps the cloud ~300ms in the past (see beambot/camera/zivid.py). If
we moved on as soon as the service returned, the cloud would publish *while the
arm is already moving to the next pose* — its timestamp would resolve (via TF)
to an in-motion, wrong robot pose, smearing the merged map. So after triggering
we hold the arm still and block until a fresh cloud arrives.

Capture trigger — which service + color
---------------------------------------
We trigger the bare /capture service (std_srvs/Trigger). It captures using
whatever the driver's `settings_file_path` points at, so we FIRST apply the
bundled "Manufacturing: Specular" preset (skip with --no-set-settings) and then
trigger. That preset has Sampling.Color: rgb, so the cloud carries real RGB.

This matters: the /capture_and_detect_markers service the older
trigger_zivid_capture.py uses inherits the driver's launched scene_capture
preset, which is Sampling.Color: grayscale — every point comes back R=G=B. For
a colored map we want the Specular RGB preset + bare /capture instead.

Usage
-----
    # Terminal: source ROS + workspace, then run. The bag is spawned for you.
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 scripts/survey_mapping/run_survey.py \
        --poses scripts/survey_mapping/survey_poses.yaml \
        --bag-out survey_session

    # If you'd rather start the bag yourself in another terminal:
    #   ros2 bag record -o survey_session /points/xyzrgba /color/image_color \
    #       /tf /tf_static /joint_states
    # then run with --no-bag.

Options
-------
    --poses PATH         survey_poses.yaml from teach_survey_poses.py (required-ish;
                         defaults to ./survey_poses.yaml next to this script)
    --bag-out NAME       output bag directory name (default: survey_session)
    --no-bag             don't spawn ros2 bag (you record it yourself)
    --settle SECONDS     dwell after arrival before capture (default: 1.5)
    --move-timeout SEC   per-move planning+execution timeout (default: 120)
    --capture-timeout S  per-capture service timeout (default: 25)
    --cloud-timeout S    wait for the cloud to publish after trigger (default: 20)
    --no-set-settings    don't apply the Specular preset; use launched settings
    --dry-run            move + settle only; skip captures (sanity-check motion)
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

from beambot_interfaces.action import MoveToAction

# ----- constants matching the rest of the codebase -----
MOVETO_ACTION = "beambot_moveto"  # MoveToActionServer (move_to_server.py)
# Bare 3D-capture trigger. Verified live to be advertised as std_srvs/Trigger
# and to honor the driver's settings_file_path — so the Specular preset we set
# up front (Sampling.Color: rgb) yields a real RGB cloud, unlike the grayscale
# scene_capture preset the marker service inherits. No dummy marker_ids and no
# spurious "failed to detect markers" warning.
CAPTURE_SERVICE = "/capture"
CLOUD_TOPIC = "/points/xyzrgba"
CAMERA_NODE = "zivid_camera"
_SETTINGS_BASENAME = "manufacturing_specular.yml"

# Topics the bag must contain for an offline merge:
#   cloud (payload) + TF (to place each cloud in base_link) + joints (debug).
BAG_TOPICS = [
    CLOUD_TOPIC,
    "/color/image_color",  # texture, for optional colored mesh later
    "/tf",
    "/tf_static",
    "/joint_states",
]

# Match the Zivid publisher QoS (RELIABLE/VOLATILE/KEEP_LAST depth 1) so our
# subscription actually receives the cloud — mismatched QoS silently drops it.
_ZIVID_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

_DEFAULT_POSES = os.path.join(os.path.dirname(__file__), "survey_poses.yaml")
_DEFAULT_QOS_OVERRIDE = os.path.join(os.path.dirname(__file__), "tf_qos_override.yaml")


def _default_settings_file() -> str:
    """Locate the bundled Manufacturing: Specular preset (installed or in-tree)."""
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = os.path.join(
            get_package_share_directory("beambot"), "config", _SETTINGS_BASENAME
        )
        if os.path.isfile(installed):
            return installed
    except Exception:  # noqa: BLE001 - ament missing / pkg not built; fall back to source
        pass
    return os.path.normpath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "src",
            "beambot",
            "config",
            _SETTINGS_BASENAME,
        )
    )


def load_poses(path: str) -> dict:
    """Parse the inline-list survey YAML into {name: [6 floats in degrees]}.

    Insertion order is preserved (dict keeps it), so poses are visited in file
    order. Hand-rolled parser so no PyYAML dependency is required.
    """
    poses = {}
    with open(path) as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            name, _, rest = line.partition(":")
            rest = rest.strip().strip("[]")
            try:
                vals = [float(v) for v in rest.split(",")]
            except ValueError:
                continue
            if len(vals) != 6:
                print(
                    f"WARN: {name.strip()} has {len(vals)} values, expected 6 — skipping"
                )
                continue
            poses[name.strip()] = vals
    return poses


class SurveyRunner(Node):
    """Sequences moves + captures; waits for each cloud before advancing."""

    def __init__(self, args):
        super().__init__("survey_runner")
        self._args = args

        self._move_client = ActionClient(self, MoveToAction, MOVETO_ACTION)
        self._capture_client = self.create_client(Trigger, CAPTURE_SERVICE)

        # Cloud arrival detection. We don't keep the cloud — we only need to
        # know a FRESH one landed after our trigger, so we track message count.
        self._cloud_count = 0
        self._cloud_lock = threading.Lock()
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self._on_cloud, _ZIVID_QOS)

    def _on_cloud(self, _msg: PointCloud2):
        with self._cloud_lock:
            self._cloud_count += 1

    def _cloud_seq(self) -> int:
        with self._cloud_lock:
            return self._cloud_count

    # ---- setup / teardown ----

    def wait_for_servers(self) -> bool:
        self.get_logger().info(f"Waiting for '{MOVETO_ACTION}' action server...")
        if not self._move_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                f"'{MOVETO_ACTION}' action server not available. Is beambot bringup up?"
            )
            return False
        if not self._args.dry_run:
            self.get_logger().info(f"Waiting for '{CAPTURE_SERVICE}'...")
            if not self._capture_client.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(
                    f"'{CAPTURE_SERVICE}' not available. Is the Zivid driver running?"
                )
                return False
        return True

    def apply_settings_preset(self):
        """Apply the Specular 3D preset once, like trigger_zivid_capture.py.

        Non-fatal: on any failure we log and keep the driver's launched
        settings so the survey still proceeds.
        """
        settings_file = os.path.abspath(self._args.settings_file)
        if not os.path.isfile(settings_file):
            self.get_logger().warning(
                f"Settings file not found: {settings_file} — keeping current settings."
            )
            return
        param_srv = f"/{CAMERA_NODE}/set_parameters"
        client = self.create_client(SetParameters, param_srv)
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warning(
                f"'{param_srv}' unavailable — keeping current capture settings."
            )
            return
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="settings_file_path",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=settings_file
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.done() and future.result() is not None:
            if all(r.successful for r in future.result().results):
                self.get_logger().info(
                    f"Applied capture preset: {os.path.basename(settings_file)}"
                )
                return
        self.get_logger().warning(
            f"Could not apply preset {os.path.basename(settings_file)} — "
            f"keeping current settings."
        )

    # ---- per-pose primitives ----

    def move_to(self, name: str, joints_deg: list) -> bool:
        """Send a joint goal as a one-pose poses_json move, block until done.

        We pass the joints under `target`+`poses_json` exactly like the
        orchestrator does (orchestrator._create_moveto_goal), so the move runs
        through the same MTC joint planner — collision-aware, and degrees are
        converted to radians inside the stage.
        """
        import json

        goal = MoveToAction.Goal()
        goal.target = name
        goal.planning_type = "joint"
        goal.poses_json = json.dumps({name: joints_deg})

        self.get_logger().info(f"  -> moving to {name}: {joints_deg}")
        send_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"  move goal for {name} was REJECTED")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._args.move_timeout
        )
        if not result_future.done():
            self.get_logger().error(f"  move to {name} timed out")
            return False
        result = result_future.result().result
        if not result.success:
            self.get_logger().error(f"  move to {name} failed: {result.error_message}")
            return False
        return True

    def capture_and_wait_for_cloud(self, name: str) -> bool:
        """Trigger a capture, then block until a fresh cloud publishes.

        Returns True only once the cloud count advances past the pre-trigger
        value — guaranteeing the bag recorded a cloud while the arm was still.
        """
        seq_before = self._cloud_seq()

        self.get_logger().info(f"  capturing at {name}...")
        cap_future = self._capture_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self, cap_future, timeout_sec=self._args.capture_timeout
        )
        if not cap_future.done() or cap_future.result() is None:
            self.get_logger().error(f"  capture service timed out at {name}")
            return False
        if not cap_future.result().success:
            self.get_logger().warning(
                f"  capture reported failure at {name}: {cap_future.result().message}"
            )
            # keep going — wait below confirms whether a cloud actually published

        # Now wait for the heavy cloud to actually land on the topic.
        self.get_logger().info("  waiting for point cloud to publish...")
        deadline = self.get_clock().now().nanoseconds + int(
            self._args.cloud_timeout * 1e9
        )
        while self._cloud_seq() == seq_before:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > deadline:
                self.get_logger().error(
                    f"  no cloud on {CLOUD_TOPIC} within {self._args.cloud_timeout}s at {name}"
                )
                return False
        self.get_logger().info("  cloud received.")
        return True

    # ---- main sequence ----

    def run(self, poses: dict) -> bool:
        ok = 0
        total = len(poses)
        for i, (name, joints_deg) in enumerate(poses.items(), start=1):
            self.get_logger().info(f"[{i}/{total}] {name}")
            if not self.move_to(name, joints_deg):
                self.get_logger().error(f"  skipping capture at {name} (move failed)")
                continue

            # Settle: hold still so vibration dies and the cloud is crisp.
            if self._args.settle > 0:
                time.sleep(self._args.settle)

            if self._args.dry_run:
                ok += 1
                continue

            if self.capture_and_wait_for_cloud(name):
                ok += 1

        self.get_logger().info(f"Survey done: {ok}/{total} poses captured.")
        return ok == total


def _spawn_bag(bag_out: str, qos_override: str = "") -> subprocess.Popen:
    """Start `ros2 bag record` for the survey topics in its own process group.

    Own process group so we can SIGINT just the bag (clean .db3 finalize)
    without killing this script.

    qos_override: path to a rosbag2 QoS-overrides YAML. Critical for /tf, which
    has both a TRANSIENT_LOCAL publisher (tcp_pose_broadcaster) and a VOLATILE
    one (robot_state_publisher). Without forcing the /tf subscription to
    VOLATILE, rosbag2 auto-picks TRANSIENT_LOCAL and silently drops the volatile
    arm transforms — splitting the TF tree so the offline merge can't connect
    base_link to zivid_optical_frame. See tf_qos_override.yaml.
    """
    cmd = ["ros2", "bag", "record", "-o", bag_out]
    if qos_override:
        cmd += ["--qos-profile-overrides-path", qos_override]
    cmd += BAG_TOPICS
    print(f"[bag] {' '.join(cmd)}")
    return subprocess.Popen(cmd, preexec_fn=os.setsid)


def _stop_bag(proc: subprocess.Popen):
    """SIGINT the bag's process group so rosbag2 flushes and closes cleanly."""
    if proc is None or proc.poll() is not None:
        return
    print("[bag] stopping (SIGINT) and finalizing...")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=15)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            proc.terminate()
        except ProcessLookupError:
            pass
    print("[bag] stopped.")


def main():
    parser = argparse.ArgumentParser(
        description="Drive arm through survey poses, capture Zivid clouds, record a bag."
    )
    parser.add_argument(
        "--poses",
        default=_DEFAULT_POSES,
        help=f"survey_poses.yaml (default: {_DEFAULT_POSES})",
    )
    parser.add_argument(
        "--bag-out",
        default="survey_session",
        help="rosbag output directory (default: survey_session)",
    )
    parser.add_argument(
        "--no-bag",
        action="store_true",
        help="don't spawn ros2 bag; you record it yourself",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=1.5,
        help="seconds to dwell after arrival before capture (default: 1.5)",
    )
    parser.add_argument(
        "--move-timeout",
        type=float,
        default=120.0,
        help="per-move plan+exec timeout seconds (default: 120)",
    )
    parser.add_argument(
        "--capture-timeout",
        type=float,
        default=25.0,
        help="per-capture service timeout seconds (default: 25)",
    )
    parser.add_argument(
        "--cloud-timeout",
        type=float,
        default=20.0,
        help="wait for cloud to publish after trigger (default: 20)",
    )
    parser.add_argument(
        "--settings-file",
        default=_default_settings_file(),
        help="3D capture preset .yml (default: bundled Specular)",
    )
    parser.add_argument(
        "--no-set-settings",
        action="store_true",
        help="don't apply Specular preset; use launched settings",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="move + settle only; skip captures (motion sanity check)",
    )
    parser.add_argument(
        "--max-poses",
        type=int,
        default=0,
        help="only run the first N poses (0 = all). Use a small N "
        "like 10 for a quick end-to-end pipeline test.",
    )
    parser.add_argument(
        "--qos-overrides",
        default=_DEFAULT_QOS_OVERRIDE,
        help="rosbag2 QoS-overrides YAML (forces /tf VOLATILE so "
        "the arm transforms aren't dropped). Empty string "
        "disables the override.",
    )
    args, _ = parser.parse_known_args()

    if not os.path.isfile(args.poses):
        print(f"ERROR: poses file not found: {args.poses}")
        print("Run teach_survey_poses.py first.")
        sys.exit(1)

    if args.qos_overrides and not os.path.isfile(args.qos_overrides):
        print(
            f"WARN: QoS overrides file not found: {args.qos_overrides} — "
            f"recording /tf with auto QoS (arm transforms may be dropped!)"
        )
        args.qos_overrides = ""

    # Fail fast if the bag dir already exists: `ros2 bag record` refuses to
    # overwrite and exits immediately, which would otherwise leave us driving
    # the whole survey into a dead recorder (clouds captured, nothing saved).
    if not args.no_bag and not args.dry_run and os.path.exists(args.bag_out):
        print(f"ERROR: bag output dir already exists: {args.bag_out}")
        print(
            "ros2 bag record won't overwrite it. Use a fresh --bag-out name "
            "or remove the existing directory, then re-run."
        )
        sys.exit(1)

    poses = load_poses(args.poses)
    if not poses:
        print(f"ERROR: no valid poses parsed from {args.poses}")
        sys.exit(1)

    # Limit to the first N poses for a quick test (dict preserves file order).
    if args.max_poses > 0 and args.max_poses < len(poses):
        kept = list(poses)[: args.max_poses]
        poses = {k: poses[k] for k in kept}
        print(
            f"--max-poses {args.max_poses}: running first {len(poses)} of "
            f"the file's poses"
        )
    print(f"Loaded {len(poses)} survey pose(s): {', '.join(poses)}")

    rclpy.init()
    node = SurveyRunner(args)

    bag_proc = None
    try:
        if not node.wait_for_servers():
            sys.exit(1)

        if not args.dry_run and not args.no_set_settings:
            node.apply_settings_preset()

        # Start recording BEFORE the first move so /tf_static (latched) and the
        # whole motion history are in the bag.
        if not args.no_bag and not args.dry_run:
            bag_proc = _spawn_bag(args.bag_out, args.qos_overrides)
            time.sleep(2.0)  # let the recorder discover topics & latch /tf_static

            # Verify the recorder actually survived startup. `ros2 bag record`
            # exits immediately on errors (e.g. output dir exists), and a dead
            # recorder would otherwise look identical to a healthy one — we'd
            # drive the whole survey capturing clouds that nobody is recording.
            if bag_proc.poll() is not None:
                print(
                    f"ERROR: bag recorder exited immediately "
                    f"(code {bag_proc.returncode}) — NOT recording. Aborting "
                    f"before moving the robot. Check the [bag] error above "
                    f"(commonly: output dir already exists)."
                )
                sys.exit(1)
            print("[bag] recorder is alive — starting survey.")

        success = node.run(poses)

    except KeyboardInterrupt:
        print("\nInterrupted — stopping.")
        success = False
    finally:
        _stop_bag(bag_proc)
        node.destroy_node()
        rclpy.shutdown()

    if not args.dry_run and not args.no_bag:
        print(f"\nBag written to: {args.bag_out}")
        print(f"Next: merge_survey_bag.py --bag {args.bag_out} --out survey_map.ply")
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
