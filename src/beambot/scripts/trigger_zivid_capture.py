#!/usr/bin/env python3
"""
Periodically trigger Zivid captures so a rosbag can record point clouds.

Why this exists
---------------
The Zivid ROS driver does NOT stream continuously. The /points/xyzrgba
PointCloud2 topic only publishes a frame *in response to a capture trigger*
(at rest its publisher count is 0). A rosbag started against the camera
topics therefore records nothing from the camera unless something pokes the
driver on an interval. This script is that poke.

It calls /capture_and_detect_markers on a fixed interval (default 3.0 s).
That service is the only live trigger in the standard bringup that produces a
full 3D point cloud (the bare /capture 3D-only service is not advertised, and
/capture_2d yields a 2D image with no cloud). We discard the marker-detection
result entirely; we only want the capture side-effect that publishes
/points/xyzrgba and /color/image_color.

Note: the service rejects an empty marker_ids list, so we populate a dummy
ID range + dictionary even though the detection output is ignored.

Capture settings ("Manufacturing: Specular" preset)
----------------------------------------------------
Before the first capture, this script switches the Zivid 3D capture settings
to the "Manufacturing: Specular" preset, tuned for shiny / reflective parts
that need a lot of dynamic range. The preset settings were serialized from
the Zivid SDK (2.17.1, model zivid2PlusMR60) into
beambot/config/manufacturing_specular.yml — presets are coupled to SDK
versions, so we pin the .yml rather than referencing the preset by name.

We apply it by setting the driver's `settings_file_path` parameter at runtime
(and clearing `settings_yaml`, since the driver rejects both being set). The
driver caches settings lazily but resets that cache on any change to those
parameters (see capture_settings_controller.hpp), so the next capture uses
Specular with no relaunch. Pass --no-set-settings to leave whatever the
driver was launched with, or --settings-file PATH to use a different preset.

Usage
-----
    # In one terminal, start recording everything (incl. point clouds):
    ros2 bag record -a -o zivid_session

    # In another terminal, drive the captures:
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 src/beambot/scripts/trigger_zivid_capture.py
    # or, if installed:  ros2 run beambot trigger_zivid_capture

Options
-------
    --interval SECONDS   Seconds between captures (default: 3.0)
    --count N            Stop after N captures (default: 0 = run forever)
    --timeout SECONDS    Per-call service timeout (default: 20.0)
    --service NAME       Trigger service (default: /capture_and_detect_markers)
    --dictionary NAME    ArUco dictionary for the dummy request (default: aruco4x4_50)
    --settings-file PATH 3D capture settings .yml to apply (default: the bundled
                         Manufacturing: Specular preset)
    --no-set-settings    Don't change settings; use whatever the driver launched with
    --camera-node NAME   Zivid driver node name for the parameter call (default: zivid_camera)
"""

import argparse
import os
import sys

import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rclpy.node import Node

from zivid_interfaces.srv import CaptureAndDetectMarkers

_SETTINGS_BASENAME = "manufacturing_specular.yml"


def _default_settings_file() -> str:
    """Locate the bundled Manufacturing: Specular preset .yml.

    Prefer the installed package share dir (where `ros2 run` finds it), and
    fall back to the source-tree layout (scripts/ -> ../config/) so the script
    also works run directly from a checkout.
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = os.path.join(
            get_package_share_directory("beambot"), "config", _SETTINGS_BASENAME
        )
        if os.path.isfile(installed):
            return installed
    except Exception:  # noqa: BLE001 - ament not available / package not built
        pass
    return os.path.normpath(
        os.path.join(os.path.dirname(__file__), "..", "config", _SETTINGS_BASENAME)
    )


# Default 3D capture preset: "Manufacturing: Specular", serialized from the
# Zivid SDK into beambot/config/; override with --settings-file.
_DEFAULT_SETTINGS_FILE = _default_settings_file()


def _spin_until_complete(node: Node, future, timeout: float) -> bool:
    """Spin `node` until `future` completes or `timeout` (seconds) elapses.

    Used for the one-shot parameter call during setup, before rclpy.spin() is
    driving the node. Uses the node clock for the deadline. Returns True if the
    future completed in time.
    """
    deadline = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds >= deadline:
            break
    return future.done()


class ZividCaptureTrigger(Node):
    """Timer-driven node that fires Zivid captures at a fixed interval."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("zivid_capture_trigger")
        self._interval = args.interval
        self._max_count = args.count
        self._timeout = args.timeout
        self._dictionary = args.dictionary
        self._service_name = args.service
        self._camera_node = args.camera_node
        self._settings_file = None if args.no_set_settings else args.settings_file

        self._count = 0          # successful/attempted captures so far
        self._in_flight = False  # guard against overlapping calls if a capture runs long

        self._client = self.create_client(CaptureAndDetectMarkers, self._service_name)
        self.get_logger().info(f"Waiting for service '{self._service_name}'...")
        if not self._client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(
                f"Service '{self._service_name}' not available. Is the Zivid "
                f"driver running (e.g. via beambot_bringup)?"
            )
            raise SystemExit(1)

        # Apply the capture preset before the first trigger. The driver resets
        # its cached settings when these parameters change, so subsequent
        # captures use the new preset without a relaunch.
        if self._settings_file:
            self._apply_settings_file(self._settings_file)

        self.get_logger().info(
            f"Triggering captures every {self._interval:.1f}s"
            + (f" for {self._max_count} captures" if self._max_count else " (forever, Ctrl-C to stop)")
        )
        # Fire one immediately, then on the interval.
        self._fire()
        self._timer = self.create_timer(self._interval, self._fire)

    def _apply_settings_file(self, settings_file: str) -> None:
        """Point the Zivid driver at a 3D-settings .yml (the Specular preset).

        Sets `settings_file_path` and clears `settings_yaml` on the camera node
        in one SetParameters call — the driver errors if both are non-empty.
        Non-fatal on failure: we log and fall back to the launched settings so
        recording still proceeds.
        """
        settings_file = os.path.abspath(settings_file)
        if not os.path.isfile(settings_file):
            self.get_logger().error(
                f"Settings file not found: {settings_file} — keeping the driver's "
                f"current settings. (Pass --settings-file or --no-set-settings.)"
            )
            return

        param_srv = f"/{self._camera_node}/set_parameters"
        client = self.create_client(SetParameters, param_srv)
        self.get_logger().info(f"Setting capture preset via '{param_srv}'...")
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warning(
                f"Parameter service '{param_srv}' unavailable — keeping current "
                f"settings. Is --camera-node correct (default 'zivid_camera')?"
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
            # Clear the inline-YAML param so the driver doesn't see both set.
            Parameter(
                name="settings_yaml",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=""
                ),
            ),
        ]
        future = client.call_async(request)
        if not _spin_until_complete(self, future, 10.0):
            self.get_logger().warning("set_parameters timed out — keeping current settings.")
            return

        results = future.result().results
        if all(r.successful for r in results):
            self.get_logger().info(
                f"Applied 'Manufacturing: Specular' preset from {settings_file}"
            )
        else:
            reasons = "; ".join(r.reason for r in results if not r.successful)
            self.get_logger().warning(f"Preset not fully applied: {reasons}")

    def _build_request(self) -> CaptureAndDetectMarkers.Request:
        """Build a dummy detection request — the service rejects empty marker_ids,
        so we send a throwaway range. The detection result is ignored; we only
        want the capture side-effect that publishes the point cloud."""
        request = CaptureAndDetectMarkers.Request()
        request.marker_ids = list(range(50))  # arbitrary; detection output discarded
        request.marker_dictionary = self._dictionary
        return request

    def _fire(self) -> None:
        if self._in_flight:
            self.get_logger().warning(
                "Previous capture still in flight — skipping this tick "
                "(capture takes longer than the interval)."
            )
            return

        if self._max_count and self._count >= self._max_count:
            self.get_logger().info(f"Reached {self._max_count} captures — shutting down.")
            rclpy.shutdown()
            return

        self._count += 1
        n = self._count
        self._in_flight = True
        self.get_logger().info(f"Capture #{n} -> {self._service_name}")

        future = self._client.call_async(self._build_request())

        def _done(fut, n=n):
            self._in_flight = False
            try:
                result = fut.result()
            except Exception as exc:  # noqa: BLE001 - log any service failure, keep looping
                self.get_logger().error(f"Capture #{n} raised: {exc}")
                return
            if result is not None and result.success:
                self.get_logger().info(f"Capture #{n} OK — point cloud published.")
            else:
                msg = getattr(result, "message", "(no message)")
                self.get_logger().warning(f"Capture #{n} reported failure: {msg}")
            if self._max_count and n >= self._max_count:
                self.get_logger().info(f"Reached {self._max_count} captures — shutting down.")
                rclpy.shutdown()

        future.add_done_callback(_done)


def main() -> None:
    parser = argparse.ArgumentParser(description="Periodically trigger Zivid captures for rosbag recording.")
    parser.add_argument("--interval", type=float, default=3.0, help="Seconds between captures (default: 3.0)")
    parser.add_argument("--count", type=int, default=0, help="Stop after N captures (0 = forever)")
    parser.add_argument("--timeout", type=float, default=20.0, help="Per-call service timeout seconds (default: 20.0)")
    parser.add_argument("--service", default="/capture_and_detect_markers", help="Trigger service name")
    parser.add_argument("--dictionary", default="aruco4x4_50", help="ArUco dictionary for the dummy request")
    parser.add_argument(
        "--settings-file",
        default=_DEFAULT_SETTINGS_FILE,
        help="3D capture settings .yml to apply (default: bundled Manufacturing: Specular preset)",
    )
    parser.add_argument(
        "--no-set-settings",
        action="store_true",
        help="Don't change capture settings; use whatever the driver was launched with",
    )
    parser.add_argument(
        "--camera-node",
        default="zivid_camera",
        help="Zivid driver node name for the set_parameters call (default: zivid_camera)",
    )
    # Tolerate ROS args (e.g. when launched via ros2 run) by ignoring unknowns.
    args, _ = parser.parse_known_args()

    rclpy.init()
    try:
        node = ZividCaptureTrigger(args)
    except SystemExit as exc:
        rclpy.try_shutdown()
        sys.exit(exc.code)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted — stopping capture trigger.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
