#!/usr/bin/env python3
"""Loop Zivid /capture_2d to emulate a continuous 2D feed on /color/image_color.

The Zivid driver doesn't stream — it publishes a frame only per capture trigger.
This pokes /capture_2d in a loop so /color/image_color updates continuously. View
with:  ros2 run rqt_image_view rqt_image_view  (pick /color/image_color), or RViz.

Unlike the stock zivid_samples sample_capture_2d, this does NOT touch the driver's
2D settings — it reuses whatever beambot_bringup loaded via settings_2d_file_path
(zivid_settings.yml), so it won't clobber the settings spincoater detection shares.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 src/beambot/scripts/stream_zivid_2d.py            # as fast as the camera allows
    python3 src/beambot/scripts/stream_zivid_2d.py --rate 2   # ~2 Hz

ponytail: no arbiter here — this contends with the robot's 3D captures for the one
sensor. Run it standalone while testing the feed; add a "robot busy" pause before
running it alongside live picks.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class Zivid2DStream(Node):
    def __init__(self, rate_hz: float) -> None:
        super().__init__("zivid_2d_stream")
        # 0 => back-to-back (capture is blocking, so no overlap either way).
        self._period = 1.0 / rate_hz if rate_hz > 0 else 0.0
        self._n = 0  # frames received, for the FPS readout

        self._client = self.create_client(Trigger, "capture_2d")
        self.get_logger().info("Waiting for /capture_2d ...")
        if not self._client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("/capture_2d not available — is the Zivid driver up?")
            raise SystemExit(1)

        self.create_subscription(Image, "color/image_color", self._on_image, 10)
        # Measured feed rate over each 5 s window — the real number depends on
        # exposure (zivid_settings.yml) + transfer, not a documented spec.
        self._t0 = self.get_clock().now()
        self.create_timer(5.0, self._report_fps)

    def _on_image(self, msg: Image) -> None:
        self._n += 1

    def _report_fps(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._t0).nanoseconds / 1e9
        if dt > 0:
            self.get_logger().info(f"~{self._n / dt:.1f} Hz ({self._n} frames / {dt:.0f}s)")
        self._t0, self._n = now, 0

    def run(self) -> None:
        """Blocking capture loop: trigger, wait for the result, optionally pace, repeat."""
        self.get_logger().info("Streaming 2D — Ctrl-C to stop.")
        while rclpy.ok():
            cycle_start = self.get_clock().now().nanoseconds
            future = self._client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)  # drives subscription too
            result = future.result()
            if result is not None and not result.success:
                self.get_logger().warning(f"capture_2d failed: {result.message}")
            if self._period:
                # Pace the whole cycle to --rate: deadline is measured from cycle
                # START (before the capture), so the capture time counts toward the
                # period instead of being added on top. If a capture already takes
                # longer than the period, we don't wait — you run at the camera's
                # ceiling. Keep spinning so /color/image_color is still processed.
                end = cycle_start + int(self._period * 1e9)
                while rclpy.ok() and self.get_clock().now().nanoseconds < end:
                    rclpy.spin_once(self, timeout_sec=0.02)


def main() -> None:
    parser = argparse.ArgumentParser(description="Loop Zivid 2D captures as a live feed.")
    parser.add_argument("--rate", type=float, default=0.0,
                        help="Target Hz between captures (0 = as fast as the camera allows)")
    args, _ = parser.parse_known_args()

    rclpy.init()
    try:
        node = Zivid2DStream(args.rate)
        node.run()
    except (KeyboardInterrupt, SystemExit) as exc:
        rclpy.try_shutdown()
        if isinstance(exc, SystemExit):
            sys.exit(exc.code)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
