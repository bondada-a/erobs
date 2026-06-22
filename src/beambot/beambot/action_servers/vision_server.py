#!/usr/bin/env python3
"""VisionScanAction server — batch marker scanning + tag-pose cache.

vision_moveto migrated to the unified VisionTaskAction server (issue #88), so
this node now hosts only vision_scan. It keeps a VisionEngine instance because
scan_all_tags lives there and populates the tag-pose cache; the cache is
consumed by the unified pipeline's marker detector via its own VisionEngine.
"""

from std_srvs.srv import Trigger

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.pipeline.vision_engine import VisionEngine
from beambot_interfaces.action import VisionScanAction


class VisionActionServer(BaseActionServer):
    """Action server for batch marker scanning (VisionScanAction)."""

    def __init__(self):
        """Initialize the Vision scan server."""
        # Hosts only VisionScan now; register it as the primary action.
        super().__init__(
            node_name="beambot_vision_server",
            action_name="beambot_vision_scan",
            action_type=VisionScanAction,
        )

        # Service to reset TF buffer after tool exchange (URDF change)
        self._reset_tf_service = self.create_service(
            Trigger, "beambot_vision_reset_tf", self._reset_tf_callback
        )
        self.get_logger().info("TF reset service: beambot_vision_reset_tf")

    def create_stages(self):
        """Build the VisionEngine with camera config from beamline config."""
        from beambot.config_loader import load_beamline_config

        config, _ = load_beamline_config()
        camera_config = config.get("camera", {})
        self.get_logger().info(
            f"Camera config: type={camera_config.get('type')}, "
            f"frame={camera_config.get('frame')}"
        )

        return VisionEngine(
            self,
            camera_type=camera_config.get("type"),
            camera_frame=camera_config.get("frame"),
            marker_dictionary=camera_config.get("marker_dictionary"),
        )

    def _execute(self, goal_handle):
        """Execute VisionScanAction — batch scan all markers from multiple positions.

        Scans from multiple robot positions, detects ALL visible ArUco markers
        at each position (multiple captures per position), averages the poses,
        and caches them. The base class succeeds/aborts the goal from
        result.success — do NOT call goal_handle.succeed()/abort() here.
        """
        goal = goal_handle.request

        # Parse scan positions from flattened array
        num_positions = goal.num_scan_positions
        flat = list(goal.scan_positions_flat)

        if len(flat) != num_positions * 6:
            self.get_logger().error(
                f"Invalid scan_positions_flat length: {len(flat)}, "
                f"expected {num_positions * 6}"
            )
            result = VisionScanAction.Result()
            result.success = False
            result.error_message = "Invalid scan positions"
            result.tags_detected = 0
            return result

        scan_positions = [flat[i * 6 : (i + 1) * 6] for i in range(num_positions)]

        # Get parameters with defaults
        scans_per_position = (
            goal.scans_per_position if goal.scans_per_position > 0 else 3
        )
        timeout = goal.timeout if goal.timeout > 0 else 10.0

        self.get_logger().info(
            f"VisionScan: {num_positions} positions × {scans_per_position} scans"
        )

        # Run the batch scan (populates _stages._tag_pose_cache)
        tags_detected = self._stages.scan_all_tags(
            scan_positions=scan_positions,
            scans_per_position=scans_per_position,
            timeout=timeout,
            settle_time=0.3,  # Default settle time after each move
        )

        # Build result
        result = VisionScanAction.Result()
        result.success = tags_detected > 0
        result.tags_detected = tags_detected
        result.error_message = "" if result.success else "No tags detected"
        return result

    def _reset_tf_callback(self, request, response):
        """Handle TF reset service call (after tool exchange)."""
        self._stages.reset_tf()
        response.success = True
        response.message = "TF buffer cleared and listener re-created"
        return response


def main(args=None):
    run_server(VisionActionServer, args)


if __name__ == "__main__":
    main()
