#!/usr/bin/env python3
"""VisionMoveToAction and VisionScanAction server.

Handles vision-guided movement via ArUco markers and batch scanning.
Both actions share the same VisionStages instance so the tag pose cache
populated by vision_scan is available to vision_moveto.
"""

import yaml
from rclpy.action import ActionServer
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.stages.vision_stages import VisionStages
from beambot_interfaces.action import VisionMoveToAction, VisionScanAction


class VisionActionServer(BaseActionServer):
    """Action server for vision-guided operations.

    Handles:
    - VisionMoveToAction: ArUco marker detection and motion to detected pose
    - VisionScanAction: Batch scan all markers from multiple positions, cache results

    Both actions share the same VisionStages instance so the cache populated
    by vision_scan is available to vision_moveto.
    """

    def __init__(self):
        """Initialize the Vision action server."""
        super().__init__(
            node_name="beambot_vision_server",
            action_name="beambot_vision_moveto",
            action_type=VisionMoveToAction,
        )

        # Add second action server for batch scanning (shares same stages)
        self._scan_action_server = ActionServer(
            self,
            VisionScanAction,
            "beambot_vision_scan",
            execute_callback=self._execute_scan,
        )
        self.get_logger().info("VisionScan action server started: beambot_vision_scan")

        # Service to reset TF buffer after tool exchange (URDF change)
        self._reset_tf_service = self.create_service(
            Trigger, "beambot_vision_reset_tf", self._reset_tf_callback
        )
        self.get_logger().info("TF reset service: beambot_vision_reset_tf")

    def initialize_stages(self):
        """Create VisionStages instance with camera config from beamline config."""
        # Load camera config from beamline config file
        self.declare_parameter(
            "beamline_config",
            get_package_share_directory("beambot") + "/config/default_beamline.yaml"
        )
        config_file = self.get_parameter("beamline_config").value

        camera_config = {}
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            camera_config = config.get("camera", {})
            self.get_logger().info(
                f"Camera config: type={camera_config.get('type', 'zivid')}, "
                f"frame={camera_config.get('frame', 'zivid_optical_frame')}"
            )
        except Exception as e:
            self.get_logger().warning(f"Failed to load camera config: {e}, using defaults")

        self._stages = VisionStages(
            self,
            camera_type=camera_config.get("type"),
            camera_frame=camera_config.get("frame"),
            marker_dictionary=camera_config.get("marker_dictionary"),
        )

    def _execute(self, goal_handle):
        """Execute VisionMoveToAction with detect_only support."""
        goal = goal_handle.request
        error = self._stages.run(goal)

        result = VisionMoveToAction.Result()
        if error is not None:
            result.success = False
            result.error_message = error
        else:
            result.success = True
            # Populate detected pose if detect_only was used
            if goal.detect_only and self._stages.last_detected_pose is not None:
                pose = self._stages.last_detected_pose.pose
                result.detected_position = [pose.position.x, pose.position.y, pose.position.z]
                result.detected_orientation = [
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w
                ]

        return result

    def _reset_tf_callback(self, request, response):
        """Handle TF reset service call (after tool exchange)."""
        self._stages.reset_tf()
        response.success = True
        response.message = "TF buffer cleared and listener re-created"
        return response

    def _execute_scan(self, goal_handle):
        """Execute VisionScanAction - batch scan all markers from multiple positions.

        This method scans from multiple robot positions, detects ALL visible
        ArUco markers at each position (with multiple captures per position),
        averages the poses, and caches them for subsequent vision_moveto calls.

        Args:
            goal_handle: Action goal handle containing VisionScanAction.Goal

        Returns:
            VisionScanAction.Result
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
            goal_handle.abort()
            return result

        scan_positions = [flat[i*6:(i+1)*6] for i in range(num_positions)]

        # Get parameters with defaults
        scans_per_position = goal.scans_per_position if goal.scans_per_position > 0 else 3
        timeout = goal.timeout if goal.timeout > 0 else 10.0

        self.get_logger().info(
            f"VisionScan: {num_positions} positions × {scans_per_position} scans"
        )

        # Run the batch scan (populates _stages._tag_pose_cache)
        tags_detected = self._stages.scan_all_tags(
            scan_positions=scan_positions,
            scans_per_position=scans_per_position,
            timeout=timeout,
            settle_time=0.3  # Default settle time after each move
        )

        # Build result
        result = VisionScanAction.Result()
        result.success = tags_detected > 0
        result.tags_detected = tags_detected
        result.error_message = "" if result.success else "No tags detected"

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result


def main(args=None):
    run_server(VisionActionServer, args)


if __name__ == '__main__':
    main()
