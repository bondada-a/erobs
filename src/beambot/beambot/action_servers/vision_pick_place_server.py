#!/usr/bin/env python3
"""VisionPickPlaceAction server - handles vision-guided pick and place."""

import yaml
from ament_index_python.packages import get_package_share_directory

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.stages.vision_pick_place_stages import VisionPickPlaceStages
from beambot_interfaces.action import VisionPickPlaceAction


class VisionPickPlaceActionServer(BaseActionServer):
    """Action server for vision-guided pick and place operations.

    Handles:
    - ArUco marker detection for pick/place targets (configurable camera)
    - Grasp pose computation with offsets
    - Full pick sequence with gripper operations
    """

    def __init__(self):
        """Initialize the VisionPickPlace action server."""
        super().__init__(
            node_name="beambot_vision_pickplace_server",
            action_name="beambot_vision_pickplace",
            action_type=VisionPickPlaceAction,
        )

    def initialize_stages(self):
        """Create VisionPickPlaceStages instance with camera config from beamline config."""
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
            self.get_logger().warn(f"Failed to load camera config: {e}, using defaults")

        self._stages = VisionPickPlaceStages(
            self,
            camera_type=camera_config.get("type"),
            camera_frame=camera_config.get("frame"),
            marker_dictionary=camera_config.get("marker_dictionary"),
        )


def main(args=None):
    run_server(VisionPickPlaceActionServer, args)


if __name__ == '__main__':
    main()
