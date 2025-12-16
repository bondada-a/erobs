#!/usr/bin/env python3
"""VisionPickPlaceAction server - handles vision-guided pick and place."""

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.stages.vision_pick_place_stages import VisionPickPlaceStages
from beambot_interfaces.action import VisionPickPlaceAction


class VisionPickPlaceActionServer(BaseActionServer):
    """Action server for vision-guided pick and place operations.

    Handles:
    - ArUco marker detection for pick/place targets
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
        """Create VisionPickPlaceStages instance."""
        self._stages = VisionPickPlaceStages(self)


def main(args=None):
    run_server(VisionPickPlaceActionServer, args)


if __name__ == '__main__':
    main()
