#!/usr/bin/env python3
"""VisionMoveToAction server - handles vision-guided movement via ArUco markers."""

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.stages.vision_stages import VisionStages
from beambot_interfaces.action import VisionMoveToAction


class VisionActionServer(BaseActionServer):
    """Action server for vision-guided MoveTo operations.

    Handles:
    - ArUco marker detection via Zivid camera
    - TF transform to robot base frame
    - Motion to detected marker pose
    """

    def __init__(self):
        """Initialize the Vision action server."""
        super().__init__(
            node_name="beambot_vision_server",
            action_name="beambot_vision_moveto",
            action_type=VisionMoveToAction,
        )

    def initialize_stages(self):
        """Create VisionStages instance."""
        self._stages = VisionStages(self)


def main(args=None):
    run_server(VisionActionServer, args)


if __name__ == '__main__':
    main()
