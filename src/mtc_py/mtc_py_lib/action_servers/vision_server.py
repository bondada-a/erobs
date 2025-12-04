#!/usr/bin/env python3
"""VisionMoveToAction server - handles vision-guided movement via ArUco markers."""

from mtc_py_lib.action_servers.base_action_server import BaseActionServer, run_server
from mtc_py_lib.stages.vision_stages import VisionStages
from mtc_py.action import VisionMoveToAction


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
            node_name="mtc_vision_server_py",
            action_name="mtc_vision_moveto_py",
            action_type=VisionMoveToAction,
        )
        self.initialize_stages()

    def initialize_stages(self):
        """Create VisionStages instance."""
        self._stages = VisionStages(self)

    def _get_failure_message(self) -> str:
        """Custom error message for vision failures."""
        return "Vision-guided motion failed"


def main(args=None):
    run_server(VisionActionServer, args)


if __name__ == '__main__':
    main()
