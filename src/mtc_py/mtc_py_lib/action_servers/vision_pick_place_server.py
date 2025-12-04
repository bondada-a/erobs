#!/usr/bin/env python3
"""VisionPickPlaceAction server - handles vision-guided pick and place."""

from mtc_py_lib.action_servers.base_action_server import BaseActionServer, run_server
from mtc_py_lib.stages.vision_pick_place_stages import VisionPickPlaceStages
from mtc_py.action import VisionPickPlaceAction


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
            node_name="mtc_vision_pickplace_server_py",
            action_name="mtc_vision_pickplace_py",
            action_type=VisionPickPlaceAction,
        )
        self.initialize_stages()

    def initialize_stages(self):
        """Create VisionPickPlaceStages instance."""
        self._stages = VisionPickPlaceStages(self)

    def _execute(self, goal_handle):
        """Execute VisionPickPlace goal with logging."""
        goal = goal_handle.request
        self.get_logger().info(
            f"Executing vision pick/place: pick_tag={goal.pick_tag_id}, "
            f"place_tag={goal.place_tag_id}, gripper={goal.gripper}"
        )
        return super()._execute(goal_handle)

    def _get_failure_message(self) -> str:
        """Custom error message for vision pick/place failures."""
        return "Vision-guided pick/place failed"


def main(args=None):
    run_server(VisionPickPlaceActionServer, args)


if __name__ == '__main__':
    main()
