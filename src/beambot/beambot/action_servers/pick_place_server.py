#!/usr/bin/env python3
"""PickPlaceAction server - handles pick and place sequences via MTC."""

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.stages.pick_place_stages import PickPlaceStages
from beambot_interfaces.action import PickPlaceAction


class PickPlaceActionServer(BaseActionServer):
    """Action server for pick and place operations.

    Handles complete pick and place sequences:
    - Pick: open gripper, approach, grasp, close, retreat
    - Place: approach, position, open, retreat
    - Return to home position
    """

    def __init__(self):
        """Initialize the PickPlace action server."""
        super().__init__(
            node_name="beambot_pickplace_server",
            action_name="beambot_pickplace",
            action_type=PickPlaceAction,
        )

    def initialize_stages(self):
        """Create PickPlaceStages instance."""
        self._stages = PickPlaceStages(self)


def main(args=None):
    run_server(PickPlaceActionServer, args)


if __name__ == '__main__':
    main()
