#!/usr/bin/env python3
"""MoveToAction server - handles MoveTo goals via MTC."""

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.stages.move_to_stages import MoveToStages
from beambot_interfaces.action import MoveToAction


class MoveToActionServer(BaseActionServer):
    """Action server for MoveTo operations.

    Handles:
    - Relative moves (direction + distance)
    - Joint pose moves (from JSON)
    - Named SRDF state moves
    """

    def __init__(self):
        """Initialize the MoveTo action server."""
        super().__init__(
            node_name="beambot_moveto_server",
            action_name="beambot_moveto",
            action_type=MoveToAction,
        )

    def create_stages(self):
        return MoveToStages(self)


def main(args=None):
    run_server(MoveToActionServer, args)


if __name__ == '__main__':
    main()
