#!/usr/bin/env python3
"""MoveToAction server - handles MoveTo goals via MTC."""

from mtc_py_lib.action_servers.base_action_server import BaseActionServer, run_server
from mtc_py_lib.stages.move_to_stages import MoveToStages
from mtc_interfaces.action import MoveToAction


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
            node_name="mtc_moveto_server_py",
            action_name="mtc_moveto_py",
            action_type=MoveToAction,
        )

    def initialize_stages(self):
        """Create MoveToStages instance."""
        self._stages = MoveToStages(self)


def main(args=None):
    run_server(MoveToActionServer, args)


if __name__ == '__main__':
    main()
