#!/usr/bin/env python3
"""EndEffectorAction server - handles gripper commands via MTC."""

from mtc_py_lib.action_servers.base_action_server import BaseActionServer, run_server
from mtc_py_lib.stages.end_effector_stages import EndEffectorStages
from mtc_interfaces.action import EndEffectorAction


class EndEffectorActionServer(BaseActionServer):
    """Action server for EndEffector (gripper) operations.

    Handles gripper open/close commands by moving to SRDF-defined states.
    """

    def __init__(self):
        """Initialize the EndEffector action server."""
        super().__init__(
            node_name="mtc_endeffector_server_py",
            action_name="mtc_endeffector_py",
            action_type=EndEffectorAction,
        )

    def initialize_stages(self):
        """Create EndEffectorStages instance."""
        self._stages = EndEffectorStages(self)


def main(args=None):
    run_server(EndEffectorActionServer, args)


if __name__ == '__main__':
    main()
