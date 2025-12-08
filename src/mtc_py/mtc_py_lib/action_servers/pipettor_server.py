#!/usr/bin/env python3
"""PipettorAction server - handles pipettor operations."""

from mtc_py_lib.action_servers.base_action_server import BaseActionServer, run_server
from mtc_py_lib.stages.pipettor_stages import PipettorStages
from mtc_py.action import PipettorAction


class PipettorActionServer(BaseActionServer):
    """Action server for pipettor operations.

    Handles:
    - SUCK: Aspirate liquid
    - EXPEL: Dispense liquid
    - EJECT_TIP: Eject disposable tip
    - SET_LED: Control LED color
    """

    def __init__(self):
        """Initialize the Pipettor action server."""
        super().__init__(
            node_name="mtc_pipettor_server_py",
            action_name="mtc_pipettor_py",
            action_type=PipettorAction,
        )

    def initialize_stages(self):
        """Create PipettorStages instance."""
        self._stages = PipettorStages(self)


def main(args=None):
    run_server(PipettorActionServer, args)


if __name__ == '__main__':
    main()
