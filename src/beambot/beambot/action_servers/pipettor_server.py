#!/usr/bin/env python3
"""PipettorAction server - handles pipettor operations."""

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.stages.pipettor_stages import PipettorStages
from beambot_interfaces.action import PipettorAction


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
            node_name="beambot_pipettor_server",
            action_name="beambot_pipettor",
            action_type=PipettorAction,
        )

    def initialize_stages(self):
        """Create PipettorStages instance."""
        self._stages = PipettorStages(self)


def main(args=None):
    run_server(PipettorActionServer, args)


if __name__ == '__main__':
    main()
