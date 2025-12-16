#!/usr/bin/env python3
"""ToolExchangeAction server - handles tool load/dock via MTC."""

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.stages.tool_exchange_stages import ToolExchangeStages
from beambot_interfaces.action import ToolExchangeAction


class ToolExchangeActionServer(BaseActionServer):
    """Action server for tool exchange operations.

    Handles:
    - Load: Pick up a tool from a magnetic dock
    - Dock: Store a tool on a magnetic dock
    """

    def __init__(self):
        """Initialize the ToolExchange action server."""
        super().__init__(
            node_name="beambot_toolexchange_server",
            action_name="beambot_toolexchange",
            action_type=ToolExchangeAction,
        )

    def initialize_stages(self):
        """Create ToolExchangeStages instance."""
        self._stages = ToolExchangeStages(self)


def main(args=None):
    run_server(ToolExchangeActionServer, args)


if __name__ == '__main__':
    main()
