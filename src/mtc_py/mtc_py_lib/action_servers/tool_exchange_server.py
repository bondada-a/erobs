#!/usr/bin/env python3
"""ToolExchangeAction server - handles tool load/dock via MTC."""

import rclpy

from mtc_py_lib.action_servers.base_action_server import BaseActionServer
from mtc_py_lib.stages.tool_exchange_stages import ToolExchangeStages
from mtc_py.action import ToolExchangeAction  # Generated interface - stays mtc_py


class ToolExchangeActionServer(BaseActionServer):
    """Action server for tool exchange operations.

    Handles:
    - Load: Pick up a tool from a magnetic dock
    - Dock: Store a tool on a magnetic dock
    """

    def __init__(self):
        """Initialize the ToolExchange action server."""
        super().__init__(
            node_name="mtc_toolexchange_server_py",
            action_name="mtc_toolexchange_py",
            action_type=ToolExchangeAction,
        )
        self.initialize_stages()

    def initialize_stages(self):
        """Create ToolExchangeStages instance."""
        self._stages = ToolExchangeStages(self)

    def _execute(self, goal_handle):
        """Execute ToolExchange goal.

        Args:
            goal_handle: The goal handle with ToolExchangeAction.Goal

        Returns:
            ToolExchangeAction.Result with success and error_message
        """
        result = ToolExchangeAction.Result()
        goal = goal_handle.request

        if self._stages is None:
            result.success = False
            result.error_message = "Stages not initialized"
            return result

        # Log the operation
        self.get_logger().info(
            f"Executing tool exchange: operation={goal.operation}, "
            f"gripper={goal.gripper}, dock={goal.dock_number}"
        )

        result.success = self._stages.run(goal)
        if not result.success:
            result.error_message = "Tool exchange motion planning or execution failed"

        return result


def main(args=None):
    """Run the ToolExchange action server."""
    rclpy.init(args=args)
    node = ToolExchangeActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ToolExchange server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
