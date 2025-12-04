#!/usr/bin/env python3
"""MoveToAction server - handles MoveTo goals via MTC."""

import rclpy

from mtc_py_lib.action_servers.base_action_server import BaseActionServer
from mtc_py_lib.stages.move_to_stages import MoveToStages
from mtc_py.action import MoveToAction  # Generated interface - stays mtc_py


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
        self.initialize_stages()

    def initialize_stages(self):
        """Create MoveToStages instance."""
        self._stages = MoveToStages(self)

    def _execute(self, goal_handle):
        """Execute MoveTo goal.

        Args:
            goal_handle: The goal handle with MoveToAction.Goal

        Returns:
            MoveToAction.Result with success and error_message
        """
        result = MoveToAction.Result()
        goal = goal_handle.request

        if self._stages is None:
            result.success = False
            result.error_message = "Stages not initialized"
            return result

        try:
            result.success = self._stages.run(goal)
            if not result.success:
                result.error_message = "Motion planning or execution failed"
        except Exception as e:
            result.success = False
            result.error_message = str(e)
            self.get_logger().error(f"MoveTo execution error: {e}")

        return result


def main(args=None):
    """Run the MoveTo action server."""
    rclpy.init(args=args)
    node = MoveToActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MoveTo server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
