#!/usr/bin/env python3
"""EndEffectorAction server - handles gripper commands via MTC."""

import rclpy

from mtc_py_lib.action_servers.base_action_server import BaseActionServer
from mtc_py_lib.stages.end_effector_stages import EndEffectorStages
from mtc_py.action import EndEffectorAction  # Generated interface - stays mtc_py


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
        self.initialize_stages()

    def initialize_stages(self):
        """Create EndEffectorStages instance."""
        self._stages = EndEffectorStages(self)

    def _execute(self, goal_handle):
        """Execute EndEffector goal.

        Args:
            goal_handle: The goal handle with EndEffectorAction.Goal

        Returns:
            EndEffectorAction.Result with success and error_message
        """
        result = EndEffectorAction.Result()
        goal = goal_handle.request

        if self._stages is None:
            result.success = False
            result.error_message = "Stages not initialized"
            return result

        result.success = self._stages.run(goal)
        if not result.success:
            result.error_message = "Gripper operation failed"

        return result


def main(args=None):
    """Run the EndEffector action server."""
    rclpy.init(args=args)
    node = EndEffectorActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down EndEffector server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
