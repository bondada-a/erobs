#!/usr/bin/env python3
"""PickPlaceAction server - handles pick and place sequences via MTC."""

import rclpy

from mtc_py_lib.action_servers.base_action_server import BaseActionServer
from mtc_py_lib.stages.pick_place_stages import PickPlaceStages
from mtc_py.action import PickPlaceAction  # Generated interface - stays mtc_py


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
            node_name="mtc_pickplace_server_py",
            action_name="mtc_pickplace_py",
            action_type=PickPlaceAction,
        )
        self.initialize_stages()

    def initialize_stages(self):
        """Create PickPlaceStages instance."""
        self._stages = PickPlaceStages(self)

    def _execute(self, goal_handle):
        """Execute PickPlace goal.

        Args:
            goal_handle: The goal handle with PickPlaceAction.Goal

        Returns:
            PickPlaceAction.Result with success and error_message
        """
        result = PickPlaceAction.Result()
        goal = goal_handle.request

        if self._stages is None:
            result.success = False
            result.error_message = "Stages not initialized"
            return result

        # Log the operation
        self.get_logger().info(
            f"Executing pick/place: gripper={goal.gripper}, "
            f"pick={goal.pick_target}, place={goal.place_target}"
        )

        result.success = self._stages.run(goal)
        if not result.success:
            result.error_message = "Pick/place motion planning or execution failed"

        return result


def main(args=None):
    """Run the PickPlace action server."""
    rclpy.init(args=args)
    node = PickPlaceActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PickPlace server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
