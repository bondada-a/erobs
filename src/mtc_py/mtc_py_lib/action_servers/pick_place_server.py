#!/usr/bin/env python3
"""PickPlaceAction server - handles pick and place sequences via MTC."""

from mtc_py_lib.action_servers.base_action_server import BaseActionServer, run_server
from mtc_py_lib.stages.pick_place_stages import PickPlaceStages
from mtc_py.action import PickPlaceAction


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
        """Execute PickPlace goal with logging."""
        goal = goal_handle.request
        self.get_logger().info(
            f"Executing pick/place: gripper={goal.gripper}, "
            f"pick={goal.pick_target}, place={goal.place_target}"
        )
        return super()._execute(goal_handle)

    def _get_failure_message(self) -> str:
        """Custom error message for pick/place failures."""
        return "Pick/place motion planning or execution failed"


def main(args=None):
    run_server(PickPlaceActionServer, args)


if __name__ == '__main__':
    main()
