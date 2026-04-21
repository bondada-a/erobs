"""PipettorStages - Python equivalent of pipettor_stages.cpp.

Handles pipettor operations:
- SUCK: Aspirate liquid
- EXPEL: Dispense liquid
- EJECT_TIP: Eject the disposable tip
- SET_LED: Control LED color

Note: Unlike other MTC stages, pipettor operations don't move the robot.
We directly call the pipettor action server instead of using MTC.
"""

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import ColorRGBA

from pipette_driver.action import PipettorOperation


class PipettorStages:
    """Handles pipettor operations via action client."""

    # Action server name
    PIPETTOR_ACTION = "pipettor_operation"

    # Default timeout for pipettor operations (seconds)
    DEFAULT_TIMEOUT = 60.0

    def __init__(self, rclpy_node: Node):
        """Initialize PipettorStages.

        Args:
            rclpy_node: ROS node for action client
        """
        self.rclpy_node = rclpy_node
        self.logger = rclpy_node.get_logger()

        # Create action client
        self._action_client = ActionClient(
            rclpy_node,
            PipettorOperation,
            self.PIPETTOR_ACTION
        )

        self.logger.info("PipettorStages initialized")

    def run(self, goal) -> str | None:
        """Execute Pipettor action.

        Args:
            goal: PipettorAction.Goal with fields:
                - operation: "SUCK", "EXPEL", "EJECT_TIP", "SET_LED"
                - volume_pct: 0.0-1.0 for SUCK/EXPEL
                - led_color: ColorRGBA for SET_LED

        Returns:
            None if successful, error string describing failure otherwise
        """
        # Format descriptive log message
        if goal.operation in ["SUCK", "EXPEL"]:
            op_desc = f"{goal.operation} {goal.volume_pct * 100.0:.0f}%"
        elif goal.operation == "SET_LED":
            op_desc = (
                f"SET_LED ({int(goal.led_color.r * 255)}, "
                f"{int(goal.led_color.g * 255)}, "
                f"{int(goal.led_color.b * 255)})"
            )
        else:
            op_desc = goal.operation

        self.logger.info(f"Pipettor: {op_desc}")

        return self._execute_pipettor_action(
            goal.operation,
            goal.volume_pct,
            goal.led_color
        )

    def _execute_pipettor_action(
        self,
        operation: str,
        volume_pct: float,
        led_color: ColorRGBA
    ) -> str | None:
        """Execute the pipettor action.

        Args:
            operation: Operation type
            volume_pct: Volume percentage (0.0-1.0)
            led_color: LED color for SET_LED

        Returns:
            None if successful, error string on failure
        """
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            return f"Pipettor action server '{self.PIPETTOR_ACTION}' not available (timeout 5s)"

        # Create goal
        action_goal = PipettorOperation.Goal()
        action_goal.operation = operation
        action_goal.volume_pct = volume_pct
        action_goal.led_color = led_color

        # Send goal
        send_goal_future = self._action_client.send_goal_async(action_goal)
        rclpy.spin_until_future_complete(
            self.rclpy_node,
            send_goal_future,
            timeout_sec=5.0
        )

        if not send_goal_future.done():
            return f"Pipettor goal send timeout for operation '{operation}'"

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return f"Pipettor goal rejected for operation '{operation}'"

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.rclpy_node,
            result_future,
            timeout_sec=self.DEFAULT_TIMEOUT
        )

        if not result_future.done():
            self.logger.error("Pipettor operation timeout")
            # Try to cancel
            goal_handle.cancel_goal_async()
            return f"TIMEOUT: Pipettor operation '{operation}' timed out after {self.DEFAULT_TIMEOUT}s"

        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return f"Pipettor action failed with status {result.status} for operation '{operation}'"

        if not result.result.success:
            return f"Pipettor operation '{operation}' failed: {result.result.message}"

        self.logger.info(f"Pipettor operation succeeded: {result.result.message}")
        return None
