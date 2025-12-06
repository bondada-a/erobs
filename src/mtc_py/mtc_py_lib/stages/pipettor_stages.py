"""PipettorStages - Python equivalent of pipettor_stages.cpp.

Handles pipettor operations:
- SUCK: Aspirate liquid
- EXPEL: Dispense liquid
- EJECT_TIP: Eject the disposable tip
- SET_LED: Control LED color

Note: Unlike other MTC stages, pipettor operations don't move the robot.
We directly call the pipettor action server instead of using MTC.
"""

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

    def run(self, goal) -> bool:
        """Execute Pipettor action.

        Args:
            goal: PipettorAction.Goal with fields:
                - operation: "SUCK", "EXPEL", "EJECT_TIP", "SET_LED"
                - volume_pct: 0.0-1.0 for SUCK/EXPEL
                - led_color: ColorRGBA for SET_LED

        Returns:
            True if successful, False otherwise
        """
        # Format descriptive name for logging
        name = goal.operation
        if goal.operation in ["SUCK", "EXPEL"]:
            name += f" {goal.volume_pct * 100.0:.0f}%"
        elif goal.operation == "SET_LED":
            name += (
                f" ({int(goal.led_color.r * 255)}, "
                f"{int(goal.led_color.g * 255)}, "
                f"{int(goal.led_color.b * 255)})"
            )

        self.logger.info(f"Pipettor: {name}")

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
    ) -> bool:
        """Execute the pipettor action.

        Args:
            operation: Operation type
            volume_pct: Volume percentage (0.0-1.0)
            led_color: LED color for SET_LED

        Returns:
            True if successful, False otherwise
        """
        import rclpy

        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.logger.error("Pipettor action server not available")
            return False

        # Create goal
        action_goal = PipettorOperation.Goal()
        action_goal.operation = operation
        action_goal.volume_pct = volume_pct
        action_goal.led_color = led_color

        self.logger.info(
            f"Sending pipettor operation: {operation} ({volume_pct * 100.0:.0f}%)"
        )

        # Send goal
        send_goal_future = self._action_client.send_goal_async(action_goal)
        rclpy.spin_until_future_complete(
            self.rclpy_node,
            send_goal_future,
            timeout_sec=5.0
        )

        if not send_goal_future.done():
            self.logger.error("Pipettor goal send timeout")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.logger.error("Pipettor goal rejected")
            return False

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
            return False

        result = result_future.result()
        if result.status != 4:  # SUCCEEDED = 4
            self.logger.error(f"Pipettor action failed with status: {result.status}")
            return False

        if not result.result.success:
            self.logger.error(f"Pipettor operation failed: {result.result.message}")
            return False

        self.logger.info(f"Pipettor operation succeeded: {result.result.message}")
        return True

    def suck(self, volume_pct: float = 1.0) -> bool:
        """Aspirate liquid.

        Args:
            volume_pct: Volume percentage (0.0-1.0)

        Returns:
            True if successful
        """
        return self._execute_pipettor_action(
            "SUCK",
            volume_pct,
            ColorRGBA()
        )

    def expel(self, volume_pct: float = 1.0) -> bool:
        """Dispense liquid.

        Args:
            volume_pct: Volume percentage (0.0-1.0)

        Returns:
            True if successful
        """
        return self._execute_pipettor_action(
            "EXPEL",
            volume_pct,
            ColorRGBA()
        )

    def eject_tip(self) -> bool:
        """Eject the disposable tip.

        Returns:
            True if successful
        """
        return self._execute_pipettor_action(
            "EJECT_TIP",
            0.0,
            ColorRGBA()
        )

    def set_led(self, r: float, g: float, b: float) -> bool:
        """Set LED color.

        Args:
            r: Red component (0.0-1.0)
            g: Green component (0.0-1.0)
            b: Blue component (0.0-1.0)

        Returns:
            True if successful
        """
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = 1.0
        return self._execute_pipettor_action(
            "SET_LED",
            0.0,
            color
        )
