"""Coordinate physical tool exchange and dependent runtime state."""

import time
from collections.abc import Callable, Mapping
from typing import Any

from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from beambot.stages.base_stages import wait_for_future
from beambot_interfaces.action import ToolExchangeAction


class ToolExchangeManager:
    """Run tool exchange, then synchronize MoveIt and vision state."""

    def __init__(
        self,
        node: Node,
        grippers: Mapping[str, Any],
        moveit_manager,
        callback_group,
        *,
        use_mock_hardware: bool,
        send_and_wait: Callable[..., bool],
        on_gripper_changed: Callable[[str], None],
    ):
        self._node = node
        self._logger = node.get_logger()
        self._grippers = grippers
        self._moveit_manager = moveit_manager
        self._callback_group = callback_group
        self._use_mock_hardware = use_mock_hardware
        self._send_and_wait = send_and_wait
        self._on_gripper_changed = on_gripper_changed
        self.last_error = ""

        self._action_client = ActionClient(
            node,
            ToolExchangeAction,
            "beambot_toolexchange",
            callback_group=callback_group,
        )
        self._vision_reset_tf_clients = (
            (
                "vision",
                node.create_client(
                    Trigger,
                    "beambot_vision_reset_tf",
                    callback_group=callback_group,
                ),
            ),
            (
                "vision_task",
                node.create_client(
                    Trigger,
                    "beambot_vision_task_reset_tf",
                    callback_group=callback_group,
                ),
            ),
        )

    def exchange(
        self,
        step: dict[str, Any],
        poses_json: str,
        current_gripper: str,
        timeout: float,
    ) -> bool:
        """Execute one validated tool exchange."""
        self.last_error = ""
        operation = step.get("operation", "")
        if operation not in {"dock", "load"}:
            self.last_error = (
                f"Unknown tool exchange operation: '{operation}' "
                "(expected 'load' or 'dock')"
            )
            self._logger.error(self.last_error)
            return False

        new_gripper = "none"
        if operation == "load":
            new_gripper = step.get("gripper", "")
            if new_gripper not in self._grippers:
                self.last_error = (
                    f"Unknown gripper '{new_gripper}' in tool_exchange "
                    f"(available: {', '.join(self._grippers)})"
                )
                self._logger.error(self.last_error)
                return False

        if operation == "dock" and not self._use_mock_hardware:
            self._logger.info("Setting tool voltage to 0V for dock (QC release)")
            if not self._set_tool_voltage_via_io(0):
                self.last_error = (
                    "Failed to release quick changer: tool voltage remains on"
                )
                return False
            self._moveit_manager.notify_voltage_change(0)
            time.sleep(0.5)

        goal = ToolExchangeAction.Goal()
        goal.operation = operation
        goal.gripper = step.get("gripper", "")
        goal.current_attached_gripper = current_gripper
        goal.dock_number = int(step.get("dock_number", 0))
        goal.approach_pose = step.get("approach_pose", "")
        goal.poses_json = poses_json

        if not self._send_and_wait(
            self._action_client,
            goal,
            "tool_exchange",
            timeout,
        ):
            return False

        if new_gripper == current_gripper:
            return True

        self._logger.info(
            f"Gripper changed: {current_gripper} → {new_gripper}, restarting MoveIt"
        )
        self._on_gripper_changed(new_gripper)
        current_gripper = new_gripper

        if not self._moveit_manager.launch_moveit_with_gripper(new_gripper):
            self.last_error = (
                "Failed to restart MoveIt after tool exchange "
                f"({current_gripper} → {new_gripper})"
            )
            self._logger.error(self.last_error)
            return False

        self._reset_vision_tf()
        return True

    def _set_tool_voltage_via_io(self, voltage: int) -> bool:
        """Set tool voltage without stopping the external-control program."""
        from ur_msgs.srv import SetIO

        client = self._node.create_client(
            SetIO,
            "/io_and_status_controller/set_io",
            callback_group=self._callback_group,
        )
        if not client.wait_for_service(timeout_sec=3.0):
            self._logger.error("set_io service not available")
            self._node.destroy_client(client)
            return False

        request = SetIO.Request()
        request.fun = SetIO.Request.FUN_SET_TOOL_VOLTAGE
        request.pin = 0
        request.state = float(voltage)

        future = client.call_async(request)
        done = wait_for_future(future, timeout=5.0, poll_interval=0.05)

        self._node.destroy_client(client)
        if done and future.result().success:
            self._logger.info(f"Tool voltage set to {voltage}V via set_io")
            return True
        self._logger.error(f"Failed to set tool voltage to {voltage}V")
        return False

    def _reset_vision_tf(self):
        """Clear stale gripper transforms after MoveIt changes URDF."""
        for name, client in self._vision_reset_tf_clients:
            if not client.wait_for_service(timeout_sec=3.0):
                self._logger.warning(
                    f"{name} TF reset service not available — "
                    "server may detect wrong gripper frame"
                )
                continue
            future = client.call_async(Trigger.Request())
            done = wait_for_future(future, timeout=5.0, poll_interval=0.05)
            if done and future.result().success:
                self._logger.info(f"{name} server TF buffer reset")
            else:
                self._logger.warning(f"{name} TF reset did not complete")
