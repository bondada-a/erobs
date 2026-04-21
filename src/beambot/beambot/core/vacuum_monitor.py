"""Vacuum monitor — detects object drops during ePick transport.

Arms when vacuum_on is sent, disarms on vacuum_off. A background
subscription on /object_detection_status watches for NO_OBJECT_DETECTED
while armed, and flags the drop so the orchestrator can abort before
the next motion step.
"""

from typing import Any

from rclpy.node import Node
from epick_msgs.msg import ObjectDetectionStatus


# ePick status code for "no object detected"
_NO_OBJECT = 3


class VacuumMonitor:
    """Monitors ePick vacuum grasp and detects object drops."""

    def __init__(self, node: Node, grippers_config: dict[str, Any], callback_group=None):
        self._node = node
        self._logger = node.get_logger()
        self._grippers = grippers_config

        self.armed = False
        self.lost = False
        self.status: 'int | None' = None
        self.last_error = ""

        self._sub = node.create_subscription(
            ObjectDetectionStatus, '/object_detection_status',
            self._on_status, 10,
            callback_group=callback_group,
        )

    def reset(self):
        """Reset state for a new goal."""
        self.armed = False
        self.lost = False

    def _on_status(self, msg):
        """Subscription callback — fires continuously while ePick is active."""
        self.status = int(msg.status)
        if self.armed and self.status == _NO_OBJECT:
            self.lost = True
            self._logger.warning(
                "VACUUM_LOST: object detection status changed to NO_OBJECT_DETECTED "
                "while vacuum is active"
            )

    def update_after_tasks(
        self, executed_tasks: list[dict[str, Any]], current_gripper: str
    ):
        """Arm/disarm monitor based on vacuum_on/off actions in executed tasks."""
        if current_gripper != "epick":
            return

        grasp_state = self._grippers.get("epick", {}).get("states", {}).get("grasp", "vacuum_on")
        release_state = self._grippers.get("epick", {}).get("states", {}).get("release", "vacuum_off")

        for task in executed_tasks:
            if task.get("task_type") != "end_effector":
                continue
            action = task.get("end_effector_action", "")
            if action == grasp_state:
                self.lost = False
                self.armed = True
                self._logger.info("Vacuum monitor ARMED (vacuum_on detected)")
                if self.status == _NO_OBJECT:
                    self.lost = True
                    self._logger.warning(
                        "VACUUM_LOST: no seal detected immediately after vacuum_on"
                    )
            elif action == release_state:
                self.armed = False
                self.lost = False
                self._logger.info("Vacuum monitor DISARMED (vacuum_off detected)")

    def check_lost(self) -> str | None:
        """Check if vacuum was lost. Returns error string if lost, None if OK."""
        if not self.armed or not self.lost:
            return None
        self.armed = False
        return (
            "VACUUM_LOST: object dropped — ePick reports NO_OBJECT_DETECTED "
            "while vacuum was active. Send vacuum_off then vacuum_on to retry."
        )
