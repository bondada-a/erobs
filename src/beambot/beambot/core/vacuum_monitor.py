"""Vacuum monitor — detects object drops during ePick transport.

Arms when vacuum_on is sent, disarms on vacuum_off. A background
subscription on /object_detection_status watches for NO_OBJECT_DETECTED
while armed, and flags the drop so the orchestrator can abort before
the next motion step.

epick_msgs is imported lazily so beamlines without ePick installed can
still load this module. If the import fails, VacuumMonitor instantiates
in a permanently-disarmed state and never subscribes — the orchestrator
unconditionally builds a monitor, but only ePick-using beamlines have
end_effector tasks that would arm it anyway.
"""

from typing import Any

from rclpy.node import Node


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

        # Subscription DISABLED for performance.
        #
        # /object_detection_status arrives at a high rate (~250 Hz) and each
        # message grabs the GIL in this GIL-bound Python process, contending
        # with the goal-execution thread. The vacuum-loss ABORT was already
        # disabled in the orchestrator (commit 88b3d4e — sequences continue
        # regardless of drop detection), so this subscription fed nothing
        # actionable. Leaving it active was pure GIL overhead.
        #
        # arm/disarm bookkeeping (update_after_tasks) and check_lost() still
        # work; without the subscription self.lost never flips to True, which
        # matches the current "no abort on drop" behavior. To restore live drop
        # detection, set _SUBSCRIBE = True below AND re-enable the abort checks
        # in the orchestrator.
        _SUBSCRIBE = False

        if not _SUBSCRIBE:
            self._sub = None
            return

        try:
            from epick_msgs.msg import ObjectDetectionStatus
        except ImportError:
            self._sub = None
            self._logger.info(
                "epick_msgs not available — vacuum monitor inactive "
                "(non-ePick beamline)"
            )
            return

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
        """Arm/disarm the monitor based on grasp/release actions in executed tasks.

        State names are sourced from the active beamline YAML's
        grippers.epick.states block — they must match the SRDF group_state
        names exactly (e.g. "vacuum_on"/"vacuum_off" for the stock CMS SRDF,
        or whatever the local SRDF declares). If the YAML doesn't declare
        them, the monitor refuses to run — silent mismatch would mean no
        drop detection during transport.
        """
        if current_gripper != "epick":
            return

        states = self._grippers.get("epick", {}).get("states", {})
        grasp_state = states.get("grasp")
        release_state = states.get("release")
        if not grasp_state or not release_state:
            self._logger.error(
                "Vacuum monitor disabled: grippers.epick.states.grasp/release "
                "must be declared in the active beamline YAML and match the "
                "SRDF group_state names. Drop detection is OFF until fixed."
            )
            return

        for task in executed_tasks:
            if task.get("task_type") != "end_effector":
                continue
            action = task.get("end_effector_action", "")
            if action == grasp_state:
                self.lost = False
                self.armed = True
                self._logger.info(f"Vacuum monitor ARMED ({grasp_state} detected)")
                if self.status == _NO_OBJECT:
                    self.lost = True
                    self._logger.warning(
                        f"VACUUM_LOST: no seal detected immediately after {grasp_state}"
                    )
            elif action == release_state:
                self.armed = False
                self.lost = False
                self._logger.info(f"Vacuum monitor DISARMED ({release_state} detected)")

    def check_lost(self) -> str | None:
        """Check if vacuum was lost. Returns error string if lost, None if OK."""
        if not self.armed or not self.lost:
            return None
        self.armed = False
        return (
            "VACUUM_LOST: object dropped — ePick reports NO_OBJECT_DETECTED "
            "while vacuum was active. Send vacuum_off then vacuum_on to retry."
        )
