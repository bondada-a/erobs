"""Execution controls follow the orchestrator's published state."""

import sys
from types import ModuleType, SimpleNamespace

try:
    from action_msgs.msg import GoalStatus  # noqa: F401
except ImportError:
    action_msgs = ModuleType("action_msgs")
    action_msgs_msg = ModuleType("action_msgs.msg")
    action_msgs_msg.GoalStatus = object
    sys.modules["action_msgs"] = action_msgs
    sys.modules["action_msgs.msg"] = action_msgs_msg

from mtc_gui.main_window import MTCMainWindow


class _Button:
    def setEnabled(self, enabled):
        self.enabled = enabled


def test_execution_state_drives_pause_and_resume():
    window = SimpleNamespace(
        pause_btn=_Button(),
        resume_btn=_Button(),
        _last_goal_was_dry_run=False,
    )

    for state, expected in {
        "RUNNING": (True, False),
        "COMPLETING_TASK": (False, False),
        "PAUSED": (False, True),
        "IDLE": (False, False),
    }.items():
        MTCMainWindow._on_execution_state(window, state)
        assert (window.pause_btn.enabled, window.resume_btn.enabled) == expected

    window._last_goal_was_dry_run = True
    MTCMainWindow._on_execution_state(window, "RUNNING")
    assert not window.pause_btn.enabled
