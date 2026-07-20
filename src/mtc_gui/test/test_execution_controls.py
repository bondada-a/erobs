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

from mtc_gui.main_window import (
    MTCMainWindow,
    _build_task_defaults,
    _execution_controls,
    task_summary,
)
from mtc_gui.ros2_bridge import ROS2Bridge


class _Control:
    def setEnabled(self, enabled):
        self.enabled = enabled


class _StepList:
    def set_editing_enabled(self, enabled):
        self.editing_enabled = enabled

    def set_paused(self, paused):
        self.paused = paused


def _window():
    controls = [_Control() for _ in range(11)]
    window = SimpleNamespace(
        exec_btn=controls[0],
        pause_btn=controls[1],
        resume_btn=controls[2],
        stop_btn=controls[3],
        task_toolbar=controls[4],
        up_step_btn=controls[5],
        down_step_btn=controls[6],
        remove_step_btn=controls[7],
        clear_steps_btn=controls[8],
        gripper_combo=controls[9],
        dry_run_check=controls[10],
        step_list=_StepList(),
        _execution_state=None,
        _goal_pending=False,
        _log=lambda message: None,
    )
    window._project_execution_state = (
        lambda: MTCMainWindow._project_execution_state(window)
    )
    return window


def test_pause_step_default_summary_and_preview_boundary():
    assert _build_task_defaults({})["pause"] == {"task_type": "pause"}
    assert task_summary({"task_type": "pause"}) == "Wait for operator to resume"
    assert "pause" not in MTCMainWindow._DRY_RUN_SUPPORTED


def test_execution_state_projects_every_control():
    expected = {
        "IDLE": (True, False, False, False, True, False),
        "RUNNING": (False, True, False, True, False, False),
        "COMPLETING_TASK": (False, False, False, True, False, False),
        "PAUSED": (False, False, True, True, False, True),
        "UNKNOWN": (False, False, False, False, False, False),
    }
    for state, controls in expected.items():
        assert _execution_controls(state) == controls

    assert _execution_controls("IDLE", goal_pending=True) == (
        False, False, False, True, False, False
    )


def test_active_state_clears_pending_goal_and_updates_widgets():
    window = _window()
    window._goal_pending = True

    MTCMainWindow._on_execution_state(window, "IDLE")
    assert window._goal_pending

    MTCMainWindow._on_execution_state(window, "PAUSED")
    assert not window._goal_pending
    assert window.resume_btn.enabled
    assert window.stop_btn.enabled
    assert not window.task_toolbar.enabled
    assert not window.step_list.editing_enabled
    assert window.step_list.paused


def test_bridge_caches_and_forwards_execution_state():
    bridge = ROS2Bridge()
    states = []
    bridge.execution_state_changed.connect(states.append)

    bridge._on_execution_state(SimpleNamespace(data="PAUSED"))

    assert bridge.execution_state == "PAUSED"
    assert states == ["PAUSED"]
