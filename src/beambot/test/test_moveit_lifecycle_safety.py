"""Transactional MoveIt lifecycle checks that do not need a robot."""

import signal
import subprocess
from unittest.mock import MagicMock, call, patch

import pytest

import beambot.core.moveit_lifecycle_manager as lifecycle
from beambot.core.moveit_lifecycle_manager import MoveItLifecycleManager


def _manager(*, mock_hardware=True):
    manager = MoveItLifecycleManager.__new__(MoveItLifecycleManager)
    manager._logger = MagicMock()
    manager._moveit_process = None
    manager._current_gripper = ""
    manager._use_mock_hardware = mock_hardware
    return manager


def _attempt_manager():
    manager = _manager(mock_hardware=False)
    manager._grippers = {"epick": {"moveit_package": "test_moveit", "tool_voltage": 0}}
    manager._robot_ip = "192.0.2.1"
    manager._enable_joystick = False
    manager._current_voltage = 0
    manager.cup_override = ""
    manager._wait_for_moveit_ready = MagicMock(return_value=True)
    manager._load_collision_obstacles = MagicMock(return_value=True)
    manager._restart_external_control = MagicMock(return_value=True)
    manager._verify_hardware_connected = MagicMock(return_value=True)
    manager._set_payload = MagicMock()
    manager.kill_current_process = MagicMock(return_value=True)
    return manager


def test_first_failure_is_cleaned_before_successful_retry():
    manager = _manager(mock_hardware=False)
    manager._attempt_launch = MagicMock(side_effect=[False, True])
    manager.kill_current_process = MagicMock(return_value=True)

    assert manager.launch_moveit_with_gripper("epick")
    assert manager._attempt_launch.call_count == 2
    manager.kill_current_process.assert_called_once_with()


def test_final_retry_failure_is_cleaned():
    manager = _manager(mock_hardware=False)
    manager._attempt_launch = MagicMock(return_value=False)
    manager.kill_current_process = MagicMock(return_value=True)

    assert not manager.launch_moveit_with_gripper("epick")
    assert manager._attempt_launch.call_count == 2
    assert manager.kill_current_process.call_count == 2


def test_mock_hardware_failure_is_cleaned_without_retry():
    manager = _manager()
    manager._attempt_launch = MagicMock(return_value=False)
    manager.kill_current_process = MagicMock(return_value=True)

    assert not manager.launch_moveit_with_gripper("epick")
    manager._attempt_launch.assert_called_once_with("epick")
    manager.kill_current_process.assert_called_once_with()


def test_incomplete_cleanup_blocks_retry():
    manager = _manager(mock_hardware=False)
    manager._attempt_launch = MagicMock(return_value=False)
    manager.kill_current_process = MagicMock(return_value=False)

    assert not manager.launch_moveit_with_gripper("epick")
    manager._attempt_launch.assert_called_once_with("epick")


@pytest.mark.parametrize(
    "stage",
    [
        "_set_tool_voltage",
        "_wait_for_moveit_ready",
        "_load_collision_obstacles",
        "_restart_external_control",
        "_verify_hardware_connected",
    ],
)
@pytest.mark.parametrize("raises", [False, True], ids=["false", "exception"])
def test_each_startup_stage_failure_rolls_back_both_attempts(stage, raises):
    manager = _attempt_manager()
    if stage == "_set_tool_voltage":
        manager._current_voltage = None
        manager._grippers["epick"]["tool_voltage"] = 24
    failed_stage = MagicMock(
        side_effect=RuntimeError("stage failed") if raises else None,
        return_value=False,
    )
    setattr(manager, stage, failed_stage)

    with patch.object(lifecycle.subprocess, "Popen", return_value=MagicMock()):
        assert not manager.launch_moveit_with_gripper("epick")

    assert failed_stage.call_count == 2
    assert manager.kill_current_process.call_count == 2


def test_payload_exception_rolls_back_both_attempts():
    manager = _attempt_manager()
    manager._set_payload.side_effect = RuntimeError("payload failed")

    with patch.object(lifecycle.subprocess, "Popen", return_value=MagicMock()):
        assert not manager.launch_moveit_with_gripper("epick")

    assert manager._set_payload.call_count == 2
    assert manager.kill_current_process.call_count == 2


def test_switch_does_not_launch_when_existing_process_cannot_be_cleaned():
    manager = _manager()
    manager._moveit_process = MagicMock()
    manager._current_gripper = "hand_e"
    manager._attempt_launch = MagicMock()
    manager.kill_current_process = MagicMock(return_value=False)

    assert not manager.launch_moveit_with_gripper("epick")
    manager._attempt_launch.assert_not_called()


def _teardown_manager(process):
    manager = _manager()
    manager._moveit_process = process
    manager._current_gripper = "epick"
    manager._drain_stale_execute_trajectory = MagicMock(return_value=True)
    return manager


def test_teardown_terminates_and_reaps_the_process_group():
    process = MagicMock(pid=1234)
    manager = _teardown_manager(process)

    with patch.object(lifecycle.os, "killpg") as killpg:
        assert manager.kill_current_process()

    killpg.assert_called_once_with(1234, signal.SIGTERM)
    process.wait.assert_called_once_with(timeout=3.0)
    manager._drain_stale_execute_trajectory.assert_called_once_with()
    assert manager._moveit_process is None
    assert manager._current_gripper == ""


def test_teardown_escalates_to_sigkill_after_timeout():
    process = MagicMock(pid=1234)
    process.wait.side_effect = [subprocess.TimeoutExpired("ros2 launch", 3), None]
    manager = _teardown_manager(process)

    with patch.object(lifecycle.os, "killpg") as killpg:
        assert manager.kill_current_process()

    assert killpg.call_args_list == [
        call(1234, signal.SIGTERM),
        call(1234, signal.SIGKILL),
    ]
    assert process.wait.call_args_list == [call(timeout=3.0), call()]


@pytest.mark.parametrize("failure", ["signal", "wait"])
def test_teardown_failure_clears_state_and_fails_closed(failure):
    process = MagicMock(pid=1234)
    manager = _teardown_manager(process)
    if failure == "wait":
        process.wait.side_effect = OSError("wait failed")

    with patch.object(lifecycle.os, "killpg") as killpg:
        if failure == "signal":
            killpg.side_effect = OSError("signal failed")
        assert not manager.kill_current_process()

    manager._drain_stale_execute_trajectory.assert_not_called()
    assert manager._moveit_process is None
    assert manager._current_gripper == ""


def test_drain_reports_when_stale_server_disappears():
    manager = _manager()
    manager._node = MagicMock()
    manager._callback_group = MagicMock()
    probe = MagicMock()
    probe.wait_for_server.return_value = False

    with (
        patch.object(lifecycle, "ActionClient", return_value=probe),
        patch.object(lifecycle.time, "monotonic", side_effect=[0.0, 0.0]),
    ):
        assert manager._drain_stale_execute_trajectory()

    probe.destroy.assert_called_once_with()


def test_drain_blocks_relaunch_while_stale_server_remains():
    manager = _manager()
    manager._node = MagicMock()
    manager._callback_group = MagicMock()
    probe = MagicMock()
    probe.wait_for_server.return_value = True

    with (
        patch.object(lifecycle, "ActionClient", return_value=probe),
        patch.object(lifecycle.time, "monotonic", side_effect=[0.0, 0.0, 11.0]),
    ):
        assert not manager._drain_stale_execute_trajectory()

    probe.destroy.assert_called_once_with()
