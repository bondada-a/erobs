"""Payload and controller readiness checks that do not need a robot."""

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

import beambot.core.moveit_lifecycle_manager as lifecycle
from beambot.core.moveit_lifecycle_manager import MoveItLifecycleManager


VALID_CONFIG = {
    "moveit_package": "test_moveit",
    "payload_mass": 1.5,
    "payload_cog": {"x": 0.01, "y": -0.02, "z": 0.03},
}


def _manager(*, mock_hardware=False):
    manager = MoveItLifecycleManager.__new__(MoveItLifecycleManager)
    manager._logger = MagicMock()
    manager._node = MagicMock()
    manager._callback_group = MagicMock()
    manager._moveit_process = None
    manager._current_gripper = ""
    manager._current_voltage = 0
    manager._use_mock_hardware = mock_hardware
    manager._enable_joystick = False
    manager._robot_ip = "192.0.2.1"
    manager.cup_override = ""
    return manager


def _attempt_manager(*, mock_hardware=False, config=None):
    manager = _manager(mock_hardware=mock_hardware)
    manager._grippers = {"epick": config or VALID_CONFIG}
    manager._wait_for_moveit_ready = MagicMock(return_value=True)
    manager._load_collision_obstacles = MagicMock(return_value=True)
    manager._restart_external_control = MagicMock(return_value=True)
    manager._wait_for_required_controllers = MagicMock(return_value=True)
    manager._verify_hardware_connected = MagicMock(return_value=True)
    return manager


def test_valid_payload_configuration():
    assert _manager()._validate_payload(VALID_CONFIG)


@pytest.mark.parametrize(
    "config",
    [
        {},
        {"payload_mass": 1.5},
        {"payload_cog": VALID_CONFIG["payload_cog"]},
        {**VALID_CONFIG, "payload_mass": "heavy"},
        {**VALID_CONFIG, "payload_mass": float("nan")},
        {**VALID_CONFIG, "payload_mass": float("inf")},
        {**VALID_CONFIG, "payload_mass": 0},
        {**VALID_CONFIG, "payload_mass": -1},
        {**VALID_CONFIG, "payload_cog": {"x": 0.0, "y": 0.0}},
        {**VALID_CONFIG, "payload_cog": {"x": "left", "y": 0.0, "z": 0.0}},
        {**VALID_CONFIG, "payload_cog": {"x": 0.0, "y": float("nan"), "z": 0.0}},
        {**VALID_CONFIG, "payload_cog": {"x": 0.0, "y": 0.0, "z": float("inf")}},
    ],
)
def test_invalid_payload_configuration(config):
    assert not _manager()._validate_payload(config)


def _payload_call():
    manager = _manager()
    client = manager._node.create_client.return_value
    client.wait_for_service.return_value = True
    future = client.call_async.return_value
    future.done.return_value = True
    return manager, client, future


def test_payload_service_must_be_available():
    manager, client, _ = _payload_call()
    client.wait_for_service.return_value = False

    assert not manager._set_payload(VALID_CONFIG)
    manager._node.destroy_client.assert_called_once_with(client)


@pytest.mark.parametrize("failure", ["client", "call"])
def test_payload_client_exceptions_fail(failure):
    manager, client, _ = _payload_call()
    if failure == "client":
        manager._node.create_client.side_effect = RuntimeError("client failed")
    else:
        client.call_async.side_effect = RuntimeError("call failed")

    assert not manager._set_payload(VALID_CONFIG)


def test_payload_timeout_fails():
    manager, _, future = _payload_call()
    future.done.return_value = False

    with patch.object(lifecycle.time, "monotonic", side_effect=[0.0, 21.0]):
        assert not manager._set_payload(VALID_CONFIG)


@pytest.mark.parametrize(
    ("response", "expected"),
    [
        (None, False),
        (SimpleNamespace(success=False), False),
        (SimpleNamespace(success=True), True),
    ],
)
def test_payload_response_controls_readiness(response, expected):
    manager, client, future = _payload_call()
    future.result.return_value = response

    assert manager._set_payload(VALID_CONFIG) is expected
    manager._node.destroy_client.assert_called_once_with(client)


def test_mock_hardware_validates_without_payload_service():
    manager = _attempt_manager(mock_hardware=True)

    with patch.object(lifecycle.subprocess, "Popen", return_value=MagicMock()):
        assert manager._attempt_launch("epick", "")

    manager._node.create_client.assert_not_called()


def test_mock_hardware_rejects_invalid_payload_before_launch():
    manager = _attempt_manager(mock_hardware=True, config={"moveit_package": "test"})

    with patch.object(lifecycle.subprocess, "Popen") as popen:
        assert not manager._attempt_launch("epick", "")

    popen.assert_not_called()
    manager._node.create_client.assert_not_called()


def test_payload_failure_never_reports_ready():
    manager = _attempt_manager()
    manager._set_payload = MagicMock(return_value=False)

    with patch.object(lifecycle.subprocess, "Popen", return_value=MagicMock()):
        assert not manager._attempt_launch("epick", "")

    assert manager._current_gripper == ""
    assert not any(
        "Robot ready" in str(call) for call in manager._logger.info.call_args_list
    )


def test_inactive_controller_never_reports_ready():
    manager = _attempt_manager()
    manager._wait_for_required_controllers.return_value = False

    with patch.object(lifecycle.subprocess, "Popen", return_value=MagicMock()):
        assert not manager._attempt_launch("epick", "")

    manager._verify_hardware_connected.assert_not_called()


def test_controller_readiness_requires_arm_and_gripper():
    manager = _manager()
    manager._grippers = {
        "hande": {"controller_name": "gripper_action_controller"}
    }
    client = manager._node.create_client.return_value
    client.wait_for_service.return_value = True
    future = client.call_async.return_value
    future.done.return_value = True
    active_arm = SimpleNamespace(
        controller=[
            SimpleNamespace(
                name="scaled_joint_trajectory_controller", state="active"
            )
        ]
    )
    active_all = SimpleNamespace(
        controller=active_arm.controller
        + [SimpleNamespace(name="gripper_action_controller", state="active")]
    )
    future.result.side_effect = [active_arm, active_all]

    with patch.object(lifecycle.time, "sleep"):
        assert manager._wait_for_required_controllers("hande", timeout_sec=1.0)

    assert client.call_async.call_count == 2
    manager._node.destroy_client.assert_called_once_with(client)
