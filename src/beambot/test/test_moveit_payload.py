"""Hardware-setup checks that do not need a robot."""

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

import beambot.utils.moveit_lifecycle_manager as lifecycle
from beambot.utils.moveit_lifecycle_manager import MoveItLifecycleManager


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
    manager._model_revision = ""
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
    manager._verify_hardware_connected = MagicMock(return_value=True)
    return manager


def _payload_call():
    manager = _manager()
    client = manager._node.create_client.return_value
    client.wait_for_service.return_value = True
    return manager, client


def test_payload_service_must_be_available():
    manager, client = _payload_call()
    client.wait_for_service.return_value = False

    assert not manager._set_payload(VALID_CONFIG)
    manager._node.destroy_client.assert_called_once_with(client)


@pytest.mark.parametrize("failure", ["client", "call"])
def test_payload_client_exceptions_fail(failure):
    manager, client = _payload_call()
    if failure == "client":
        manager._node.create_client.side_effect = RuntimeError("client failed")
    else:
        client.call.side_effect = RuntimeError("call failed")

    assert not manager._set_payload(VALID_CONFIG)


def test_payload_timeout_fails():
    manager, client = _payload_call()
    client.call.return_value = None  # Client.call returns None on timeout.

    assert not manager._set_payload(VALID_CONFIG)
    assert client.call.call_args.kwargs == {"timeout_sec": 10.0}


@pytest.mark.parametrize(
    ("response", "expected"),
    [
        (None, False),
        (SimpleNamespace(success=False), False),
        (SimpleNamespace(success=True), True),
    ],
)
def test_payload_response_controls_readiness(response, expected):
    manager, client = _payload_call()
    client.call.return_value = response

    assert manager._set_payload(VALID_CONFIG) is expected
    manager._node.destroy_client.assert_called_once_with(client)


def test_mock_hardware_launches_without_payload_service():
    manager = _attempt_manager(mock_hardware=True)

    with patch.object(lifecycle.subprocess, "Popen", return_value=MagicMock()):
        assert manager._attempt_launch("epick", "")

    manager._node.create_client.assert_not_called()


def test_payload_failure_never_reports_ready():
    manager = _attempt_manager()
    manager._set_payload = MagicMock(return_value=False)

    with patch.object(lifecycle.subprocess, "Popen", return_value=MagicMock()):
        assert not manager._attempt_launch("epick", "")

    assert manager._model_revision == ""
    assert not any(
        "Robot ready" in str(call) for call in manager._logger.info.call_args_list
    )


@pytest.mark.parametrize("failure", [None, "connect", "sendall"])
def test_tool_voltage_closes_socket_on_success_and_failure(failure):
    manager = _manager()
    sock = MagicMock()
    sock.__enter__.return_value = sock
    if failure:
        getattr(sock, failure).side_effect = OSError("connection failed")

    with patch.object(lifecycle.socket, "socket", return_value=sock) as factory:
        assert manager._set_tool_voltage(24) is (failure is None)

    factory.assert_called_once_with(lifecycle.socket.AF_INET, lifecycle.socket.SOCK_STREAM)
    sock.settimeout.assert_called_once_with(manager.SOCKET_TIMEOUT)
    sock.connect.assert_called_once_with((manager._robot_ip, manager.UR_SECONDARY_PORT))
    if failure != "connect":
        sock.sendall.assert_called_once_with(b"set_tool_voltage(24)\n")
    else:
        sock.sendall.assert_not_called()
    sock.__exit__.assert_called_once()


@pytest.mark.parametrize("position,expected", [(1.0, True), (0.0, False), (None, False)])
def test_hardware_readiness_uses_monotonic_deadline(position, expected):
    manager = _manager()
    manager._arm_joint_names = tuple(f"joint_{i}" for i in range(6))
    manager._joint_positions = {name: 1.0 for name in manager._arm_joint_names}

    def receive_joints(_delay):
        if position is not None:
            manager._joint_positions.update({f"joint_{i}": position for i in range(6)})

    with (
        patch.object(lifecycle.time, "time", side_effect=AssertionError("wall clock used")),
        patch.object(lifecycle.time, "monotonic", side_effect=[0.0, 0.0, 0.1, 1.0]),
        patch.object(lifecycle.time, "sleep", side_effect=receive_joints),
    ):
        assert manager._verify_hardware_connected(timeout_sec=1.0) is expected


@pytest.mark.parametrize(("program_running", "expected"), [(True, True), (False, False)])
def test_external_control_waits_for_program_running_signal(program_running, expected):
    manager = _manager()
    manager._robot_program_running = lifecycle.threading.Event()
    callbacks = []
    manager._node.create_subscription.side_effect = (
        lambda _type, _topic, callback, *_a, **_k: callbacks.append(callback)
    )
    client = manager._node.create_client.return_value
    client.wait_for_service.return_value = True

    def play(_request, timeout_sec):
        # The driver reports its program state after accepting /play.
        callbacks[0](SimpleNamespace(data=program_running))
        return SimpleNamespace(success=True, message="")

    client.call.side_effect = play

    # Report the signal without blocking for the real 5 s timeout.
    signal = manager._robot_program_running
    with patch.object(signal, "wait", side_effect=lambda _timeout: signal.is_set()):
        assert manager._restart_external_control() is expected
    manager._node.destroy_subscription.assert_called_once()
