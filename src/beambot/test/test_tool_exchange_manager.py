"""Offline checks for tool-exchange ordering and service failures."""

from types import SimpleNamespace
from unittest.mock import Mock, call

import pytest

from beambot.utils import tool_exchange_manager


@pytest.fixture
def manager(monkeypatch):
    monkeypatch.setattr(tool_exchange_manager, "ActionClient", Mock())
    monkeypatch.setattr(tool_exchange_manager, "time", Mock())
    return tool_exchange_manager.ToolExchangeManager(
        Mock(), {"none": {}, "hande": {}}, Mock(), object(),
        use_mock_hardware=False, send_and_wait=Mock(return_value=True),
        on_gripper_changed=Mock(),
    )


@pytest.mark.parametrize("outcome", [
    "success", "unavailable", "timeout", "rejected", "create_error",
    "service_wait_error", "call_error",
])
def test_voltage_service_result_and_cleanup(outcome, manager):
    client = manager._node.create_client.return_value
    client.wait_for_service.return_value = outcome != "unavailable"
    # Client.call returns None when the reply does not arrive in time.
    client.call.return_value = (
        None if outcome == "timeout" else SimpleNamespace(success=outcome != "rejected")
    )
    operations = {
        "create_error": manager._node.create_client,
        "service_wait_error": client.wait_for_service,
        "call_error": client.call,
    }
    if outcome in operations:
        operations[outcome].side_effect = RuntimeError("service interrupted")

    assert manager._set_tool_voltage_via_io(0) is (outcome == "success")

    if outcome == "create_error":
        manager._node.destroy_client.assert_not_called()
    else:
        manager._node.destroy_client.assert_called_once_with(client)
        client.wait_for_service.assert_called_once_with(timeout_sec=3.0)
    if client.call.called:
        request = client.call.call_args.args[0]
        assert (request.fun, request.pin, request.state) == (4, 0, 0.0)
        assert client.call.call_args.kwargs == {"timeout_sec": 5.0}
    if outcome != "success":
        assert manager._logger.error.called


@pytest.mark.parametrize("outcome", [
    "success", "unavailable", "timeout", "rejected", "service_wait_error", "call_error",
])
def test_vision_reset_reports_failures_without_raising(outcome, manager):
    client = Mock()
    manager._vision_task_reset_tf = client
    client.wait_for_service.return_value = outcome != "unavailable"
    client.call.return_value = (
        None if outcome == "timeout" else SimpleNamespace(success=outcome != "rejected")
    )
    operations = {
        "service_wait_error": client.wait_for_service,
        "call_error": client.call,
    }
    if outcome in operations:
        operations[outcome].side_effect = RuntimeError("service interrupted")

    manager._reset_vision_tf()

    client.wait_for_service.assert_called_once_with(timeout_sec=3.0)
    assert manager._logger.warning.called is (outcome != "success")
    if outcome == "success":
        manager._logger.info.assert_any_call("vision_task server TF buffer reset")


@pytest.mark.parametrize("operation", ["load", "dock"])
@pytest.mark.parametrize("outcome", ["success", "action_failure", "restart_failure"])
@pytest.mark.parametrize("mock_hardware", [False, True])
def test_exchange_preserves_goal_and_operation_order(operation, outcome, mock_hardware, manager):
    manager._use_mock_hardware = mock_hardware
    current, new = ("none", "hande") if operation == "load" else ("hande", "none")
    events = Mock()
    manager._set_tool_voltage_via_io = Mock(return_value=True)
    manager._reset_vision_tf = Mock()
    for name, method in (
        ("voltage", manager._set_tool_voltage_via_io),
        ("notify", manager._moveit_manager.notify_voltage_change),
        ("sleep", tool_exchange_manager.time.sleep),
        ("send", manager._send_and_wait),
        ("state", manager._on_gripper_changed),
        ("launch", manager._moveit_manager.launch_moveit_with_gripper),
        ("reset", manager._reset_vision_tf),
    ):
        events.attach_mock(method, name)
    manager._send_and_wait.return_value = outcome != "action_failure"
    manager._moveit_manager.launch_moveit_with_gripper.return_value = (
        outcome != "restart_failure"
    )
    step = {"operation": operation, "gripper": "hande", "dock_number": 2,
            "approach_pose": "dock_approach"}

    assert manager.exchange(step, '{"pose": [1]}', current, 30) is (outcome == "success")

    goal = manager._send_and_wait.call_args.args[1]
    expected_fields = {
        "operation": operation, "gripper": "hande", "current_attached_gripper": current,
        "dock_number": 2, "approach_pose": "dock_approach", "poses_json": '{"pose": [1]}',
    }
    assert {name: getattr(goal, name) for name in expected_fields} == expected_fields
    expected = []
    if operation == "dock" and not mock_hardware:
        expected = [call.voltage(0), call.notify(0), call.sleep(0.5)]
    expected.append(call.send(manager._action_client, goal, "tool_exchange", 30))
    if outcome != "action_failure":
        expected += [call.state(new), call.launch(new)]
        if outcome == "success":
            expected.append(call.reset())
        else:
            assert manager.last_error == (
                f"Failed to restart MoveIt after tool exchange ({current} → {new})"
            )
    else:
        assert manager.last_error == ""  # The orchestrator retains the action error.
    assert events.mock_calls == expected


@pytest.mark.parametrize("mock_hardware", [False, True])
def test_dock_mismatch_rejected_before_voltage_change(mock_hardware, manager):
    manager._use_mock_hardware = mock_hardware
    manager._grippers["epick"] = {}
    manager._set_tool_voltage_via_io = Mock()

    assert not manager.exchange(
        {"operation": "dock", "gripper": "epick"}, "{}", "hande", 30,
    )

    assert manager.last_error == "Cannot dock epick: hande is attached"
    manager._logger.error.assert_called_once_with(manager.last_error)
    manager._set_tool_voltage_via_io.assert_not_called()
    manager._send_and_wait.assert_not_called()


def test_unconfirmed_voltage_stops_exchange(manager):
    manager._set_tool_voltage_via_io = Mock(return_value=False)

    assert not manager.exchange(
        {"operation": "dock", "gripper": "hande"}, "{}", "hande", 30,
    )

    assert manager.last_error == "Tool voltage change to 0V was not confirmed"
    manager._send_and_wait.assert_not_called()
    manager._moveit_manager.notify_voltage_change.assert_not_called()
    manager._moveit_manager.launch_moveit_with_gripper.assert_not_called()
    manager._on_gripper_changed.assert_not_called()
    tool_exchange_manager.time.sleep.assert_not_called()
