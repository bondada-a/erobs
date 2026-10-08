"""Pipettor command forwarding and driver failure handling."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus
from std_msgs.msg import ColorRGBA

from beambot.motion.pipettor_task import PipettorTask
from beambot.motion import pipettor_task


@pytest.fixture
def pipettor():
    instance = PipettorTask.__new__(PipettorTask)
    instance.logger = Mock()
    instance._action_client = Mock()
    instance._action_client.wait_for_server.return_value = True
    handle = Mock(accepted=True)
    handle.get_result_async.return_value.done.return_value = True
    handle.get_result_async.return_value.result.return_value = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(success=True, message="Complete"),
    )
    send_future = instance._action_client.send_goal_async.return_value
    send_future.done.return_value = True
    send_future.result.return_value = handle
    color = ColorRGBA(r=1.0, g=0.2, b=0.0, a=1.0)
    goal = SimpleNamespace(operation="SUCK", volume_pct=0.5, led_color=color)
    return instance, handle, goal


@pytest.mark.parametrize("operation", ["SUCK", "EXPEL", "EJECT_TIP", "SET_LED"])
def test_pipettor_commands(pipettor, operation):
    instance, handle, goal = pipettor
    goal.operation = operation

    assert instance.run(goal) is None

    forwarded = instance._action_client.send_goal_async.call_args.args[0]
    assert (forwarded.operation, forwarded.volume_pct, forwarded.led_color) == (
        operation, 0.5, goal.led_color,
    )
    handle.cancel_goal_async.assert_not_called()


@pytest.mark.parametrize("failure, message", [
    ("unavailable", "not available"), ("send_timeout", "send timeout"),
    ("rejected", "rejected"), ("result_timeout", "TIMEOUT"),
    ("aborted", "status"), ("driver_failure", "driver refused"),
])
def test_pipettor_failures(pipettor, monkeypatch, failure, message):
    instance, handle, goal = pipettor
    instance._action_client.wait_for_server.return_value = failure != "unavailable"
    handle.accepted = failure != "rejected"
    reply = handle.get_result_async.return_value.result.return_value
    if failure == "aborted":
        reply.status = GoalStatus.STATUS_ABORTED
    if failure == "driver_failure":
        reply.result.success = False
        reply.result.message = "driver refused"
    monkeypatch.setattr(pipettor_task, "wait_for_future", Mock(
        side_effect=[failure != "send_timeout", failure != "result_timeout"],
    ))

    assert message in instance.run(goal)
    assert handle.cancel_goal_async.call_count == int(failure == "result_timeout")
    assert instance._action_client.send_goal_async.call_count == int(failure != "unavailable")
