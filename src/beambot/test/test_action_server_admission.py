"""Tests for action-server goal admission."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.action import GoalResponse

from beambot.action_servers.base_action_server import BaseActionServer
from beambot.action_servers.orchestrator import BeambotOrchestratorServer


def test_goal_admission_reserves_server_immediately():
    server = SimpleNamespace(
        _executing=False,
        get_logger=Mock(return_value=Mock()),
    )

    assert BaseActionServer._goal_callback(server, None) == GoalResponse.ACCEPT
    assert BaseActionServer._goal_callback(server, None) == GoalResponse.REJECT


def test_goal_admission_tracks_model_revision_without_busy_overwrite():
    server = SimpleNamespace(
        _executing=False,
        _robot_model_revision="",
        get_logger=Mock(return_value=Mock()),
    )

    managed = SimpleNamespace(robot_model_revision="managed")
    assert BaseActionServer._goal_callback(server, managed) == GoalResponse.ACCEPT
    assert server._robot_model_revision == "managed"

    busy = SimpleNamespace(robot_model_revision="wrong")
    assert BaseActionServer._goal_callback(server, busy) == GoalResponse.REJECT
    assert server._robot_model_revision == "managed"

    server._executing = False
    direct = SimpleNamespace(robot_model_revision="")
    assert BaseActionServer._goal_callback(server, direct) == GoalResponse.ACCEPT
    first_direct_revision = server._robot_model_revision

    server._executing = False
    assert BaseActionServer._goal_callback(server, direct) == GoalResponse.ACCEPT
    assert server._robot_model_revision != first_direct_revision


@pytest.mark.parametrize("outcome", ["success", "failure", "exception"])
def test_terminal_result_releases_server_for_next_goal(outcome):
    server = SimpleNamespace(
        _executing=True,
        _action_type=SimpleNamespace(Result=SimpleNamespace),
        _execute=Mock(return_value=SimpleNamespace(
            success=outcome == "success", error_message="stage failed"
        )),
        get_logger=Mock(return_value=Mock()),
    )
    if outcome == "exception":
        server._execute.side_effect = RuntimeError("stage failed")
    goal = Mock()

    result = BaseActionServer._execute_callback(server, goal)

    assert result.success is (outcome == "success")
    assert goal.succeed.call_count == int(outcome == "success")
    assert goal.abort.call_count == int(outcome != "success")
    assert BaseActionServer._goal_callback(server, None) == GoalResponse.ACCEPT


def test_orchestrator_stamps_only_robot_model_goals():
    class _Future:
        def __init__(self, value):
            self._value = value

        def done(self):
            return True

        def result(self):
            return self._value

    class _GoalHandle:
        accepted = True

        def get_result_async(self):
            return _Future(SimpleNamespace(result=SimpleNamespace(success=True)))

    class _Client:
        def __init__(self):
            self.sent = []

        def wait_for_server(self, timeout_sec):
            return True

        def send_goal_async(self, goal):
            self.sent.append(goal)
            return _Future(_GoalHandle())

    server = BeambotOrchestratorServer.__new__(BeambotOrchestratorServer)
    server._moveit_manager = SimpleNamespace(model_revision="revision")
    server.get_logger = Mock(return_value=Mock())
    client = _Client()

    mtc_goal = SimpleNamespace(robot_model_revision="")
    assert server._send_and_wait(client, mtc_goal, "moveto", 1.0)
    assert mtc_goal.robot_model_revision == "revision"

    pipettor_goal = SimpleNamespace()
    assert server._send_and_wait(client, pipettor_goal, "pipettor", 1.0)
    assert not hasattr(pipettor_goal, "robot_model_revision")


def test_orchestrator_rejects_unversioned_robot_model_dispatch():
    server = BeambotOrchestratorServer.__new__(BeambotOrchestratorServer)
    server._moveit_manager = SimpleNamespace(model_revision="")
    server.get_logger = Mock(return_value=Mock())
    client = Mock()

    goal = SimpleNamespace(robot_model_revision="")
    assert not server._send_and_wait(client, goal, "moveto", 1.0)
    client.wait_for_server.assert_not_called()
