"""Graceful orchestrator cancellation tests."""

import json
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from moveit_task_constructor_msgs.msg import Solution
from rclpy.action import GoalResponse

from beambot.stages import base_stages

orchestrator_module = pytest.importorskip(
    "beambot.action_servers.orchestrator", reason="requires ROS"
)
MTCOrchestratorServer = orchestrator_module.MTCOrchestratorServer


class GoalHandle:
    def __init__(self, tasks):
        self.request = SimpleNamespace(
            full_json=json.dumps({"start_gripper": "epick", "tasks": tasks}),
            dry_run=False,
        )
        self.is_cancel_requested = False
        self.canceled_count = 0
        self.succeeded_count = 0
        self.aborted_count = 0

    def canceled(self):
        self.canceled_count += 1

    def succeed(self):
        self.succeeded_count += 1

    def abort(self):
        self.aborted_count += 1


def make_orchestrator(states):
    server = MTCOrchestratorServer.__new__(MTCOrchestratorServer)
    server._executing = True
    server._faulted = False
    server._lock = threading.Lock()
    server._grippers = {"epick": {}}
    server._poses_file = "/nonexistent"
    server._current_gripper = "unknown"
    server._enable_batching = True
    server._pause_requested = False
    server._last_error = ""
    server._last_planned_sol_msg = None
    server._vacuum = SimpleNamespace(
        reset=lambda: None, update_after_tasks=lambda *_: None
    )
    server._moveit_manager = SimpleNamespace(
        cup_override="",
        current_arm_joints=lambda: None,
        launch_moveit_with_gripper=lambda _gripper: True,
        is_moveit_alive=lambda: True,
    )
    server._plan_cache = SimpleNamespace(get=lambda _key: None, store=lambda *_: None)
    server._set_current_gripper = lambda gripper: setattr(
        server, "_current_gripper", gripper
    )
    server._update_feedback = lambda *_: None
    server._publish_state = states.append
    server.get_parameter = lambda _name: SimpleNamespace(value="")
    server.get_logger = lambda: Mock()
    return server


def request_cancel(server, goal):
    goal.is_cancel_requested = True
    server._cancel_callback(goal)


def test_cancel_finishes_active_batch_and_skips_remaining_work():
    states = []
    server = make_orchestrator(states)
    goal = GoalHandle(
        [
            {"task_type": "moveto", "target": "one"},
            {"task_type": "moveto", "target": "two"},
            {"task_type": "vision_scan"},
        ]
    )
    admissions = []

    def execute_batch(*_args, **_kwargs):
        request_cancel(server, goal)
        admissions.append(server._goal_callback(None))
        return True

    server._execute_batch = execute_batch
    server._execute_step = Mock(side_effect=AssertionError("remaining work ran"))

    result = server._execute_callback(goal)

    assert result.completed_steps == 2
    assert goal.canceled_count == 1
    assert goal.succeeded_count == 0
    assert admissions == [GoalResponse.REJECT]
    assert states == ["RUNNING", "CANCELING", "IDLE"]


def test_cancel_during_final_task_is_not_reported_as_success():
    states = []
    server = make_orchestrator(states)
    goal = GoalHandle([{"task_type": "vision_scan"}])

    def execute_step(*_args):
        request_cancel(server, goal)
        return True

    server._execute_step = execute_step

    result = server._execute_callback(goal)

    assert result.completed_steps == 1
    assert goal.canceled_count == 1
    assert goal.succeeded_count == 0
    assert states == ["RUNNING", "CANCELING", "IDLE"]


def test_failed_active_task_while_canceling_latches_fault():
    states = []
    server = make_orchestrator(states)
    goal = GoalHandle([{"task_type": "vision_scan"}])

    def execute_step(*_args):
        request_cancel(server, goal)
        server._last_error = "controller failed"
        return False

    server._execute_step = execute_step

    server._execute_callback(goal)

    assert goal.aborted_count == 1
    assert server._faulted
    assert server._executing
    assert server._goal_callback(None) == GoalResponse.REJECT
    assert states == ["RUNNING", "CANCELING", "FAULTED"]


def test_missing_child_terminal_result_latches_fault(monkeypatch):
    server = MTCOrchestratorServer.__new__(MTCOrchestratorServer)
    server._faulted = False
    server.get_logger = lambda: Mock()

    child_goal = SimpleNamespace(
        accepted=True,
        get_result_async=Mock(return_value=object()),
        cancel_goal_async=Mock(),
    )
    send_future = SimpleNamespace(result=lambda: child_goal)
    client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda _goal: send_future,
    )
    completions = iter((True, False))
    monkeypatch.setattr(
        orchestrator_module, "wait_for_future", lambda *_args, **_kwargs: next(completions)
    )

    assert not server._send_and_wait(client, object(), "child", 1.0)
    assert server._faulted
    child_goal.cancel_goal_async.assert_called_once_with()


def test_unknown_moveit_goal_acceptance_is_not_safe_to_retry(monkeypatch):
    stage = base_stages.BaseStages.__new__(base_stages.BaseStages)
    stage.rclpy_node = SimpleNamespace()
    stage.logger = Mock()
    client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda _goal: object(),
    )
    monkeypatch.setattr(base_stages, "ActionClient", lambda *_args, **_kwargs: client)
    monkeypatch.setattr(base_stages, "wait_for_future", lambda *_args, **_kwargs: False)

    error = stage.execute_solution_msg(Solution())

    assert error.startswith("REPLAY_TIMEOUT:")
