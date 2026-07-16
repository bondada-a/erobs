"""Graceful orchestrator cancellation tests."""

import json
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus
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
    server._vision_targets = {}
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
    server._publish_state = Mock()
    server.get_logger = lambda: Mock()

    result_future = SimpleNamespace(done=lambda: False)
    cancel_future = SimpleNamespace(
        result=lambda: SimpleNamespace(goals_canceling=[])
    )
    child_goal = SimpleNamespace(
        accepted=True,
        get_result_async=Mock(return_value=result_future),
        cancel_goal_async=Mock(return_value=cancel_future),
    )
    send_future = SimpleNamespace(result=lambda: child_goal)
    client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda _goal: send_future,
    )
    completions = iter((True, False, True))
    monkeypatch.setattr(
        orchestrator_module, "wait_for_future", lambda *_args, **_kwargs: next(completions)
    )

    assert not server._send_and_wait(client, object(), "child", 1.0)
    assert server._faulted
    child_goal.cancel_goal_async.assert_called_once_with()
    server._publish_state.assert_called_once_with("CANCELING")


def test_end_to_end_deadline_shrinks_across_action_admission(monkeypatch):
    server = MTCOrchestratorServer.__new__(MTCOrchestratorServer)
    server._faulted = False
    server.get_logger = lambda: Mock()

    now = [2.0]
    server_waits = []
    future_waits = []
    monkeypatch.setattr(orchestrator_module.time, "monotonic", lambda: now[0])

    result_future = SimpleNamespace(
        result=lambda: SimpleNamespace(result=SimpleNamespace(success=True))
    )
    child_goal = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: result_future,
    )
    send_future = SimpleNamespace(result=lambda: child_goal)

    def wait_for_server(timeout_sec):
        server_waits.append(timeout_sec)
        now[0] += 1.0
        return True

    def wait_for_future(future, timeout):
        future_waits.append(timeout)
        if future is send_future:
            now[0] += 2.0
        return True

    client = SimpleNamespace(
        wait_for_server=wait_for_server,
        send_goal_async=lambda _goal: send_future,
    )
    monkeypatch.setattr(orchestrator_module, "wait_for_future", wait_for_future)
    goal = SimpleNamespace(timeout=10.0)

    assert server._send_and_wait(
        client, goal, "vision_task", 10.0, deadline=10.0
    )
    assert server_waits == pytest.approx([8.0])
    assert goal.timeout == pytest.approx(7.0)
    assert future_waits == pytest.approx([7.0, 5.0])


@pytest.mark.parametrize(
    ("status", "child_error", "faulted"),
    [
        (GoalStatus.STATUS_CANCELED, "CANCELED: stage=execution", False),
        (GoalStatus.STATUS_ABORTED, "CANCELED: stage=execution", True),
        (GoalStatus.STATUS_CANCELED, "EXECUTION_FAILED: controller", True),
    ],
)
def test_timeout_waits_for_accepted_child_cancellation(
    monkeypatch, status, child_error, faulted
):
    server = MTCOrchestratorServer.__new__(MTCOrchestratorServer)
    server._faulted = False
    server._publish_state = Mock()
    server.get_logger = lambda: Mock()

    completions = iter((True, False, True))
    monkeypatch.setattr(
        orchestrator_module,
        "wait_for_future",
        lambda *_args, **_kwargs: next(completions),
    )
    done = iter((False, True))
    result_future = SimpleNamespace(
        done=lambda: next(done),
        result=lambda: SimpleNamespace(
            status=status,
            result=SimpleNamespace(error_message=child_error),
        ),
    )
    cancel_future = SimpleNamespace(
        result=lambda: SimpleNamespace(goals_canceling=[object()])
    )
    child_goal = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: result_future,
        cancel_goal_async=lambda: cancel_future,
    )
    client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda _goal: SimpleNamespace(result=lambda: child_goal),
    )

    assert not server._send_and_wait(client, object(), "child", 1.0)
    assert server._faulted is faulted
    server._publish_state.assert_called_once_with("CANCELING")


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
