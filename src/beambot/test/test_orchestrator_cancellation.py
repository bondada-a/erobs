"""Graceful orchestrator cancellation tests."""

import json
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from moveit_task_constructor_msgs.msg import Solution
from rclpy.action import GoalResponse

from beambot.action_servers import orchestrator as orchestrator_module
from beambot.motion import task_builder, move_to_task

BeambotOrchestratorServer = orchestrator_module.BeambotOrchestratorServer


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
        self.feedback = []

    def canceled(self):
        self.canceled_count += 1

    def succeed(self):
        self.succeeded_count += 1

    def abort(self):
        self.aborted_count += 1

    def publish_feedback(self, feedback):
        self.feedback.append(
            (
                feedback.current_step,
                feedback.progress_percentage,
                feedback.status_message,
            )
        )


def make_orchestrator(states):
    server = BeambotOrchestratorServer.__new__(BeambotOrchestratorServer)
    server._executing = True
    server._faulted = False
    server._lock = threading.Lock()
    server._grippers = {"epick": {}}
    server._poses_file = "/nonexistent"
    server._vision_targets = {}
    server._current_gripper = "unknown"
    server._enable_batching = True
    server._pause_requested = False
    server._is_paused = False
    server._pause_event = threading.Event()
    server._last_error = ""
    server._last_planned_sol_msg = None
    server._vacuum = SimpleNamespace(reset=Mock(), update_after_tasks=Mock())
    server._moveit_manager = SimpleNamespace(
        cup_override="",
        model_revision="test-model",
        current_arm_joints=lambda: None,
        launch_moveit_with_gripper=lambda _gripper: True,
        is_moveit_alive=lambda: True,
    )
    server._trajectory_cache = SimpleNamespace(get=lambda _key: None, store=lambda *_: None)
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


@pytest.mark.parametrize("batching", [False, True])
def test_unknown_final_task_has_zero_robot_side_effects(batching):
    server = make_orchestrator([])
    server._enable_batching = batching
    effects = Mock()
    server._set_current_gripper = effects.set_gripper
    server._moveit_manager.launch_moveit_with_gripper = effects.launch_moveit
    server._execute_batch = effects.execute_batch
    server._execute_step = effects.execute_step
    goal = GoalHandle([
        {"task_type": "moveto", "target": "home"},
        {"task_type": "vision_scan"},
        {"task_type": "unknown"},
    ])

    result = server._execute_callback(goal)

    assert not result.success
    assert result.completed_steps == 0
    assert "tasks[2].task_type" in result.error_message
    assert goal.aborted_count == 1
    assert effects.mock_calls == []


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
    assert result.total_steps == 3
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


@pytest.mark.parametrize("dry_run", [False, True])
def test_single_task_feedback_precedes_dispatch_once(dry_run):
    server = make_orchestrator([])
    server._enable_batching = False
    server._update_feedback = Mock()
    goal = GoalHandle([{"task_type": "moveto", "target": "home"}])
    goal.request.dry_run = dry_run

    def execute(*_args, **_kwargs):
        assert server._update_feedback.call_count == 2
        assert server._update_feedback.call_args.args[2:] == (1, 1, "moveto")
        return True

    server._execute_batch = Mock(side_effect=execute)
    server._execute_step = Mock(side_effect=execute)
    result = server._execute_callback(goal)

    assert result.success
    assert result.completed_steps == 1
    assert server._update_feedback.call_count == 3
    assert server._execute_batch.call_count == int(dry_run)
    assert server._execute_step.call_count == int(not dry_run)


@pytest.mark.parametrize("enable_batching", [False, True])
@pytest.mark.parametrize("dry_run", [False, True])
def test_execution_without_vacuum_monitor(enable_batching, dry_run):
    server = make_orchestrator([])
    del server._vacuum
    server._enable_batching = enable_batching
    server._execute_batch = Mock(return_value=True)
    server._execute_step = Mock(return_value=True)
    goal = GoalHandle([{"task_type": "moveto", "target": "home"}])
    goal.request.dry_run = dry_run

    result = server._execute_callback(goal)

    assert result.success
    assert result.completed_steps == 1
    assert goal.succeeded_count == 1


@pytest.mark.parametrize(
    "task_types, batching, eligible",
    [
        (["moveto"], True, True),
        (["moveto", "end_effector"], True, True),
        (["moveto", "vision_moveto", "moveto"], True, False),
        (["moveto", "tool_exchange", "moveto"], True, False),
        (["moveto", "moveto"], False, False),
        (["pick_sample"], True, False),
    ],
)
def test_orchestrator_caches_only_one_complete_motion_batch(task_types, batching, eligible):
    server = make_orchestrator([])
    server._enable_batching = batching
    server._trajectory_cache = SimpleNamespace(get=Mock(return_value=None), store=Mock())
    solution = object()

    def plan_batch(*_args, **kwargs):
        assert kwargs["cached_trajectory"] is None
        server._last_planned_sol_msg = solution
        return True

    server._execute_batch = Mock(side_effect=plan_batch)
    server._execute_step = Mock(return_value=True)
    goal = GoalHandle([
        {"task_type": kind, **({"target": "home"} if kind == "moveto" else {})}
        for kind in task_types
    ])

    result = server._execute_callback(goal)

    assert result.success
    assert result.completed_steps == len(task_types)
    if eligible:
        server._trajectory_cache.get.assert_called_once()
        key, = server._trajectory_cache.get.call_args.args
        server._trajectory_cache.store.assert_called_once_with(key, solution)
    else:
        server._trajectory_cache.get.assert_not_called()
        server._trajectory_cache.store.assert_not_called()


@pytest.mark.parametrize("dry_run", [False, True])
def test_orchestrator_replays_live_goals_but_replans_previews(dry_run):
    server = make_orchestrator([])
    cached_trajectory = object()
    server._trajectory_cache = SimpleNamespace(get=Mock(return_value=cached_trajectory), store=Mock())
    fresh_solution = object()

    def execute_batch(_tasks, _poses, **kwargs):
        assert kwargs["dry_run"] is dry_run
        assert kwargs["cached_trajectory"] is (None if dry_run else cached_trajectory)
        server._last_planned_sol_msg = fresh_solution if dry_run else None
        return True

    server._execute_batch = Mock(side_effect=execute_batch)
    server._execute_step = Mock(side_effect=AssertionError("single-task dispatch ran"))
    goal = GoalHandle([{"task_type": "moveto", "target": "home"}])
    goal.request.dry_run = dry_run

    assert server._execute_callback(goal).success
    server._execute_batch.assert_called_once()
    if dry_run:
        server._trajectory_cache.get.assert_not_called()
        server._trajectory_cache.store.assert_called_once()
        assert server._trajectory_cache.store.call_args.args[1] is fresh_solution
    else:
        server._trajectory_cache.get.assert_called_once()
        server._trajectory_cache.store.assert_not_called()


@pytest.mark.parametrize("outcome", [
    "live", "dry_run", "stage_failure", "plan_failure", "execution_failure",
    "replay", "replay_timeout", "replay_failure", "exception", "construction_error",
])
def test_repeated_batches_release_tf_subscriptions(outcome, monkeypatch):
    server = SimpleNamespace(
        _arm_group="ur_arm",
        subscriptions=[object()],
        get_logger=lambda: Mock(),
        _create_moveto_goal=lambda step, poses: step,
    )
    original_subscriptions = server.subscriptions.copy()

    class Listener:
        def __init__(self, buffer, node):
            self.node = node
            # Model the node retaining both TF subscription callbacks.
            node.subscriptions.extend([self, self])

        def unregister(self):
            self.node.subscriptions.remove(self)
            self.node.subscriptions.remove(self)

    monkeypatch.setattr(
        task_builder.MtcTaskBuilder, "__init__",
        lambda self, node, *_: setattr(self, "rclpy_node", node),
    )
    monkeypatch.setattr(move_to_task, "Buffer", object)
    monkeypatch.setattr(move_to_task, "TransformListener", Listener)

    class Stage(move_to_task.MoveToTask):
        last_sol_msg = object()

        def create_task_template(self, name):
            assert len(server.subscriptions) == 3
            return object()

        def add_to_task(self, task, goal):
            return "stage failed" if outcome == "stage_failure" else None

        def init_and_plan(self, task, dry_run=False):
            assert dry_run
            assert len(server.subscriptions) == 3
            return "planning failed" if outcome == "plan_failure" else None

        def load_plan_execute(self, task):
            assert len(server.subscriptions) == 3
            if outcome == "exception":
                raise RuntimeError("execution exception")
            return "execution failed" if outcome == "execution_failure" else None

        def execute_solution_msg(self, solution, is_replay=False):
            assert is_replay
            assert len(server.subscriptions) == 3
            if outcome == "replay_timeout":
                return "REPLAY_TIMEOUT: result unavailable"
            return "replay failed" if outcome == "replay_failure" else None

    monkeypatch.setattr(orchestrator_module, "MoveToTask", Stage)
    monkeypatch.setattr(
        orchestrator_module, "EndEffectorTask",
        Mock(side_effect=RuntimeError("construction failed"))
        if outcome == "construction_error" else Mock(),
    )
    cached_trajectory = object() if outcome.startswith("replay") else None
    dry_run = outcome in {"dry_run", "plan_failure"}

    for _ in range(3):
        server._last_planned_sol_msg = None
        kwargs = dict(dry_run=dry_run, cached_trajectory=cached_trajectory)
        if outcome in {"exception", "construction_error"}:
            with pytest.raises(RuntimeError):
                BeambotOrchestratorServer._execute_batch(
                    server, [{"task_type": "moveto"}], "{}", **kwargs
                )
        else:
            result = BeambotOrchestratorServer._execute_batch(
                server, [{"task_type": "moveto"}], "{}", **kwargs
            )
            assert result == (outcome in {"live", "dry_run", "replay", "replay_failure"})
            if outcome in {"live", "dry_run", "replay_failure"}:
                assert server._last_planned_sol_msg is Stage.last_sol_msg
        assert server.subscriptions == original_subscriptions


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


@pytest.mark.parametrize("cancel", [False, True])
def test_automatic_pause_resumes_or_cancels_without_robot_dispatch(cancel):
    states = []
    server = make_orchestrator(states)
    server._execute_step = Mock(side_effect=AssertionError("pause dispatched"))
    goal = GoalHandle([{"task_type": "pause"}])
    resume_response = SimpleNamespace(success=False, message="")

    def on_wait(timeout):
        assert server._is_paused
        # An extra wait fails immediately instead of hanging the test process.
        assert server._pause_event.wait.call_count == 1
        if cancel:
            goal.is_cancel_requested = True
            return False
        server._resume_callback(None, resume_response)
        return server._pause_event.is_set()

    server._pause_event.wait = Mock(side_effect=on_wait)
    result = server._execute_callback(goal)

    server._pause_event.wait.assert_called_once()
    assert resume_response.success is (not cancel)
    assert result.success is (not cancel)
    assert result.completed_steps == int(not cancel)
    assert goal.succeeded_count == int(not cancel)
    assert goal.canceled_count == int(cancel)
    assert goal.feedback[-1][0] == 1
    assert "completed 0/1" in goal.feedback[-1][2]
    server._execute_step.assert_not_called()
    server._vacuum.update_after_tasks.assert_not_called()


def test_missing_child_terminal_result_latches_fault(monkeypatch):
    server = BeambotOrchestratorServer.__new__(BeambotOrchestratorServer)
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
        orchestrator_module,
        "wait_for_future",
        lambda *_args, **_kwargs: next(completions),
    )

    assert not server._send_and_wait(client, object(), "child", 1.0)
    assert server._faulted
    child_goal.cancel_goal_async.assert_called_once_with()


@pytest.mark.parametrize(
    ("step", "message"),
    [
        ({}, "vision_scan requires 'scan_positions' list"),
        ({"scan_positions": ["missing"]}, "No valid scan positions found"),
    ],
)
def test_vision_scan_validation_propagates_error(step, message):
    server = BeambotOrchestratorServer.__new__(BeambotOrchestratorServer)
    server.get_logger = lambda: Mock()

    assert not server._call_vision_scan(step, "{}")
    assert server._last_error == message


def test_unknown_moveit_goal_acceptance_is_not_safe_to_retry(monkeypatch):
    stage = task_builder.MtcTaskBuilder.__new__(task_builder.MtcTaskBuilder)
    stage.rclpy_node = SimpleNamespace()
    stage.logger = Mock()
    client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda _goal: object(),
    )
    monkeypatch.setattr(task_builder, "ActionClient", lambda *_args, **_kwargs: client)
    monkeypatch.setattr(task_builder, "wait_for_future", lambda *_args, **_kwargs: False)

    error = stage.execute_solution_msg(Solution())

    assert error.startswith("REPLAY_TIMEOUT:")
