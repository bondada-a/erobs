"""Task-script validation tests."""

import json
from types import SimpleNamespace

import pytest

from beambot.core.task_script import parse_task_script


GRIPPERS = {"epick": {}}
SUPPORTED_TYPES = [
    "moveto",
    "end_effector",
    "tool_exchange",
    "vision_task",
    "vision_moveto",
    "vision_scan",
    "pick_sample",
    "place_sample",
    "pick_spincoater",
    "place_spincoater",
    "pipettor",
]


def _parse(document, *, dry_run=False):
    return parse_task_script(
        json.dumps(document),
        dry_run=dry_run,
        grippers=GRIPPERS,
        poses_file="/nonexistent",
    )


@pytest.mark.parametrize(
    ("document", "path"),
    [
        (3, "root"),
        ([], "root"),
        ({"start_gripper": [], "tasks": [{"task_type": "moveto"}]}, "start_gripper"),
        ({"start_gripper": "epick", "tasks": {}}, "tasks"),
        ({"start_gripper": "epick", "tasks": "moveto"}, "tasks"),
        ({"start_gripper": "epick", "tasks": []}, "tasks"),
        (
            {"start_gripper": "epick", "tasks": [{"task_type": "moveto"}, 3]},
            "tasks[1]",
        ),
        (
            {"start_gripper": "epick", "tasks": [{"task_type": []}]},
            "tasks[0].task_type",
        ),
        (
            {
                "start_gripper": "epick",
                "tasks": [{"task_type": "moveto"}, {"task_type": "unknown"}],
            },
            "tasks[1].task_type",
        ),
    ],
)
def test_invalid_document_is_a_path_specific_value_error(document, path):
    with pytest.raises(ValueError) as error:
        _parse(document)

    assert path in str(error.value)


@pytest.mark.parametrize("task_type", SUPPORTED_TYPES)
def test_live_run_accepts_supported_task_types(task_type):
    _, tasks, _, _ = _parse(
        {"start_gripper": "epick", "tasks": [{"task_type": task_type}]}
    )

    assert tasks[0]["task_type"] == task_type


def test_dry_run_remains_limited_to_moveto_and_end_effector():
    with pytest.raises(ValueError, match="Dry-run not supported"):
        _parse(
            {"start_gripper": "epick", "tasks": [{"task_type": "vision_scan"}]},
            dry_run=True,
        )


def test_unknown_final_task_has_zero_robot_side_effects():
    orchestrator_module = pytest.importorskip(
        "beambot.action_servers.orchestrator", reason="requires ROS"
    )
    orchestrator_type = orchestrator_module.MTCOrchestratorServer
    effects = {"gripper": 0, "moveit": 0, "capture": 0, "motion": 0}

    class GoalHandle:
        request = SimpleNamespace(
            full_json=json.dumps(
                {
                    "start_gripper": "epick",
                    "tasks": [
                        {"task_type": "moveto"},
                        {"task_type": "vision_scan"},
                        {"task_type": "unknown"},
                    ],
                }
            ),
            dry_run=False,
        )
        is_cancel_requested = False
        abort_count = 0

        def abort(self):
            self.abort_count += 1

    def set_gripper(gripper):
        effects["gripper"] += 1
        orchestrator._current_gripper = gripper

    def launch_moveit(_gripper):
        effects["moveit"] += 1
        return True

    def execute_step(task_type, _task, _poses):
        effects["capture" if task_type == "vision_scan" else "motion"] += 1
        if task_type == "unknown":
            orchestrator._last_error = "unknown"
            return False
        return True

    logger = SimpleNamespace(
        info=lambda *_: None,
        warning=lambda *_: None,
        error=lambda *_: None,
    )
    orchestrator = orchestrator_type.__new__(orchestrator_type)
    orchestrator.get_logger = lambda: logger
    orchestrator.get_parameter = lambda _name: SimpleNamespace(value="")
    orchestrator._grippers = GRIPPERS
    orchestrator._poses_file = "/nonexistent"
    orchestrator._vacuum = SimpleNamespace(
        reset=lambda: None, update_after_tasks=lambda *_: None
    )
    orchestrator._moveit_manager = SimpleNamespace(
        current_arm_joints=lambda: None,
        launch_moveit_with_gripper=launch_moveit,
        is_moveit_alive=lambda: True,
    )
    orchestrator._set_current_gripper = set_gripper
    orchestrator._execute_step = execute_step
    orchestrator._update_feedback = lambda *_: None
    orchestrator._publish_state = lambda *_: None
    orchestrator._enable_batching = False
    orchestrator._pause_requested = False
    orchestrator._last_error = ""

    goal_handle = GoalHandle()
    result = orchestrator._execute(goal_handle)

    assert "tasks[2].task_type" in result.error_message
    assert goal_handle.abort_count == 1
    assert effects == {"gripper": 0, "moveit": 0, "capture": 0, "motion": 0}
