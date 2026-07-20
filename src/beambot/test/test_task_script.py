"""Task-script validation tests."""

import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from beambot.core.task_script import TASK_MACROS, moveto_goal_error, parse_task_script


GRIPPERS = {"epick": {}, "pipettor": {}}
VISION_TARGETS = yaml.safe_load(
    (Path(__file__).parents[1] / "config" / "cms_beamline.yaml").read_text()
)["vision_targets"]
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
    "pause",
]


def _parse(document, *, dry_run=False, vision_targets=VISION_TARGETS):
    return parse_task_script(
        json.dumps(document),
        dry_run=dry_run,
        grippers=GRIPPERS,
        poses_file="/nonexistent",
        vision_targets=vision_targets,
    )


def _moveto_error(task):
    return moveto_goal_error(
        target=task.get("target", ""),
        direction=task.get("direction", ""),
        distance=task.get("distance", 0.0),
        cartesian_target=task.get("cartesian_target", ()),
        planning_type=task.get("planning_type", ""),
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
            {
                "start_gripper": "epick",
                "tasks": [{"task_type": "moveto", "target": "home"}, 3],
            },
            "tasks[1]",
        ),
        (
            {"start_gripper": "epick", "tasks": [{"task_type": []}]},
            "tasks[0].task_type",
        ),
        (
            {
                "start_gripper": "epick",
                "tasks": [
                    {"task_type": "moveto", "target": "home"},
                    {"task_type": "unknown"},
                ],
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
    task = {"task_type": task_type}
    if task_type == "moveto":
        task["target"] = "home"
    _, tasks, _, _ = _parse({"start_gripper": "epick", "tasks": [task]})

    assert tasks[0]["task_type"] == task_type


@pytest.mark.parametrize(
    "goal",
    [
        {"target": "home"},
        {"target": "home", "planning_type": "auto"},
        {"target": "home", "planning_type": "joint"},
        {"direction": "backward", "distance": 0.1, "planning_type": "cartesian"},
        {"cartesian_target": [0.3, -0.2, 0.15], "planning_type": "pilz"},
        {
            "cartesian_target": [0.3, -0.2, 0.15, 180, 0, 0],
            "planning_type": "pilz_ptp",
        },
    ],
)
def test_valid_moveto_goal_modes(goal):
    assert moveto_goal_error(**goal) is None


@pytest.mark.parametrize(
    ("goal", "message"),
    [
        ({}, "exactly one"),
        (
            {"target": "home", "direction": "backward", "distance": 0.1},
            "exactly one",
        ),
        (
            {"direction": "backward", "distance": 0.1, "cartesian_target": [0, 0, 0]},
            "exactly one",
        ),
        ({"direction": "backward"}, "distance"),
        ({"distance": 0.1}, "direction"),
        ({"direction": "diagonal", "distance": 0.1}, "direction"),
        ({"direction": "backward", "distance": 0.0}, "distance"),
        ({"direction": "backward", "distance": -0.1}, "distance"),
        ({"direction": "backward", "distance": float("nan")}, "distance"),
        ({"direction": "backward", "distance": float("inf")}, "distance"),
        ({"cartesian_target": [0, 0, 0, 0]}, "cartesian_target"),
        ({"cartesian_target": [0, 0, float("nan")]}, "cartesian_target"),
        ({"cartesian_target": [0, 0, float("inf")]}, "cartesian_target"),
        ({"planning_type": "unknown", "target": "home"}, "planning_type"),
        ({"target": "   "}, "target"),
    ],
)
def test_invalid_moveto_goals(goal, message):
    assert message in moveto_goal_error(**goal)


@pytest.mark.parametrize(
    "offset",
    [
        {"offset_direction": "right"},
        {"offset_distance": 0.1},
        {"offset_direction": "right", "offset_distance": -0.1},
        {"offset_direction": "right", "offset_distance": float("nan")},
        {"offset_direction": "sideways", "offset_distance": 0.1},
    ],
)
def test_task_script_rejects_invalid_flange_offsets(offset):
    with pytest.raises(ValueError, match="offset"):
        _parse(
            {
                "start_gripper": "epick",
                "tasks": [{"task_type": "vision_moveto", **offset}],
            }
        )


def test_task_script_accepts_valid_flange_offset():
    _, tasks, _, _ = _parse(
        {
            "start_gripper": "epick",
            "tasks": [
                {
                    "task_type": "vision_moveto",
                    "offset_direction": "right",
                    "offset_distance": 0.1,
                }
            ],
        }
    )

    assert tasks[0]["offset_direction"] == "right"


def test_task_script_rejects_ambiguous_moveto():
    with pytest.raises(ValueError, match=r"tasks\[0\].*exactly one"):
        _parse(
            {
                "start_gripper": "epick",
                "tasks": [
                    {
                        "task_type": "moveto",
                        "target": "home",
                        "direction": "backward",
                        "distance": 0.1,
                    }
                ],
            }
        )


def test_checked_in_moveto_tasks_satisfy_contract():
    cms = Path(__file__).parents[3] / "src" / "cms"
    for path in cms.rglob("*.json"):
        document = json.loads(path.read_text())
        for index, task in enumerate(document.get("tasks", [])):
            if task.get("task_type") == "moveto":
                assert _moveto_error(task) is None, f"{path}: tasks[{index}]"


@pytest.mark.parametrize("task_type", ["vision_scan", "pause"])
def test_dry_run_remains_limited_to_moveto_and_end_effector(task_type):
    with pytest.raises(ValueError, match="Dry-run not supported"):
        _parse(
            {"start_gripper": "epick", "tasks": [{"task_type": task_type}]},
            dry_run=True,
        )


def test_pickup_macros_expand_to_supported_steps_with_vial_operation_before_retreat():
    _, tasks, _, _ = _parse(
        {
            "start_gripper": "pipettor",
            "tasks": [
                {"task_type": "pickup_tip", "row": 0, "col": 3},
                {
                    "task_type": "pickup_vial",
                    "row": 0,
                    "col": 1,
                    "pipettor_operation": "SUCK",
                    "volume_pct": 0.5,
                },
            ],
        }
    )

    task_types = [task["task_type"] for task in tasks]
    assert TASK_MACROS.keys().isdisjoint(task_types)
    assert task_types.count("vision_moveto") == 2
    pipettor_index = task_types.index("pipettor")
    assert tasks[pipettor_index - 1]["direction"] == "forward"
    assert tasks[pipettor_index + 1]["direction"] == "backward"


def test_element_index_and_row_column_addressing_expand_identically():
    base = {"start_gripper": "pipettor"}
    _, indexed, _, _ = _parse(
        {**base, "tasks": [{"task_type": "pickup_tip", "element_index": 13}]}
    )
    _, addressed, _, _ = _parse(
        {**base, "tasks": [{"task_type": "pickup_tip", "row": 1, "col": 1}]}
    )

    assert indexed == addressed


@pytest.mark.parametrize(
    "task",
    [
        {"task_type": "pickup_tip", "row": 8, "col": 0},
        {"task_type": "pickup_tip", "row": 0, "col": 12},
        {"task_type": "pickup_tip", "element_index": 96},
    ],
)
def test_pickup_macro_rejects_out_of_range_grid_addresses(task):
    with pytest.raises(ValueError, match="out of range"):
        _parse({"start_gripper": "pipettor", "tasks": [task]})


@pytest.mark.parametrize(
    ("field_path", "value", "message"),
    [
        (("mode",), "offset", "mode"),
        (("grid", "col_direction"), "sideways", "col_direction"),
        (("moves", 0, "distance"), float("nan"), "finite number"),
    ],
)
def test_pickup_macro_rejects_bad_target_configuration(field_path, value, message):
    targets = copy.deepcopy(VISION_TARGETS)
    current = targets["tip_rack"]
    for field in field_path[:-1]:
        current = current[field]
    current[field_path[-1]] = value

    with pytest.raises(ValueError, match=message):
        _parse(
            {
                "start_gripper": "pipettor",
                "tasks": [{"task_type": "pickup_tip", "row": 0, "col": 0}],
            },
            vision_targets=targets,
        )


def test_pickup_macro_rejects_missing_target_configuration():
    with pytest.raises(ValueError, match="missing or invalid"):
        _parse(
            {
                "start_gripper": "pipettor",
                "tasks": [{"task_type": "pickup_tip", "row": 0, "col": 0}],
            },
            vision_targets={},
        )


def test_checked_in_pickup_tasks_parse_to_primitive_steps():
    cms = Path(__file__).parents[3] / "src" / "cms"
    paths = [
        cms / "runs" / "anti_solvent_hold.json",
        cms / "runs" / "first_solution_dispense.json",
        cms / "runs" / "hold_anti_solvent_notip.json",
        cms / "tasks" / "pipettor_tip_to_spincoater.json",
    ]

    for path in paths:
        _, tasks, _, _ = _parse(json.loads(path.read_text()))
        assert TASK_MACROS.keys().isdisjoint(task["task_type"] for task in tasks), path


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
                        {"task_type": "moveto", "target": "home"},
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
    orchestrator._vision_targets = VISION_TARGETS
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
