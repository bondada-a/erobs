"""Task-script validation tests."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from beambot.utils.task_parser import TASK_MACROS, moveto_goal_error, parse_task_script


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


def _parse(
    document, *, dry_run=False, vision_targets=VISION_TARGETS,
    poses_file="/nonexistent",
):
    return parse_task_script(
        json.dumps(document),
        dry_run=dry_run,
        grippers=GRIPPERS,
        poses_file=poses_file,
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
        ({"tasks": []}, "start_gripper"),
        ({"start_gripper": "epick"}, "tasks"),
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


@pytest.mark.parametrize("full_json", ["", "not json"])
def test_invalid_json_is_rejected_by_the_task_parser(full_json):
    with pytest.raises(ValueError, match="full_json|Invalid JSON"):
        parse_task_script(
            full_json, dry_run=False, grippers=GRIPPERS,
            poses_file="/nonexistent", vision_targets=VISION_TARGETS,
        )


def test_parser_preserves_motion_and_device_fields():
    document = {
        "start_gripper": "pipettor",
        "poses": {"home": [0, -90, 90, -90, -90, 0]},
        "tasks": [
            {
                "task_type": "moveto", "target": "home",
                "constraints": {"joint_constraints": [{
                    "joint_name": "wrist_3_joint", "position": 0.0,
                    "tolerance_above": 5.0, "tolerance_below": 5.0,
                }]},
            },
            {"task_type": "pick_sample", "use_vision": True, "tag_id": 0},
            {
                "task_type": "place_sample", "tag_id": 30,
                "offset_direction": "right", "offset_distance": 0.0533,
            },
            {"task_type": "pipettor", "operation": "SUCK", "volume_pct": 0.55},
        ],
    }
    gripper, tasks, poses_json = _parse(document)

    assert gripper == "pipettor"
    assert tasks == document["tasks"]
    assert json.loads(poses_json) == document["poses"]


def test_registry_resolves_references_and_inline_poses_take_precedence(tmp_path):
    registry = {
        "home": [0, -90, 90, -90, -90, 0],
        "transfer": [10, -80, 100, -80, -90, 0],
        "scan_a": [20, -70, 110, -70, -90, 0],
        "scan_b": [30, -60, 120, -60, -90, 0],
        "pickup_scan": [40, -50, 130, -50, -90, 0],
        "unused": [50, -40, 140, -40, -90, 0],
    }
    poses_file = tmp_path / "poses.yaml"
    poses_file.write_text(yaml.safe_dump(registry))
    inline_home = [1, -91, 91, -91, -91, 1]
    _, _, poses_json = _parse(
        {
            "start_gripper": "epick",
            "poses": {"home": inline_home},
            "tasks": [
                {"task_type": "moveto", "target": "home"},
                {"task_type": "moveto", "target": "transfer"},
                {"task_type": "vision_scan", "scan_positions": ["scan_a", "scan_b"]},
                {"task_type": "pick_sample", "scan_pose": "pickup_scan"},
            ],
        },
        poses_file=str(poses_file),
    )

    assert json.loads(poses_json) == {
        "home": inline_home,
        "transfer": registry["transfer"],
        "scan_a": registry["scan_a"],
        "scan_b": registry["scan_b"],
        "pickup_scan": registry["pickup_scan"],
    }


@pytest.mark.parametrize("task_type", SUPPORTED_TYPES)
def test_live_run_accepts_supported_task_types(task_type):
    task = {"task_type": task_type}
    if task_type == "moveto":
        task["target"] = "home"
    _, tasks, _ = _parse({"start_gripper": "epick", "tasks": [task]})

    assert tasks[0]["task_type"] == task_type


@pytest.mark.parametrize("gripper", ["epik", "", None, []])
def test_invalid_end_effector_type_rejects_recipe(gripper):
    with pytest.raises(ValueError, match=r"tasks\[1\]\.end_effector_type: unknown gripper"):
        _parse({
            "start_gripper": "epick",
            "tasks": [
                {"task_type": "moveto", "target": "home"},
                {"task_type": "end_effector", "end_effector_type": gripper},
            ],
        })


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
    task = {"task_type": "moveto", **goal}
    _, tasks, _ = _parse(
        {"start_gripper": "epick", "tasks": [task]}, dry_run=True,
    )
    assert tasks == [task]


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
    _, tasks, _ = _parse(
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
    _, tasks, _ = _parse(
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


def test_vial_rinse_expands_to_cycles_then_final_suck_before_retreat():
    _, tasks, _ = _parse(
        {
            "start_gripper": "pipettor",
            "tasks": [
                {
                    "task_type": "pickup_vial",
                    "row": 0,
                    "col": 1,
                    "pipettor_operation": "RINSE",
                    "rinse_count": 2,
                    "volume_pct": 0.4,
                }
            ],
        }
    )

    operations = [
        task["operation"] for task in tasks if task["task_type"] == "pipettor"
    ]
    assert operations == ["SUCK", "EXPEL", "SUCK", "EXPEL", "SUCK"]
    assert all(
        task["volume_pct"] == 0.4
        for task in tasks
        if task["task_type"] == "pipettor"
    )
    assert tasks[-1]["direction"] == "backward"


@pytest.mark.parametrize("rinse_count", [0, -1, 1.5, True])
def test_vial_rinse_rejects_invalid_cycle_count(rinse_count):
    with pytest.raises(ValueError, match="rinse_count"):
        _parse(
            {
                "start_gripper": "pipettor",
                "tasks": [
                    {
                        "task_type": "pickup_vial",
                        "row": 0,
                        "col": 0,
                        "pipettor_operation": "RINSE",
                        "rinse_count": rinse_count,
                    }
                ],
            }
        )


def test_pickup_task_config_overrides_grid_and_moves():
    _, tasks, _ = _parse(
        {
            "start_gripper": "pipettor",
            "tasks": [
                {
                    "task_type": "pickup_tip",
                    "row": 1,
                    "col": 1,
                    "config": {
                        "marker_id": 42,
                        "scan_pose": "custom_scan",
                        "grid": {
                            "row_pitch": 0.02,
                            "col_pitch": 0.03,
                            "row_offset": 0.01,
                            "col_offset": 0.01,
                        },
                        "moves": [
                            "column_offset",
                            "row_offset",
                            {"direction": "forward", "distance": 0.04},
                        ],
                    },
                }
            ],
        }
    )

    assert tasks == [
        {"task_type": "moveto", "target": "custom_scan"},
        {"task_type": "vision_moveto", "tag_id": 42},
        {
            "task_type": "moveto",
            "target": "",
            "planning_type": "cartesian",
            "direction": "right",
            "distance": 0.02,
        },
        {
            "task_type": "moveto",
            "target": "",
            "planning_type": "cartesian",
            "direction": "down",
            "distance": 0.01,
        },
        {
            "task_type": "moveto",
            "target": "",
            "planning_type": "cartesian",
            "direction": "forward",
            "distance": 0.04,
        },
    ]


def test_element_index_and_row_column_addressing_expand_identically():
    base = {"start_gripper": "pipettor"}
    _, indexed, _ = _parse(
        {**base, "tasks": [{"task_type": "pickup_tip", "element_index": 13}]}
    )
    _, addressed, _ = _parse(
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
        cms / "runs" / "2.hold_anti_solvent_notip.json",
        cms / "tasks" / "pipettor_tip_to_spincoater.json",
    ]

    for path in paths:
        _, tasks, _ = _parse(json.loads(path.read_text()))
        assert TASK_MACROS.keys().isdisjoint(task["task_type"] for task in tasks), path
