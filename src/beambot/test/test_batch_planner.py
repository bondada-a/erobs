"""Behavioral contracts for grouping tasks; no ROS or motion planning."""

import copy

import pytest

from beambot.utils.task_batching import group_into_batches


@pytest.mark.parametrize(
    ("task_types", "expected"),
    [
        ([], []),
        (["moveto"], [("batched", ["moveto"])]),
        (["end_effector"], [("batched", ["end_effector"])]),
        (
            ["moveto", "end_effector", "moveto"],
            [("batched", ["moveto", "end_effector", "moveto"])],
        ),
        (
            ["moveto", "moveto", "vision_moveto", "moveto"],
            [
                ("batched", ["moveto", "moveto"]),
                ("single", ["vision_moveto"]),
                ("batched", ["moveto"]),
            ],
        ),
        (
            ["tool_exchange", "vision_scan", "moveto", "pause"],
            [
                ("single", ["tool_exchange"]),
                ("single", ["vision_scan"]),
                ("batched", ["moveto"]),
                ("single", ["pause"]),
            ],
        ),
    ],
)
def test_batch_boundaries(task_types, expected):
    tasks = [{"task_type": task_type} for task_type in task_types]
    batches = group_into_batches(tasks)
    assert [
        (kind, [task["task_type"] for task in batch]) for kind, batch in batches
    ] == expected


@pytest.mark.parametrize(
    "task_type",
    [
        "tool_exchange", "vision_task", "vision_moveto", "vision_scan",
        "pick_sample", "place_sample", "pick_spincoater", "place_spincoater",
        "pipettor", "pause", "unknown_type",
    ],
)
def test_nonbatchable_tasks_remain_single(task_type):
    task = {"task_type": task_type}
    assert group_into_batches([task]) == [("single", [task])]


def test_missing_task_type_remains_single():
    task = {"target": "home"}
    assert group_into_batches([task]) == [("single", [task])]


@pytest.mark.parametrize("enabled", [True, False])
def test_task_order_and_data_survive_batching_without_mutation(enabled):
    tasks = [
        {"task_type": "moveto", "target": "pick", "planning_type": "joint"},
        {"task_type": "end_effector", "end_effector_action": "vacuum_on"},
        {"task_type": "vision_scan", "scan_positions": ["scan_a", "scan_b"]},
        {"task_type": "moveto", "target": "place"},
        {"task_type": "end_effector", "end_effector_action": "vacuum_off"},
    ]
    original = copy.deepcopy(tasks)
    batches = group_into_batches(tasks, enabled=enabled)

    assert [task for _, batch in batches for task in batch] == original
    assert tasks == original
    if not enabled:
        assert batches == [("single", [task]) for task in original]
