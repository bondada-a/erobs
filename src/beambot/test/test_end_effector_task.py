"""Tests for standalone and batched gripper tasks."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from beambot.motion import task_builder
from beambot.motion.end_effector_task import EndEffectorTask


@pytest.mark.parametrize("mode", ["standalone", "batch_default", "batch_custom"])
@pytest.mark.parametrize(
    ("group", "action", "expected_error"),
    [
        ("", "", None),
        ("", "hande_closed", None),
        ("hande_gripper", "", "No end_effector_action specified"),
        ("hande_gripper", "hande_closed", None),
    ],
)
def test_gripper_stage_behavior(mode, group, action, expected_error, monkeypatch):
    stage = SimpleNamespace(setGoal=Mock())
    move_to = Mock(return_value=stage)
    monkeypatch.setattr(task_builder.stages, "MoveTo", move_to)
    instance = EndEffectorTask.__new__(EndEffectorTask)
    instance.logger = Mock()
    planner = object()
    instance.make_joint_interpolation_planner = Mock(return_value=planner)
    task = SimpleNamespace(add=Mock())
    instance.create_task_template = Mock(return_value=task)
    instance.load_plan_execute = Mock(return_value=None)
    goal = SimpleNamespace(gripper_group=group, end_effector_action=action)

    if mode == "standalone":
        result = instance.run(goal)
    else:
        result = instance.add_to_task(
            task, goal, planner if mode == "batch_custom" else None
        )
    assert result == expected_error

    if mode == "standalone" and group:
        instance.create_task_template.assert_called_once_with("EndEffector Task")
    else:
        instance.create_task_template.assert_not_called()

    if group and action:
        move_to.assert_called_once_with(f"gripper_{action}", planner)
        assert stage.group == group
        stage.setGoal.assert_called_once_with(action)
        task.add.assert_called_once_with(stage)
    else:
        move_to.assert_not_called()
        task.add.assert_not_called()

    if group and action and mode != "batch_custom":
        instance.make_joint_interpolation_planner.assert_called_once_with()
    else:
        instance.make_joint_interpolation_planner.assert_not_called()

    if mode == "standalone" and group and action:
        instance.load_plan_execute.assert_called_once_with(task)
    else:
        instance.load_plan_execute.assert_not_called()
