"""Tests for tool-exchange motion planning."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from beambot.stages import tool_exchange_stages
from beambot.stages.tool_exchange_stages import ToolExchangeStages


@pytest.mark.parametrize(
    ("operation", "current_gripper"),
    [("load", "none"), ("dock", "hande")],
)
def test_cartesian_stages_require_complete_paths(operation, current_gripper):
    exchange = ToolExchangeStages.__new__(ToolExchangeStages)
    exchange.logger = Mock()
    exchange.arm_group = "ur_arm"
    exchange.parse_poses = Mock(return_value={"approach": [0] * 6})
    exchange.create_task_template = Mock(
        return_value=SimpleNamespace(add=Mock())
    )
    exchange.make_pipeline_planner = Mock(return_value=object())
    planner = SimpleNamespace(min_fraction=0.9)
    exchange.make_cartesian_planner = Mock(return_value=planner)
    exchange.get_joint_pose = Mock(return_value=[0] * 6)
    exchange._set_ik_frame = Mock()
    exchange.create_relative_move_stage = Mock(return_value=object())
    exchange.load_plan_execute = Mock(return_value=None)

    goal = SimpleNamespace(
        operation=operation,
        gripper="hande",
        current_attached_gripper=current_gripper,
        dock_number=2,
        approach_pose="approach",
        poses_json="{}",
    )
    move_to = SimpleNamespace(setGoal=Mock())

    with patch.object(tool_exchange_stages.stages, "MoveTo", return_value=move_to):
        assert exchange.run(goal) is None

    planners = [
        call.args[-1]
        for call in exchange.create_relative_move_stage.call_args_list
    ]
    assert len(planners) == 4
    assert all(stage_planner is planner for stage_planner in planners)
    assert planner.min_fraction == 1.0
