"""Checks that sample tasks run named poses only and skip the vacuum check."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from beambot.motion import pick_sample_task, place_sample_task


@pytest.mark.parametrize(
    ("module", "class_name"),
    [
        (pick_sample_task, "PickSampleTask"),
        (place_sample_task, "PlaceSampleTask"),
    ],
)
@pytest.mark.parametrize("use_vision", [False, True])
def test_sample_task_runs_named_poses(module, class_name, use_vision, monkeypatch):
    def init_base(self, node, arm_group, *, ik_frame):
        self.rclpy_node = node
        self.logger = node.get_logger()

    monkeypatch.setattr(module.MtcTaskBuilder, "__init__", init_base)
    stage = getattr(module, class_name)(Mock())
    stage.parse_poses = Mock(return_value={})
    stage._run_hardcoded = Mock(return_value=None)
    stage._check_vacuum = Mock(side_effect=AssertionError("Vacuum check is disabled"))
    # Vision pick/place goes to the vision server; a stray use_vision flag here
    # still runs the named-pose sequence.
    goal = SimpleNamespace(
        use_vision=use_vision, offset_direction="", offset_distance=0.0,
        poses_json="{}", gripper_states_json="", constraints_json="",
        gripper_group="",
    )

    assert stage.run(goal) is None
    stage._run_hardcoded.assert_called_once_with(goal, {}, {}, None)
    stage._check_vacuum.assert_not_called()
    if class_name == "PickSampleTask":
        assert stage.vacuum_ok is True


@pytest.mark.parametrize(
    ("module", "class_name"),
    [
        (pick_sample_task, "PickSampleTask"),
        (place_sample_task, "PlaceSampleTask"),
    ],
)
def test_sample_task_returns_hardcoded_error(module, class_name, monkeypatch):
    def init_base(self, node, arm_group, *, ik_frame):
        self.rclpy_node = node
        self.logger = node.get_logger()

    monkeypatch.setattr(module.MtcTaskBuilder, "__init__", init_base)
    stage = getattr(module, class_name)(Mock())
    stage.parse_poses = Mock(return_value={})
    stage._run_hardcoded = Mock(return_value="Pose 'missing' not found")
    goal = SimpleNamespace(
        use_vision=False, offset_direction="", offset_distance=0.0,
        poses_json="{}", gripper_states_json="", constraints_json="",
        gripper_group="",
    )

    assert stage.run(goal) == "Pose 'missing' not found"
