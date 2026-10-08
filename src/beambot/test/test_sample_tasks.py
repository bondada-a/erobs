"""Checks for lazy sample vision setup and disabled pick vacuum checking."""

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
@pytest.mark.parametrize("configured", [False, True])
def test_sample_vision_is_lazy(module, class_name, configured, monkeypatch):
    def init_base(self, node, arm_group, *, ik_frame):
        self.rclpy_node = node
        self.logger = node.get_logger()

    monkeypatch.setattr(module.MtcTaskBuilder, "__init__", init_base)
    factory = Mock()
    monkeypatch.setattr(module, "TargetLocalizer", factory)
    kwargs = {
        "arm_group": "custom_arm" if configured else "",
        "ik_frame": "custom_tip" if configured else "",
        "camera_type": "zivid" if configured else None,
        "camera_frame": "camera_frame" if configured else None,
        "marker_dictionary": "aruco4x4_50" if configured else None,
    }
    node = Mock()
    stage = getattr(module, class_name)(node, **kwargs)
    assert stage._vision is None
    factory.assert_not_called()
    stage.parse_poses = Mock(return_value={})
    stage._run_hardcoded = Mock(return_value=None)
    stage._check_vacuum = Mock(side_effect=AssertionError("Vacuum check is disabled"))
    goal = SimpleNamespace(
        use_vision=False, offset_direction="", offset_distance=0.0,
        poses_json="{}", gripper_states_json="", constraints_json="",
        vision_method="marker", tag_id=1, scan_pose="scan", gripper_group="",
    )
    assert stage.run(goal) is None
    stage._run_hardcoded.assert_called_once_with(goal, {}, {}, None)
    stage._check_vacuum.assert_not_called()
    factory.assert_not_called()
    if class_name == "PickSampleTask":
        assert stage.vacuum_ok is True

    goal.use_vision = True
    stage.create_task_template = Mock(return_value=SimpleNamespace(add=Mock()))
    factory.side_effect = ValueError("Missing camera configuration")
    assert stage.run(goal) == "Vision initialization failed: Missing camera configuration"
    assert stage._vision is None
    stage.create_task_template.assert_not_called()

    factory.reset_mock(side_effect=True)
    stage.make_joint_interpolation_planner = Mock()
    stage.make_gripper_stage = Mock(return_value=None)
    stage.make_move_to_named_stage = Mock(return_value=object())
    stage.load_plan_execute = Mock(return_value="Stop after positioning")
    operation = "pick" if class_name == "PickSampleTask" else "place"
    for _ in range(2):
        assert stage.run(goal) == f"Position for {operation} failed: Stop after positioning"
    factory.assert_called_once_with(node, **kwargs)
    assert stage._vision is factory.return_value

    stage._run_vision = Mock(return_value=None)
    assert stage.run(goal) is None
    stage._check_vacuum.assert_not_called()
