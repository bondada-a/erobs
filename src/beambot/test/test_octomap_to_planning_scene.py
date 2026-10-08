"""Offline checks for Octomap bridge messages and throttling."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import Octomap, OctomapWithPose

from beambot.mapping import octomap_to_planning_scene


@pytest.mark.parametrize("logs", [False, True])
def test_bridge_preserves_messages_and_uses_elapsed_time(logs, monkeypatch):
    clock = MagicMock()
    clock.time.side_effect = AssertionError("Throttle must not use wall-clock time")
    monkeypatch.setattr(octomap_to_planning_scene, "time", clock)
    params = {}
    logger = MagicMock()
    published = []
    node = octomap_to_planning_scene.Node
    monkeypatch.setattr(node, "__init__", lambda self, name: None)
    monkeypatch.setattr(node, "declare_parameter",
                        lambda self, name, default: params.update({name: logs if name == "log_updates" else default}))
    monkeypatch.setattr(node, "get_parameter", lambda self, name: SimpleNamespace(value=params[name]))
    monkeypatch.setattr(node, "get_logger", lambda self: logger)
    monkeypatch.setattr(node, "create_subscription", lambda *args: MagicMock())
    monkeypatch.setattr(node, "create_publisher", lambda *args: SimpleNamespace(publish=published.append))
    relay = octomap_to_planning_scene.OctomapToPlanningScene()
    logger.reset_mock()

    messages = []
    for index, seconds in enumerate((0.0, 0.25, 0.5)):
        clock.monotonic.return_value = seconds
        msg = Octomap(binary=True, id="OcTree", resolution=0.01, data=[index])
        msg.header.frame_id = "base_link"
        msg.header.stamp.sec = 42 + index
        messages.append(msg)
        relay._octomap_callback(msg)
        assert len(published) == (1 if index < 2 else 2)

    for result, msg in zip(published, (messages[0], messages[2])):
        expected = PlanningScene()
        expected.is_diff = True
        expected.world.octomap = OctomapWithPose(header=msg.header, octomap=msg)
        expected.world.octomap.origin = Pose()
        expected.world.octomap.origin.orientation.w = 1.0
        assert result == expected
    assert relay._updates_sent == 2
    assert relay._updates_throttled == 1
    assert relay._last_update_time == 0.5
    assert logger.info.call_count == (2 if logs else 0)
    clock.time.assert_not_called()
