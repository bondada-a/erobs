"""Tests for action-server goal admission."""

import threading
from types import SimpleNamespace
from unittest.mock import Mock

from rclpy.action import GoalResponse

from beambot.action_servers.base_action_server import BaseActionServer


def test_goal_admission_reserves_server_immediately():
    server = SimpleNamespace(
        _executing=False,
        _lock=threading.Lock(),
        get_logger=Mock(return_value=Mock()),
    )

    assert BaseActionServer._goal_callback(server, None) == GoalResponse.ACCEPT
    assert BaseActionServer._goal_callback(server, None) == GoalResponse.REJECT
