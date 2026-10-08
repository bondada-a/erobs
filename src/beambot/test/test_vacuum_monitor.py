"""Offline checks for vacuum-loss state and warning suppression."""

from types import SimpleNamespace
from unittest.mock import Mock

from beambot.utils.vacuum_monitor import VacuumMonitor


def test_loss_warns_once_until_rearmed():
    node = Mock()
    monitor = VacuumMonitor(
        node, {"epick": {"states": {"grasp": "vacuum_on", "release": "vacuum_off"}}},
    )
    grasp = [{"task_type": "end_effector", "end_effector_action": "vacuum_on"}]
    release = [{"task_type": "end_effector", "end_effector_action": "vacuum_off"}]
    empty = SimpleNamespace(status=3)
    held = SimpleNamespace(status=2)

    node.create_subscription.assert_not_called()
    assert monitor._sub is None
    monitor._on_status(empty)
    assert not monitor.lost
    node.get_logger().warning.assert_not_called()

    for cycle in range(2):
        monitor._on_status(held)
        monitor.update_after_tasks(grasp, "epick")
        assert monitor.armed and not monitor.lost
        for _ in range(10):
            monitor._on_status(empty)
        assert node.get_logger().warning.call_count == cycle + 1
        assert monitor.lost

        monitor._on_status(held)
        assert monitor.status == 2 and monitor.lost
        assert monitor.check_lost().startswith("VACUUM_LOST:")
        assert monitor.check_lost() is None
        monitor.update_after_tasks(release, "epick")
        assert not monitor.armed and not monitor.lost

    # A loss already reported during arming must not be logged again by status updates.
    monitor._on_status(empty)
    monitor.update_after_tasks(grasp, "epick")
    assert node.get_logger().warning.call_count == 3
    monitor._on_status(empty)
    assert node.get_logger().warning.call_count == 3
    assert monitor.check_lost().startswith("VACUUM_LOST:")
