"""Focused checks for lifecycle joint-state readiness."""

import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

from beambot.core.moveit_lifecycle_manager import MoveItLifecycleManager


def _message(stamp_ns, names, positions):
    """Build the JointState fields used by the readiness callback."""
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            )
        ),
        name=names,
        position=positions,
    )


def _manager():
    """Build a lifecycle manager without creating ROS entities."""
    manager = MoveItLifecycleManager.__new__(MoveItLifecycleManager)
    manager._arm_joints_cache = frozenset(
        {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        }
    )
    manager._logger = MagicMock()
    manager._node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=200)
        )
    )
    manager._joint_state_lock = threading.Lock()
    manager._joint_positions = {}
    manager._joint_attempt_started_ns = 100
    manager._joint_state_errors = {
        "stale": set(),
        "malformed": set(),
        "non_finite": set(),
    }
    return manager


def test_joint_state_readiness_requires_fresh_complete_valid_data():
    """Accept all-zero state and reject stale, malformed, or non-finite data."""
    manager = _manager()
    joints = sorted(manager.ARM_JOINTS)

    manager._joint_state_cb(_message(100, joints + ["extra"], [0.0] * 7))
    assert set(manager._joint_positions) == manager.ARM_JOINTS

    manager._joint_positions.clear()
    manager._joint_state_cb(_message(99, joints, [1.0] * 6))
    manager._joint_state_cb(_message(0, joints, [1.0] * 6))
    assert not manager._joint_positions
    assert manager._joint_state_errors["stale"] == manager.ARM_JOINTS

    manager._joint_state_cb(_message(100, joints, [1.0] * 5))
    assert manager._joint_state_errors["malformed"] == manager.ARM_JOINTS

    manager._joint_state_cb(
        _message(100, joints, [float("nan"), float("inf")] + [1.0] * 4)
    )
    assert len(manager._joint_positions) == 4
    assert manager._joint_state_errors["non_finite"] == set(joints[:2])

    manager._joint_attempt_started_ns = None

    def publish_after_attempt_starts():
        while True:
            with manager._joint_state_lock:
                boundary_ns = manager._joint_attempt_started_ns
            if boundary_ns is not None:
                manager._joint_state_cb(
                    _message(boundary_ns, joints, [0.0] * 6)
                )
                return
            time.sleep(0.001)

    publisher = threading.Thread(
        target=publish_after_attempt_starts,
        daemon=True,
    )
    publisher.start()
    assert manager._verify_hardware_connected(timeout_sec=0.5)
    publisher.join()
