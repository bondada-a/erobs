"""Headless tests for MoveToForm mode selector (#93).

Verifies that the mode selector prevents ambiguous goals by emitting only
the active mode's keys in collect_values().
"""

import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest  # noqa: E402

pytest.importorskip("PyQt6.QtWidgets", reason="PyQt6 not installed")

from PyQt6.QtWidgets import QApplication  # noqa: E402

from mtc_gui.task_forms import MoveToForm  # noqa: E402

_app = QApplication.instance() or QApplication([])


def _make_form(step=None):
    step = step or {"task_type": "move_to"}
    return MoveToForm(step, 0, {})


def test_cartesian_mode_emits_only_cartesian_keys():
    form = _make_form()
    form.mode_combo.setCurrentText("Cartesian target")
    form.cart_x.setValue(0.5)
    form.cart_y.setValue(0.1)
    form.cart_z.setValue(0.3)
    result = form.collect_values()
    assert "cartesian_target" in result
    assert "frame_id" in result
    assert "planning_type" in result
    assert "direction" not in result
    assert "distance" not in result
    assert "target" not in result


def test_relative_mode_emits_only_relative_keys():
    form = _make_form()
    form.mode_combo.setCurrentText("Relative move")
    form.direction.setCurrentText("up")
    form.distance.setValue(0.1)
    result = form.collect_values()
    assert result["direction"] == "up"
    assert result["distance"] == 0.1
    assert "cartesian_target" not in result
    assert "frame_id" not in result
    assert "target" not in result


def test_named_mode_emits_only_target():
    form = _make_form({"task_type": "move_to", "target": "hotplate"})
    form.mode_combo.setCurrentText("Named target")
    result = form.collect_values()
    assert result["target"] == "hotplate"
    assert "direction" not in result
    assert "distance" not in result
    assert "cartesian_target" not in result
    assert "frame_id" not in result


def test_ambiguous_step_defaults_to_relative():
    """Backend precedence: relative > cartesian > named. A step with both
    direction+distance AND cartesian_target should default to Relative mode."""
    step = {
        "task_type": "move_to",
        "direction": "forward",
        "distance": 0.05,
        "cartesian_target": [0.1, 0.2, 0.3],
        "target": "foo",
    }
    form = _make_form(step)
    assert form.mode_combo.currentText() == "Relative move"
    result = form.collect_values()
    assert "direction" in result
    assert "cartesian_target" not in result


def test_joint_mode_emits_target_and_inline_pose():
    """Selecting Joint values, setting 6 spinboxes → step has target + _inline_joint_pose."""
    form = MoveToForm({"task_type": "move_to"}, 2, {})
    form.mode_combo.setCurrentText("Joint values")
    angles = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    for spin, val in zip(form.joint_spins, angles):
        spin.setValue(val)
    result = form.collect_values()
    assert result["target"] == "moveto_joints_3"
    assert result["_inline_joint_pose"]["name"] == "moveto_joints_3"
    assert result["_inline_joint_pose"]["values"] == angles
    assert "direction" not in result
    assert "distance" not in result
    assert "cartesian_target" not in result


def test_joint_mode_detected_from_poses():
    """Step with target pointing at a 6-element poses entry → defaults to Joint values."""
    poses = {"my_joint_pose": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]}
    step = {"task_type": "move_to", "target": "my_joint_pose"}
    form = MoveToForm(step, 0, poses)
    assert form.mode_combo.currentText() == "Joint values"
    # Spinboxes should be loaded with the pose values
    for spin, expected in zip(form.joint_spins, poses["my_joint_pose"]):
        assert spin.value() == expected
    # Collect preserves the name
    result = form.collect_values()
    assert result["target"] == "my_joint_pose"
    assert result["_inline_joint_pose"]["values"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_read_current_pose_fills_spinboxes():
    """Read current pose button fills spinboxes from provided current_pose."""
    pose = [11.0, 22.0, 33.0, 44.0, 55.0, 66.0]
    form = MoveToForm({"task_type": "move_to"}, 0, {}, current_pose=pose)
    form.mode_combo.setCurrentText("Joint values")
    form._read_current_pose()
    for spin, expected in zip(form.joint_spins, pose):
        assert spin.value() == expected


def test_read_current_pose_none_no_crash(monkeypatch):
    """Read current pose with no live pose warns and leaves spinboxes untouched."""
    # QMessageBox.warning is modal and blocks even under offscreen Qt, so patch
    # it out; we only care that the no-pose path is handled without filling.
    import mtc_gui.task_forms as tf

    warned = {}
    monkeypatch.setattr(
        tf.QMessageBox, "warning",
        lambda *a, **k: warned.setdefault("called", True),
    )
    form = MoveToForm({"task_type": "move_to"}, 0, {}, current_pose=None)
    form.mode_combo.setCurrentText("Joint values")
    form._read_current_pose()
    assert warned.get("called") is True
    # Spinboxes stay at default 0.0 — no live pose was injected.
    assert all(spin.value() == 0.0 for spin in form.joint_spins)


if __name__ == "__main__":
    test_cartesian_mode_emits_only_cartesian_keys()
    test_relative_mode_emits_only_relative_keys()
    test_named_mode_emits_only_target()
    test_ambiguous_step_defaults_to_relative()
    test_joint_mode_emits_target_and_inline_pose()
    test_joint_mode_detected_from_poses()
    test_read_current_pose_fills_spinboxes()
    print("All MoveToForm tests passed (run via pytest for the monkeypatch test).")
