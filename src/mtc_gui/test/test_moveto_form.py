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


if __name__ == "__main__":
    test_cartesian_mode_emits_only_cartesian_keys()
    test_relative_mode_emits_only_relative_keys()
    test_named_mode_emits_only_target()
    test_ambiguous_step_defaults_to_relative()
    print("All MoveToForm tests passed.")
