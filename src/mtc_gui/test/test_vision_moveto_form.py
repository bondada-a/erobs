"""Headless check for VisionMoveTo detector-specific options."""

import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest  # noqa: E402

pytest.importorskip("PyQt6.QtWidgets", reason="PyQt6 not installed")

from PyQt6.QtWidgets import QApplication  # noqa: E402

from mtc_gui.task_forms import VisionMoveToForm  # noqa: E402

_app = QApplication.instance() or QApplication([])


def test_sample_roi_options_are_shown_and_saved_exclusively():
    form = VisionMoveToForm(
        {"task_type": "vision_moveto", "detection_type": "marker"}, 0, {}
    )
    assert form.marker_dict.isVisibleTo(form)
    assert not form.sample_roi_group.isVisibleTo(form)

    form.det_type.setCurrentText("sample_roi")
    form.strategy.setCurrentText("nearest_edge")
    form.edge_inset.setValue(4.25)

    assert not form.marker_dict.isVisibleTo(form)
    assert form.sample_roi_group.isVisibleTo(form)
    result = form.collect_values()
    assert result["strategy"] == "nearest_edge"
    assert result["edge_inset_mm"] == 4.25
    assert "marker_dictionary" not in result

    form.det_type.setCurrentText("marker")
    result = form.collect_values()
    assert "marker_dictionary" in result
    assert "strategy" not in result
    assert "edge_inset_mm" not in result
