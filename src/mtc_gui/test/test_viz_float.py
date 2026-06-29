"""Tests for the viz-panel detach/redock + embed state machine.

Exercises the removeTab→embed-in-dialog→setParent(None)→insertTab sequence
on real Qt widgets without importing MTCMainWindow (which pulls ROS deps).
"""

import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest  # noqa: E402

pytest.importorskip("PyQt6.QtWidgets", reason="PyQt6 not installed")

from PyQt6.QtWidgets import QApplication, QTabWidget, QWidget, QHBoxLayout  # noqa: E402
from PyQt6.QtCore import Qt  # noqa: E402

_app = QApplication.instance() or QApplication([])

WEBENGINE_AVAILABLE = True


def _detach_viz_for_form(self):
    """Extracted detach logic matching main_window._detach_viz_for_form."""
    if not (WEBENGINE_AVAILABLE and hasattr(self, "viz_panel")):
        return
    if self._viz_floating:
        return
    self._viz_home_index = self.right_tabs.indexOf(self.viz_panel)
    if self._viz_home_index >= 0:
        self.right_tabs.removeTab(self._viz_home_index)
    self._viz_floating = True


def _redock_viz_after_form(self):
    """Extracted redock logic matching main_window._redock_viz_after_form."""
    if not (WEBENGINE_AVAILABLE and hasattr(self, "viz_panel")):
        return
    if not self._viz_floating:
        return
    self.viz_panel.setWindowFlags(Qt.WindowType.Widget)
    idx = min(self._viz_home_index, self.right_tabs.count())
    self.right_tabs.insertTab(idx, self.viz_panel, self._viz_tab_title)
    self.viz_panel.show()
    self._viz_floating = False


class FakeWindow:
    """Minimal stand-in exposing the attributes the helpers use."""

    def __init__(self):
        self.right_tabs = QTabWidget()
        chat = QWidget()
        self.viz_panel = QWidget()
        self.right_tabs.addTab(chat, "Chat")
        self._viz_tab_title = "3D View"
        self.right_tabs.addTab(self.viz_panel, self._viz_tab_title)
        self._viz_floating = False
        self._viz_home_index = -1

    _detach_viz_for_form = _detach_viz_for_form
    _redock_viz_after_form = _redock_viz_after_form


class FakeDialog(QWidget):
    """Stand-in for a MoveToForm dialog with a _content_row layout."""

    def __init__(self):
        super().__init__()
        self._content_row = QHBoxLayout(self)


def test_detach_removes_from_tab():
    w = FakeWindow()
    assert w.right_tabs.indexOf(w.viz_panel) == 1
    w._detach_viz_for_form()
    assert w.right_tabs.indexOf(w.viz_panel) == -1
    assert w._viz_floating is True


def test_embed_into_dialog():
    w = FakeWindow()
    w._detach_viz_for_form()
    dialog = FakeDialog()
    w.viz_panel.setParent(None)
    dialog._content_row.addWidget(w.viz_panel)
    w.viz_panel.show()
    assert w.viz_panel.parent() is not None


def test_done_detaches_from_dialog():
    w = FakeWindow()
    w._detach_viz_for_form()
    dialog = FakeDialog()
    w.viz_panel.setParent(None)
    dialog._content_row.addWidget(w.viz_panel)
    w.viz_panel.show()
    # Simulate MoveToForm.done() detaching before dialog destruction
    w.viz_panel.setParent(None)
    assert w.viz_panel.parent() is None


def test_redock_returns_panel_to_tab():
    w = FakeWindow()
    original_idx = w.right_tabs.indexOf(w.viz_panel)
    w._detach_viz_for_form()
    # Simulate the form embedding and then done() detaching
    w.viz_panel.setParent(None)
    w._redock_viz_after_form()
    assert w.right_tabs.indexOf(w.viz_panel) == original_idx
    assert w._viz_floating is False


def test_double_detach_is_noop():
    w = FakeWindow()
    w._detach_viz_for_form()
    w._detach_viz_for_form()
    assert w._viz_floating is True
    assert w.right_tabs.indexOf(w.viz_panel) == -1


def test_redock_without_detach_is_noop():
    w = FakeWindow()
    original_idx = w.right_tabs.indexOf(w.viz_panel)
    w._redock_viz_after_form()
    assert w._viz_floating is False
    assert w.right_tabs.indexOf(w.viz_panel) == original_idx


def test_full_cycle_preserves_panel():
    """Full detach→embed→done→redock cycle: panel survives and returns home."""
    w = FakeWindow()
    original_idx = w.right_tabs.indexOf(w.viz_panel)
    panel_id = id(w.viz_panel)

    w._detach_viz_for_form()
    dialog = FakeDialog()
    w.viz_panel.setParent(None)
    dialog._content_row.addWidget(w.viz_panel)
    w.viz_panel.show()

    # done() detaches
    w.viz_panel.setParent(None)
    # dialog would be destroyed here in real code

    w._redock_viz_after_form()
    assert w.right_tabs.indexOf(w.viz_panel) == original_idx
    assert id(w.viz_panel) == panel_id
    assert w._viz_floating is False
