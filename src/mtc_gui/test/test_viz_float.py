"""Tests for the viz-panel float/dock state machine.

Exercises the removeTab→setParent→insertTab sequence on real Qt widgets
without importing MTCMainWindow (which pulls ROS dependencies).
"""

import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest  # noqa: E402

pytest.importorskip("PyQt6.QtWidgets", reason="PyQt6 not installed")

from PyQt6.QtWidgets import QApplication, QTabWidget, QWidget  # noqa: E402
from PyQt6.QtCore import Qt, QRect  # noqa: E402

_app = QApplication.instance() or QApplication([])

WEBENGINE_AVAILABLE = True


def _float_viz_for_form(self):
    """Extracted float logic matching main_window._float_viz_for_form."""
    if not (WEBENGINE_AVAILABLE and hasattr(self, "viz_panel")):
        return
    if self._viz_floating:
        return
    self._viz_home_index = self.right_tabs.indexOf(self.viz_panel)
    if self._viz_home_index >= 0:
        self.right_tabs.removeTab(self._viz_home_index)
    self.viz_panel.setParent(None)
    self.viz_panel.setWindowFlags(
        Qt.WindowType.Tool
        | Qt.WindowType.WindowStaysOnTopHint
        | Qt.WindowType.WindowTitleHint
    )
    self.viz_panel.setWindowTitle("Goal Preview")
    geo = self.frameGeometry()
    x = geo.x() + geo.width() + 8
    y = geo.y()
    screen = QApplication.primaryScreen()
    if screen:
        avail = screen.availableGeometry()
        x = min(x, avail.right() - 480)
        y = max(y, avail.top())
    self.viz_panel.setGeometry(x, y, 480, 480)
    self.viz_panel.show()
    self.viz_panel.raise_()
    self._viz_floating = True


def _dock_viz_after_form(self):
    """Extracted dock logic matching main_window._dock_viz_after_form."""
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

    def frameGeometry(self):
        return QRect(100, 100, 800, 600)

    _float_viz_for_form = _float_viz_for_form
    _dock_viz_after_form = _dock_viz_after_form


def test_float_removes_from_tab_and_sets_toplevel():
    w = FakeWindow()
    assert w.right_tabs.indexOf(w.viz_panel) == 1
    w._float_viz_for_form()
    assert w.right_tabs.indexOf(w.viz_panel) == -1
    assert w.viz_panel.parent() is None
    assert w._viz_floating is True


def test_dock_returns_panel_to_tab():
    w = FakeWindow()
    w._float_viz_for_form()
    w._dock_viz_after_form()
    assert w.right_tabs.indexOf(w.viz_panel) >= 0
    assert w._viz_floating is False


def test_dock_restores_original_index():
    w = FakeWindow()
    original_idx = w.right_tabs.indexOf(w.viz_panel)
    w._float_viz_for_form()
    w._dock_viz_after_form()
    assert w.right_tabs.indexOf(w.viz_panel) == original_idx


def test_double_float_is_noop():
    w = FakeWindow()
    w._float_viz_for_form()
    w._float_viz_for_form()
    assert w._viz_floating is True
    assert w.right_tabs.indexOf(w.viz_panel) == -1


def test_dock_without_float_is_noop():
    w = FakeWindow()
    original_idx = w.right_tabs.indexOf(w.viz_panel)
    w._dock_viz_after_form()
    assert w._viz_floating is False
    assert w.right_tabs.indexOf(w.viz_panel) == original_idx
