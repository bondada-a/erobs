#!/usr/bin/env python3
"""MTC GUI Client — PyQt6 entry point."""

import sys
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt

# QtWebEngine must be imported before QApplication is created
try:
    from PyQt6 import QtWebEngineWidgets  # noqa: F401
except ImportError:
    pass

from .ros2_bridge import ROS2Bridge, ROS2_AVAILABLE
from . import theme


def apply_dark_theme(app):
    """Backward-compatible name — applies the centralized theme."""
    theme.apply(app)


def toggle_dark_mode(app, enabled):
    """Light mode is not currently themed; dark theme is always applied.
    Kept for API compatibility with callers."""
    if enabled:
        theme.apply(app)
    else:
        # Fallback to platform default if explicitly disabled.
        app.setPalette(app.style().standardPalette())
        app.setStyleSheet("")


def main():
    # Required for QtWebEngine in Qt6 — must be set before QApplication is constructed
    QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
    app = QApplication(sys.argv)
    theme.apply(app)

    # ROS2 bridge (works without ROS2 — UI still functional)
    ros2 = ROS2Bridge()
    if ROS2_AVAILABLE:
        ros2.init_ros2()

    # Import here to avoid circular imports
    from .main_window import MTCMainWindow
    window = MTCMainWindow(ros2)
    window.show()

    ret = app.exec()
    ros2.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
