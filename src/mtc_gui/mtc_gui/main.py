#!/usr/bin/env python3
"""MTC GUI Client — PyQt6 entry point."""

import sys
from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QPalette, QColor
from PyQt6.QtCore import Qt

# QtWebEngine must be imported before QApplication is created
try:
    from PyQt6 import QtWebEngineWidgets  # noqa: F401
except ImportError:
    pass

from .ros2_bridge import ROS2Bridge, ROS2_AVAILABLE


_dark_palette = None

def apply_dark_theme(app):
    global _dark_palette
    p = QPalette()
    p.setColor(QPalette.ColorRole.Window, QColor(53, 53, 53))
    p.setColor(QPalette.ColorRole.WindowText, QColor(Qt.GlobalColor.white))
    p.setColor(QPalette.ColorRole.Base, QColor(35, 35, 35))
    p.setColor(QPalette.ColorRole.AlternateBase, QColor(53, 53, 53))
    p.setColor(QPalette.ColorRole.ToolTipBase, QColor(25, 25, 25))
    p.setColor(QPalette.ColorRole.ToolTipText, QColor(Qt.GlobalColor.white))
    p.setColor(QPalette.ColorRole.Text, QColor(Qt.GlobalColor.white))
    p.setColor(QPalette.ColorRole.Button, QColor(53, 53, 53))
    p.setColor(QPalette.ColorRole.ButtonText, QColor(Qt.GlobalColor.white))
    p.setColor(QPalette.ColorRole.BrightText, QColor(Qt.GlobalColor.red))
    p.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
    p.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
    p.setColor(QPalette.ColorRole.HighlightedText, QColor(35, 35, 35))
    p.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, QColor(127, 127, 127))
    p.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, QColor(127, 127, 127))
    _dark_palette = p
    app.setPalette(p)


def toggle_dark_mode(app, enabled):
    if enabled:
        if _dark_palette:
            app.setPalette(_dark_palette)
    else:
        app.setPalette(app.style().standardPalette())


def main():
    # Required for QtWebEngine in Qt6 — must be set before QApplication is constructed
    QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
    app = QApplication(sys.argv)
    apply_dark_theme(app)

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
