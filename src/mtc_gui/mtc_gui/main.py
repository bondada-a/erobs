#!/usr/bin/env python3
"""MTC GUI Client — PyQt5 entry point."""

import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QPalette, QColor
from PyQt5.QtCore import Qt

from .ros2_bridge import ROS2Bridge, ROS2_AVAILABLE


_dark_palette = None

def apply_dark_theme(app):
    global _dark_palette
    p = QPalette()
    p.setColor(QPalette.Window, QColor(53, 53, 53))
    p.setColor(QPalette.WindowText, Qt.white)
    p.setColor(QPalette.Base, QColor(35, 35, 35))
    p.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    p.setColor(QPalette.ToolTipBase, QColor(25, 25, 25))
    p.setColor(QPalette.ToolTipText, Qt.white)
    p.setColor(QPalette.Text, Qt.white)
    p.setColor(QPalette.Button, QColor(53, 53, 53))
    p.setColor(QPalette.ButtonText, Qt.white)
    p.setColor(QPalette.BrightText, Qt.red)
    p.setColor(QPalette.Link, QColor(42, 130, 218))
    p.setColor(QPalette.Highlight, QColor(42, 130, 218))
    p.setColor(QPalette.HighlightedText, QColor(35, 35, 35))
    p.setColor(QPalette.Disabled, QPalette.Text, QColor(127, 127, 127))
    p.setColor(QPalette.Disabled, QPalette.ButtonText, QColor(127, 127, 127))
    _dark_palette = p
    app.setPalette(p)


def toggle_dark_mode(app, enabled):
    if enabled:
        if _dark_palette:
            app.setPalette(_dark_palette)
    else:
        app.setPalette(app.style().standardPalette())


def main():
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
