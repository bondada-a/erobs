"""Centralized design tokens + QSS for mtc_gui.

Translated from the pyqt6-ui-designer agent skill
(https://github.com/aminechraibi/pyqt6-ui-designer) with a few principles
borrowed from Leonxlnx/taste-skill (no slop, intentional motion-states,
4px grid, semantic color hierarchy). Robotics-tuned: dark by default,
cool indigo accent, clear semantic states for planning / success /
warning / error.

Single source of truth — do not inline hex values in widget code.
Tag widgets with objectName or dynamic property `class` to opt into
a styled variant (e.g. ``btn.setProperty("class", "primary")``).
"""

from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QPalette
from PyQt6.QtWidgets import QApplication, QWidget

# ─── Tokens ──────────────────────────────────────────────────────────────────

# Surfaces (dark, robotics-tuned — deep navy-charcoal canvas)
BG = "#0E1117"               # canvas / QMainWindow background
SURFACE = "#161B24"          # default panels, splitters
SURFACE_LOW = "#1B2230"      # cards, group boxes, list rows
SURFACE_HIGH = "#222A3A"     # hovered rows, secondary buttons
SURFACE_HIGHEST = "#2C3548"  # pressed / active emphasis
ELEVATED = "#1E2533"         # popovers, menus, dropdowns

# Borders / dividers
OUTLINE = "#2C3448"          # subtle dividers (default)
OUTLINE_STRONG = "#3D4660"   # input borders, group-box borders

# Text
ON_SURFACE = "#E6EAF2"       # primary text
ON_SURFACE_MUTED = "#A0A8BC" # secondary / labels
ON_SURFACE_DIM = "#6B7385"   # placeholders / timestamps / disabled-ish
DISABLED = "#5A6378"

# Brand accent (cool indigo — distinct from ROS-stock blue, calmer)
PRIMARY = "#5B8DEF"
PRIMARY_HOVER = "#7AA4F4"
PRIMARY_PRESSED = "#3F70D8"
PRIMARY_TINT = "#1F2C45"     # subtle accent-tinted background
ON_PRIMARY = "#0B0F18"

# Semantic
SUCCESS = "#3DD68C"          # planning cached / OK / connected
SUCCESS_DIM = "#1F4A35"
WARNING = "#F2B33D"          # dry-run / paused / waiting
WARNING_DIM = "#4A3A18"
DANGER = "#F26B6B"           # error / disconnected / aborted
DANGER_DIM = "#4A1F1F"

# Type
FONT_BODY = "Inter, 'Segoe UI', 'Helvetica Neue', Arial, sans-serif"
FONT_MONO = "'JetBrains Mono', 'Fira Code', 'DejaVu Sans Mono', monospace"

# Spacing (4px grid — never deviate)
SP_XS = 4
SP_SM = 8
SP_MD = 12
SP_LG = 16
SP_XL = 24

# Radius
R_SM = 4
R_MD = 6
R_LG = 8
R_PILL = 999


# ─── QSS ─────────────────────────────────────────────────────────────────────

DARK_QSS = f"""
/* ─── Base ─────────────────────────────────────────────────────────── */
QWidget {{
    background-color: {BG};
    color: {ON_SURFACE};
    font-family: {FONT_BODY};
    font-size: 13px;
}}

QMainWindow, QDialog {{
    background-color: {BG};
}}

QToolTip {{
    background-color: {ELEVATED};
    color: {ON_SURFACE};
    border: 1px solid {OUTLINE_STRONG};
    border-radius: {R_SM}px;
    padding: 6px 8px;
}}

/* ─── Menu bar ─────────────────────────────────────────────────────── */
QMenuBar {{
    background-color: {BG};
    color: {ON_SURFACE_MUTED};
    border-bottom: 1px solid {OUTLINE};
    padding: 2px 4px;
}}
QMenuBar::item {{
    background: transparent;
    padding: 6px 12px;
    border-radius: {R_SM}px;
    color: {ON_SURFACE_MUTED};
}}
QMenuBar::item:selected {{
    background-color: {SURFACE_HIGH};
    color: {ON_SURFACE};
}}
QMenu {{
    background-color: {ELEVATED};
    border: 1px solid {OUTLINE_STRONG};
    border-radius: {R_MD}px;
    padding: 4px;
    color: {ON_SURFACE};
}}
QMenu::item {{
    padding: 6px 16px;
    border-radius: {R_SM}px;
}}
QMenu::item:selected {{
    background-color: {PRIMARY_TINT};
    color: {ON_SURFACE};
}}
QMenu::separator {{
    height: 1px;
    background: {OUTLINE};
    margin: 4px 6px;
}}

/* ─── Buttons ──────────────────────────────────────────────────────── */
QPushButton {{
    background-color: {SURFACE_HIGH};
    color: {ON_SURFACE};
    border: 1px solid {OUTLINE_STRONG};
    border-radius: {R_SM}px;
    padding: 6px 14px;
    font-weight: 500;
    min-height: 22px;
}}
QPushButton:hover {{
    background-color: {SURFACE_HIGHEST};
    border-color: {PRIMARY};
}}
QPushButton:pressed {{
    background-color: {SURFACE_LOW};
}}
QPushButton:disabled {{
    background-color: {SURFACE_LOW};
    color: {DISABLED};
    border-color: {OUTLINE};
}}
QPushButton:flat {{
    background: transparent;
    border: 1px solid transparent;
    color: {ON_SURFACE_MUTED};
}}
QPushButton:flat:hover {{
    background-color: {SURFACE_HIGH};
    color: {ON_SURFACE};
    border-color: {OUTLINE};
}}

/* Variants — opt in via setProperty("class", "primary") etc. */
QPushButton[class="primary"] {{
    background-color: {PRIMARY};
    color: {ON_PRIMARY};
    border: 1px solid {PRIMARY};
    font-weight: 600;
}}
QPushButton[class="primary"]:hover {{
    background-color: {PRIMARY_HOVER};
    border-color: {PRIMARY_HOVER};
}}
QPushButton[class="primary"]:pressed {{
    background-color: {PRIMARY_PRESSED};
}}
QPushButton[class="primary"]:disabled {{
    background-color: {SURFACE_LOW};
    color: {DISABLED};
    border-color: {OUTLINE};
}}
QPushButton[class="danger"] {{
    background-color: {DANGER_DIM};
    color: {DANGER};
    border: 1px solid {DANGER};
}}
QPushButton[class="danger"]:hover {{
    background-color: {DANGER};
    color: {ON_PRIMARY};
}}

/* ─── Inputs ───────────────────────────────────────────────────────── */
QLineEdit, QTextEdit, QPlainTextEdit, QSpinBox, QDoubleSpinBox {{
    background-color: {SURFACE_LOW};
    color: {ON_SURFACE};
    border: 1px solid {OUTLINE_STRONG};
    border-radius: {R_SM}px;
    padding: 5px 10px;
    selection-background-color: {PRIMARY};
    selection-color: {ON_PRIMARY};
}}
QLineEdit:hover, QTextEdit:hover, QPlainTextEdit:hover,
QSpinBox:hover, QDoubleSpinBox:hover {{
    border-color: {OUTLINE_STRONG};
    background-color: {SURFACE};
}}
QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus,
QSpinBox:focus, QDoubleSpinBox:focus {{
    border: 1px solid {PRIMARY};
    background-color: {SURFACE};
}}
QLineEdit:disabled, QTextEdit:disabled, QPlainTextEdit:disabled {{
    background-color: {BG};
    color: {DISABLED};
    border-color: {OUTLINE};
}}
QLineEdit::placeholder {{
    color: {ON_SURFACE_DIM};
}}

/* ─── Combo box ────────────────────────────────────────────────────── */
QComboBox {{
    background-color: {SURFACE_LOW};
    color: {ON_SURFACE};
    border: 1px solid {OUTLINE_STRONG};
    border-radius: {R_SM}px;
    padding: 5px 10px;
    min-height: 22px;
}}
QComboBox:hover {{
    background-color: {SURFACE};
    border-color: {OUTLINE_STRONG};
}}
QComboBox:focus {{
    border-color: {PRIMARY};
}}
QComboBox::drop-down {{
    width: 22px;
    border: none;
    background: transparent;
}}
QComboBox::down-arrow {{
    image: none;
    width: 0; height: 0;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid {ON_SURFACE_MUTED};
    margin-right: 8px;
}}
QComboBox QAbstractItemView {{
    background-color: {ELEVATED};
    border: 1px solid {OUTLINE_STRONG};
    border-radius: {R_MD}px;
    padding: 4px;
    outline: none;
    selection-background-color: {PRIMARY_TINT};
    selection-color: {ON_SURFACE};
}}

/* ─── Checkboxes / Radios ──────────────────────────────────────────── */
QCheckBox, QRadioButton {{
    color: {ON_SURFACE};
    spacing: 8px;
    padding: 4px 0;
}}
QCheckBox:disabled, QRadioButton:disabled {{
    color: {DISABLED};
}}
QCheckBox::indicator, QRadioButton::indicator {{
    width: 16px; height: 16px;
    border: 1px solid {OUTLINE_STRONG};
    background-color: {SURFACE_LOW};
}}
QCheckBox::indicator {{
    border-radius: 3px;
}}
QRadioButton::indicator {{
    border-radius: 8px;
}}
QCheckBox::indicator:hover, QRadioButton::indicator:hover {{
    border-color: {PRIMARY};
}}
QCheckBox::indicator:checked, QRadioButton::indicator:checked {{
    background-color: {PRIMARY};
    border-color: {PRIMARY};
}}

/* ─── Group box ────────────────────────────────────────────────────── */
QGroupBox {{
    background-color: {SURFACE};
    border: 1px solid {OUTLINE};
    border-radius: {R_MD}px;
    margin-top: 14px;
    padding: 12px;
    font-weight: 500;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    padding: 0 6px;
    color: {ON_SURFACE_MUTED};
    background-color: {SURFACE};
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.5px;
    text-transform: uppercase;
}}

/* ─── Tabs ─────────────────────────────────────────────────────────── */
QTabWidget::pane {{
    background-color: {SURFACE};
    border: 1px solid {OUTLINE};
    border-radius: {R_MD}px;
    top: -1px;
}}
QTabBar {{
    background: transparent;
    qproperty-drawBase: 0;
}}
QTabBar::tab {{
    background-color: transparent;
    color: {ON_SURFACE_MUTED};
    padding: 8px 16px;
    border: 1px solid transparent;
    border-bottom: 1px solid {OUTLINE};
    margin-right: 2px;
}}
QTabBar::tab:hover {{
    color: {ON_SURFACE};
}}
QTabBar::tab:selected {{
    background-color: {SURFACE};
    color: {ON_SURFACE};
    border: 1px solid {OUTLINE};
    border-bottom: 1px solid {SURFACE};
    border-top-left-radius: {R_SM}px;
    border-top-right-radius: {R_SM}px;
}}

/* ─── List / Tree ──────────────────────────────────────────────────── */
QListWidget, QTreeWidget, QTreeView, QListView {{
    background-color: {SURFACE};
    color: {ON_SURFACE};
    border: 1px solid {OUTLINE};
    border-radius: {R_MD}px;
    outline: none;
    padding: 4px;
}}
QListWidget::item, QTreeWidget::item {{
    padding: 6px 8px;
    border-radius: {R_SM}px;
}}
QListWidget::item:hover, QTreeWidget::item:hover {{
    background-color: {SURFACE_HIGH};
}}
QListWidget::item:selected, QTreeWidget::item:selected {{
    background-color: {PRIMARY_TINT};
    color: {ON_SURFACE};
}}

/* ─── Splitter handle ──────────────────────────────────────────────── */
QSplitter::handle {{
    background-color: {OUTLINE};
}}
QSplitter::handle:horizontal {{
    width: 1px;
}}
QSplitter::handle:vertical {{
    height: 1px;
}}
QSplitter::handle:hover {{
    background-color: {PRIMARY};
}}

/* ─── Progress bar ─────────────────────────────────────────────────── */
QProgressBar {{
    background-color: {SURFACE_LOW};
    color: {ON_SURFACE};
    border: 1px solid {OUTLINE};
    border-radius: {R_SM}px;
    text-align: center;
    font-size: 11px;
    height: 16px;
}}
QProgressBar::chunk {{
    background-color: {PRIMARY};
    border-radius: 3px;
    margin: 1px;
}}

/* ─── Scrollbars ───────────────────────────────────────────────────── */
QScrollBar:vertical {{
    background: transparent;
    width: 10px;
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background: {OUTLINE_STRONG};
    border-radius: 4px;
    min-height: 28px;
    margin: 2px;
}}
QScrollBar::handle:vertical:hover {{
    background: {ON_SURFACE_DIM};
}}
QScrollBar:horizontal {{
    background: transparent;
    height: 10px;
    margin: 0;
}}
QScrollBar::handle:horizontal {{
    background: {OUTLINE_STRONG};
    border-radius: 4px;
    min-width: 28px;
    margin: 2px;
}}
QScrollBar::handle:horizontal:hover {{
    background: {ON_SURFACE_DIM};
}}
QScrollBar::add-line, QScrollBar::sub-line {{
    width: 0; height: 0;
    background: transparent;
}}
QScrollBar::add-page, QScrollBar::sub-page {{
    background: transparent;
}}

/* ─── Frame / status log ───────────────────────────────────────────── */
QFrame {{
    background: transparent;
}}
QLabel {{
    background: transparent;
    color: {ON_SURFACE};
}}

/* ─── Header views (tables / trees) ────────────────────────────────── */
QHeaderView::section {{
    background-color: {SURFACE};
    color: {ON_SURFACE_MUTED};
    border: none;
    border-bottom: 1px solid {OUTLINE};
    padding: 6px 10px;
    font-size: 11px;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}}

/* ─── Status pills (for plan_cached_label etc) ─────────────────────── */
QLabel[status="success"] {{
    color: {SUCCESS};
    background-color: {SUCCESS_DIM};
    border: 1px solid {SUCCESS};
    border-radius: {R_PILL}px;
    padding: 2px 10px;
    font-size: 11px;
    font-weight: 600;
}}
QLabel[status="warning"] {{
    color: {WARNING};
    background-color: {WARNING_DIM};
    border: 1px solid {WARNING};
    border-radius: {R_PILL}px;
    padding: 2px 10px;
    font-size: 11px;
    font-weight: 600;
}}
QLabel[status="danger"] {{
    color: {DANGER};
    background-color: {DANGER_DIM};
    border: 1px solid {DANGER};
    border-radius: {R_PILL}px;
    padding: 2px 10px;
    font-size: 11px;
    font-weight: 600;
}}
QLabel[role="section"] {{
    color: {ON_SURFACE_MUTED};
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 1px;
    text-transform: uppercase;
    padding: 4px 0;
}}
QLabel[role="hint"] {{
    color: {ON_SURFACE_DIM};
    font-size: 11px;
}}
"""


def _palette() -> QPalette:
    """QPalette aligned with the dark QSS — covers things QSS misses
    (native dialog widgets, rich-text rendering, default disabled colors)."""
    p = QPalette()
    p.setColor(QPalette.ColorRole.Window, QColor(BG))
    p.setColor(QPalette.ColorRole.WindowText, QColor(ON_SURFACE))
    p.setColor(QPalette.ColorRole.Base, QColor(SURFACE_LOW))
    p.setColor(QPalette.ColorRole.AlternateBase, QColor(SURFACE))
    p.setColor(QPalette.ColorRole.ToolTipBase, QColor(ELEVATED))
    p.setColor(QPalette.ColorRole.ToolTipText, QColor(ON_SURFACE))
    p.setColor(QPalette.ColorRole.Text, QColor(ON_SURFACE))
    p.setColor(QPalette.ColorRole.Button, QColor(SURFACE_HIGH))
    p.setColor(QPalette.ColorRole.ButtonText, QColor(ON_SURFACE))
    p.setColor(QPalette.ColorRole.BrightText, QColor(DANGER))
    p.setColor(QPalette.ColorRole.Link, QColor(PRIMARY))
    p.setColor(QPalette.ColorRole.Highlight, QColor(PRIMARY))
    p.setColor(QPalette.ColorRole.HighlightedText, QColor(ON_PRIMARY))
    p.setColor(QPalette.ColorRole.PlaceholderText, QColor(ON_SURFACE_DIM))
    for role in (QPalette.ColorRole.Text, QPalette.ColorRole.ButtonText,
                 QPalette.ColorRole.WindowText):
        p.setColor(QPalette.ColorGroup.Disabled, role, QColor(DISABLED))
    return p


def apply(app: QApplication) -> None:
    """Apply the dark theme + QSS to the running application."""
    app.setStyle("Fusion")  # consistent baseline across platforms
    app.setPalette(_palette())
    app.setStyleSheet(DARK_QSS)


def restyle(widget: QWidget) -> None:
    """Force a widget to re-evaluate property-based selectors after a
    setProperty() call. Use after setProperty('class', ...) etc."""
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()
