"""Step list panel: visual step-by-step progress indicator for task execution."""

import time
from enum import Enum, auto

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QAbstractItemView,
    QListWidget,
    QListWidgetItem,
    QLabel,
    QProgressBar,
    QFrame,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QSize
from PyQt6.QtGui import QFont


class StepState(Enum):
    PENDING = auto()
    RUNNING = auto()
    DONE = auto()
    FAILED = auto()


TASK_TYPE_CONFIG = {
    "moveto": {"icon": "→", "color": "#5c9ce6", "title": "Move To"},
    "pick_sample": {"icon": "⬆", "color": "#5ce6d4", "title": "Pick Sample"},
    "place_sample": {"icon": "⬇", "color": "#5ce6d4", "title": "Place Sample"},
    "vision_scan": {"icon": "◎", "color": "#5ce68a", "title": "Vision Scan"},
    "tool_exchange": {"icon": "⚙", "color": "#e6a05c", "title": "Tool Exchange"},
    "end_effector": {"icon": "✦", "color": "#a0a0a0", "title": "End Effector"},
    "vision_moveto": {"icon": "◉", "color": "#5ce68a", "title": "Vision MoveTo"},
    "pipettor": {"icon": "⬍", "color": "#b05ce6", "title": "Pipettor"},
}

STATUS_CONFIG = {
    StepState.PENDING: {"icon": "○", "color": "#888888"},
    StepState.RUNNING: {"icon": "◉", "color": "#e6a832"},
    StepState.DONE: {"icon": "✓", "color": "#4caf50"},
    StepState.FAILED: {"icon": "✗", "color": "#e05050"},
}

_ROW_HEIGHT = 52


class StepRowWidget(QFrame):
    """Single step row with status icon, type icon, title, subtitle, and timer."""

    def __init__(self, index: int, task_type: str, detail_text: str, parent=None):
        super().__init__(parent)
        self.index = index
        self.task_type = task_type
        self._state = StepState.PENDING
        self._start_time = None
        self._is_next = False

        self.setFixedHeight(_ROW_HEIGHT)

        type_cfg = TASK_TYPE_CONFIG.get(
            task_type, {"icon": "?", "color": "#888888", "title": task_type}
        )

        # Main horizontal layout
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 8, 0)
        main_layout.setSpacing(0)

        # Left border indicator (blue when running)
        self._left_border = QFrame()
        self._left_border.setFixedWidth(3)
        self._left_border.setStyleSheet("background-color: transparent;")
        main_layout.addWidget(self._left_border)

        # Step number
        self._num_label = QLabel(f"{index + 1:02d}")
        self._num_label.setFixedWidth(28)
        self._num_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._num_label.setFont(QFont("Monospace", 9))
        self._num_label.setStyleSheet("color: #888888; padding-left: 4px;")
        main_layout.addWidget(self._num_label)

        # Status icon
        self._status_icon = QLabel("○")
        self._status_icon.setFixedWidth(22)
        self._status_icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._status_icon.setFont(QFont("Sans", 11))
        self._status_icon.setStyleSheet("color: #888888;")
        main_layout.addWidget(self._status_icon)

        # Type icon (colored circle background)
        self._type_icon = QLabel(type_cfg["icon"])
        self._type_icon.setFixedSize(28, 28)
        self._type_icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._type_icon.setFont(QFont("Sans", 12))
        type_color = type_cfg["color"]
        r, g, b = (
            int(type_color[1:3], 16),
            int(type_color[3:5], 16),
            int(type_color[5:7], 16),
        )
        self._type_icon.setStyleSheet(
            f"color: {type_color}; background-color: rgba({r}, {g}, {b}, 0.13);"
            f" border-radius: 14px;"
        )
        main_layout.addWidget(self._type_icon)

        main_layout.addSpacing(8)

        # Title + subtitle column
        title_col = QVBoxLayout()
        title_col.setContentsMargins(0, 4, 0, 4)
        title_col.setSpacing(1)

        self._title_label = QLabel(type_cfg["title"])
        self._title_label.setFont(QFont("Sans", 10, QFont.Weight.Bold))
        self._title_label.setStyleSheet("color: #e0e0e0;")
        title_col.addWidget(self._title_label)

        self._subtitle_label = QLabel(detail_text)
        self._subtitle_label.setFont(QFont("Sans", 9))
        self._subtitle_label.setStyleSheet("color: #999999;")
        title_col.addWidget(self._subtitle_label)

        main_layout.addLayout(title_col, stretch=1)

        # Right column: NEXT badge / elapsed timer / done checkmark
        right_col = QVBoxLayout()
        right_col.setContentsMargins(0, 4, 0, 4)
        right_col.setSpacing(2)
        right_col.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        self._timer_label = QLabel("")
        self._timer_label.setFont(QFont("Monospace", 9))
        self._timer_label.setStyleSheet("color: #e6a832;")
        self._timer_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self._timer_label.hide()
        right_col.addWidget(self._timer_label)

        self._next_badge = QLabel("next")
        self._next_badge.setFont(QFont("Sans", 8, QFont.Weight.Bold))
        self._next_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._next_badge.setFixedSize(40, 18)
        self._next_badge.setStyleSheet(
            "color: #e6a832; background-color: #e6a83233;"
            " border-radius: 9px; padding: 1px 6px;"
        )
        self._next_badge.hide()
        right_col.addWidget(self._next_badge)

        self._done_check = QLabel("✓")
        self._done_check.setFont(QFont("Sans", 11))
        self._done_check.setStyleSheet("color: #4caf5088;")
        self._done_check.setAlignment(Qt.AlignmentFlag.AlignRight)
        self._done_check.hide()
        right_col.addWidget(self._done_check)

        main_layout.addLayout(right_col)

        self._apply_state_style()

    def set_state(self, state: StepState):
        self._state = state
        if state == StepState.RUNNING:
            self._start_time = time.monotonic()
            self._timer_label.setText("0:00")
            self._timer_label.show()
        else:
            self._timer_label.hide()

        if state == StepState.DONE:
            self._done_check.show()
        else:
            self._done_check.hide()

        # Update status icon
        cfg = STATUS_CONFIG[state]
        self._status_icon.setText(cfg["icon"])
        self._status_icon.setStyleSheet(f"color: {cfg['color']};")

        self._apply_state_style()

    def set_is_next(self, is_next: bool):
        self._is_next = is_next
        if is_next and self._state == StepState.PENDING:
            self._next_badge.show()
        else:
            self._next_badge.hide()

    def update_elapsed(self):
        if self._state == StepState.RUNNING and self._start_time is not None:
            elapsed = int(time.monotonic() - self._start_time)
            minutes = elapsed // 60
            seconds = elapsed % 60
            self._timer_label.setText(f"{minutes}:{seconds:02d}")

    def _apply_state_style(self):
        if self._state == StepState.RUNNING:
            self.setStyleSheet(
                "StepRowWidget { background-color: rgba(42, 130, 218, 0.12); }"
            )
            self._left_border.setStyleSheet("background-color: #2a82da;")
        elif self._state == StepState.FAILED:
            self.setStyleSheet(
                "StepRowWidget { background-color: rgba(224, 80, 80, 0.08); }"
            )
            self._left_border.setStyleSheet("background-color: #e05050;")
        else:
            self.setStyleSheet("StepRowWidget { background-color: transparent; }")
            self._left_border.setStyleSheet("background-color: transparent;")


class ExecutionToolbar(QFrame):
    """Compact execution info bar: step counter + progress + elapsed time."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(36)
        self.setStyleSheet(
            "ExecutionToolbar { background-color: #2a2a2a;"
            " border-bottom: 1px solid #3a3a3a; }"
        )

        layout = QHBoxLayout(self)
        layout.setContentsMargins(12, 4, 12, 4)
        layout.setSpacing(12)

        # Running indicator dot
        self._dot = QLabel("●")
        self._dot.setFont(QFont("Sans", 9))
        self._dot.setStyleSheet("color: #4caf50;")
        self._dot.setFixedWidth(14)
        layout.addWidget(self._dot)

        # Step counter + task name
        self._step_label = QLabel("Step 0/0")
        self._step_label.setFont(QFont("Sans", 9, QFont.Weight.Bold))
        self._step_label.setStyleSheet("color: #e0e0e0;")
        layout.addWidget(self._step_label)

        self._task_label = QLabel("")
        self._task_label.setFont(QFont("Sans", 9))
        self._task_label.setStyleSheet("color: #aaaaaa;")
        layout.addWidget(self._task_label)

        layout.addSpacing(8)

        # Mini progress bar
        self._progress = QProgressBar()
        self._progress.setFixedHeight(4)
        self._progress.setTextVisible(False)
        self._progress.setStyleSheet(
            "QProgressBar { background-color: #3a3a3a; border: none; border-radius: 2px; }"
            "QProgressBar::chunk { background-color: #5c9ce6; border-radius: 2px; }"
        )
        layout.addWidget(self._progress, stretch=1)

        layout.addSpacing(8)

        # Total elapsed time
        self._elapsed_label = QLabel("00:00")
        self._elapsed_label.setFont(QFont("Monospace", 9))
        self._elapsed_label.setStyleSheet("color: #aaaaaa;")
        self._elapsed_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(self._elapsed_label)

        self._start_time = None

    def start(self, total_steps: int):
        self._start_time = time.monotonic()
        self._progress.setMaximum(100)
        self._progress.setValue(0)
        self._step_label.setText(f"Step 1/{total_steps}")
        self._task_label.setText("")
        self._elapsed_label.setText("00:00")

    def update_progress(
        self, current_step: int, total_steps: int, task_name: str, progress_pct: float
    ):
        self._step_label.setText(f"Step {current_step}/{total_steps}")
        type_cfg = TASK_TYPE_CONFIG.get(task_name, {})
        display_name = type_cfg.get("title", task_name)
        self._task_label.setText(f"· {display_name}")
        self._progress.setValue(int(progress_pct))

    def update_elapsed(self):
        if self._start_time is not None:
            elapsed = int(time.monotonic() - self._start_time)
            minutes = elapsed // 60
            seconds = elapsed % 60
            self._elapsed_label.setText(f"{minutes:02d}:{seconds:02d}")

    def set_paused(self, paused: bool):
        if paused:
            self._dot.setText("⏸")
            self._dot.setStyleSheet("color: #e6a832;")
        else:
            self._dot.setText("●")
            self._dot.setStyleSheet("color: #4caf50;")

    def reset(self):
        self._start_time = None
        self._progress.setValue(0)
        self._step_label.setText("")
        self._task_label.setText("")
        self._elapsed_label.setText("00:00")


class StepListPanel(QWidget):
    """Full step list panel replacing the QTreeWidget.

    Manages a QListWidget of StepRowWidgets + ExecutionToolbar.
    """

    item_double_clicked = pyqtSignal(int)
    selection_changed = pyqtSignal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._step_rows: list[StepRowWidget] = []
        self._total_steps = 0
        self._current_step = -1
        self._execution_active = False
        self._editing_enabled = True

        self._build_ui()

        self._elapsed_timer = QTimer(self)
        self._elapsed_timer.setInterval(1000)
        self._elapsed_timer.timeout.connect(self._tick_elapsed)

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Execution toolbar (hidden in edit mode)
        self._exec_toolbar = ExecutionToolbar()
        self._exec_toolbar.hide()
        layout.addWidget(self._exec_toolbar)

        # Step list
        self._list_widget = QListWidget()
        self._list_widget.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self._list_widget.itemDoubleClicked.connect(self._on_double_click)
        self._list_widget.itemSelectionChanged.connect(self._on_selection_changed)
        self._list_widget.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self._list_widget.setStyleSheet(
            "QListWidget { background-color: #232323; border: none; outline: none; }"
            "QListWidget::item { border-bottom: 1px solid #2e2e2e; padding: 0; }"
            "QListWidget::item:selected { background-color: rgba(42, 130, 218, 0.08); }"
        )
        layout.addWidget(self._list_widget, stretch=1)

    # --- Public API ---

    def refresh(self, tasks: list, summary_fn):
        """Rebuild step list from task data."""
        self._list_widget.clear()
        self._step_rows.clear()
        self._current_step = -1

        for i, step in enumerate(tasks):
            task_type = step.get("task_type", "?")
            detail = summary_fn(step)
            row_widget = StepRowWidget(i, task_type, detail)

            item = QListWidgetItem(self._list_widget)
            item.setSizeHint(QSize(0, _ROW_HEIGHT))
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self._list_widget.setItemWidget(item, row_widget)
            self._step_rows.append(row_widget)

        self._total_steps = len(tasks)

    def start_execution(self, total_steps: int):
        """Enter execution mode."""
        self._execution_active = True
        self._editing_enabled = False
        self._total_steps = total_steps
        self._current_step = -1

        # Reset all rows to PENDING
        for row in self._step_rows:
            row.set_state(StepState.PENDING)
            row.set_is_next(False)

        # Mark first step as next
        if self._step_rows:
            self._step_rows[0].set_is_next(True)

        # Show toolbar and start timer
        self._exec_toolbar.start(total_steps)
        self._exec_toolbar.show()
        self._elapsed_timer.start()

    def update_step(self, current_step: int, progress: float, action_name: str):
        """Called on feedback. current_step is 1-indexed from orchestrator."""
        step_idx = current_step - 1
        if step_idx < 0 or step_idx >= len(self._step_rows):
            return

        # Mark all previous steps as DONE
        for i in range(step_idx):
            if self._step_rows[i]._state != StepState.DONE:
                self._step_rows[i].set_state(StepState.DONE)
                self._step_rows[i].set_is_next(False)

        # Mark current step as RUNNING
        if self._step_rows[step_idx]._state != StepState.RUNNING:
            self._step_rows[step_idx].set_state(StepState.RUNNING)
            self._step_rows[step_idx].set_is_next(False)

        # Mark next step with badge
        for i in range(step_idx + 1, len(self._step_rows)):
            self._step_rows[i].set_is_next(i == step_idx + 1)

        self._current_step = step_idx

        # Update toolbar
        self._exec_toolbar.update_progress(
            current_step, self._total_steps, action_name, progress
        )

        # Auto-scroll to active step
        item = self._list_widget.item(step_idx)
        if item:
            self._list_widget.scrollToItem(item)

    def finish_execution(self, status: str, completed_steps: int):
        """Called on result. Mark final states."""
        self._execution_active = False
        self._editing_enabled = True
        self._elapsed_timer.stop()

        for i, row in enumerate(self._step_rows):
            row.set_is_next(False)
            if i < completed_steps:
                row.set_state(StepState.DONE)
            elif i == completed_steps and status != "success":
                row.set_state(StepState.FAILED)
            elif status == "success":
                row.set_state(StepState.DONE)
            else:
                row.set_state(StepState.PENDING)

        self._exec_toolbar.hide()

    def reset_execution_state(self):
        """Return all steps to PENDING (edit mode)."""
        self._execution_active = False
        self._editing_enabled = True
        self._current_step = -1
        self._elapsed_timer.stop()
        self._exec_toolbar.hide()
        for row in self._step_rows:
            row.set_state(StepState.PENDING)
            row.set_is_next(False)

    def selected_indices(self) -> list:
        """Return sorted list of selected step indices."""
        indices = []
        for i in range(self._list_widget.count()):
            if self._list_widget.item(i).isSelected():
                indices.append(i)
        return sorted(indices)

    def set_current_row(self, index: int):
        """Select a specific row (after move up/down)."""
        self._list_widget.clearSelection()
        if 0 <= index < self._list_widget.count():
            self._list_widget.setCurrentRow(index)

    @property
    def editing_enabled(self) -> bool:
        return self._editing_enabled

    # --- Internal ---

    def _on_double_click(self, item: QListWidgetItem):
        if not self._editing_enabled:
            return
        row = self._list_widget.row(item)
        self.item_double_clicked.emit(row)

    def _on_selection_changed(self):
        self.selection_changed.emit(self.selected_indices())

    def _tick_elapsed(self):
        # Update the running row's timer
        if 0 <= self._current_step < len(self._step_rows):
            self._step_rows[self._current_step].update_elapsed()
        # Update toolbar elapsed
        self._exec_toolbar.update_elapsed()
