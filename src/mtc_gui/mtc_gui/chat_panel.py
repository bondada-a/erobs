"""Chat panel: modern conversation UI with message bubbles for LLM interaction."""

import time

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QScrollArea,
    QTextEdit,
    QSizePolicy,
    QFrame,
    QButtonGroup,
    QRadioButton,
)
from PyQt6.QtCore import pyqtSignal, Qt, QTimer, QEvent


# ---------------------------------------------------------------------------
# Style constants
# ---------------------------------------------------------------------------

_COLORS = {
    "user_bg": "#1a6b3a",
    "user_text": "#e8f5e9",
    "assistant_bg": "#2d2d2d",
    "assistant_text": "#e0e0e0",
    "tool_bg": "#1e1e2e",
    "tool_border": "#3d3d5c",
    "tool_text": "#a0a0c0",
    "error_bg": "#3d1a1a",
    "error_border": "#6b2b2b",
    "error_text": "#f0a0a0",
    "timestamp": "#777777",
    "input_bg": "#2a2a2a",
    "input_border": "#444444",
    "input_border_focus": "#5c9ce6",
    "send_btn": "#5c9ce6",
    "send_btn_hover": "#7ab3f0",
    "clear_btn": "#555555",
    "clear_btn_hover": "#666666",
    "thinking_dot": "#5c9ce6",
    "panel_bg": "#1e1e1e",
    "scroll_bg": "#252525",
}

_PANEL_STYLESHEET = f"""
    QWidget#chatPanelRoot {{
        background-color: {_COLORS["panel_bg"]};
    }}
    QScrollArea {{
        background-color: {_COLORS["scroll_bg"]};
        border: none;
    }}
    QScrollArea > QWidget > QWidget {{
        background-color: {_COLORS["scroll_bg"]};
    }}
    QScrollBar:vertical {{
        background: {_COLORS["panel_bg"]};
        width: 8px;
        margin: 0;
    }}
    QScrollBar::handle:vertical {{
        background: #555555;
        min-height: 30px;
        border-radius: 4px;
    }}
    QScrollBar::handle:vertical:hover {{
        background: #777777;
    }}
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
        height: 0;
    }}
"""


# ---------------------------------------------------------------------------
# Message bubble widgets
# ---------------------------------------------------------------------------


class MessageBubble(QFrame):
    """Single chat message rendered as a styled bubble."""

    def __init__(self, text: str, role: str, timestamp: str = "", parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.NoFrame)

        is_user = role == "user"
        bg = _COLORS["user_bg"] if is_user else _COLORS["assistant_bg"]
        fg = _COLORS["user_text"] if is_user else _COLORS["assistant_text"]
        margin_side = "margin-left: 48px;" if is_user else "margin-right: 48px;"

        self.setStyleSheet(f"""
            MessageBubble {{
                background: transparent;
                border: none;
                {margin_side}
            }}
        """)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 2, 8, 2)
        outer.setSpacing(2)

        # Role label
        role_label = QLabel("You" if is_user else "Assistant")
        role_label.setStyleSheet(f"""
            color: {fg};
            font-size: 11px;
            font-weight: bold;
            padding: 0 4px;
        """)
        role_label.setAlignment(
            Qt.AlignmentFlag.AlignLeft if not is_user else Qt.AlignmentFlag.AlignRight
        )
        outer.addWidget(role_label)

        # Bubble container
        bubble = QFrame()
        bubble.setStyleSheet(f"""
            QFrame {{
                background-color: {bg};
                border-radius: 12px;
                padding: 10px 14px;
            }}
        """)
        bubble_layout = QVBoxLayout(bubble)
        bubble_layout.setContentsMargins(0, 0, 0, 0)
        bubble_layout.setSpacing(4)

        # Message text
        msg_label = QLabel(text)
        msg_label.setWordWrap(True)
        msg_label.setTextFormat(Qt.TextFormat.PlainText)
        msg_label.setStyleSheet(f"""
            color: {fg};
            font-size: 13px;
            line-height: 1.4;
            background: transparent;
            padding: 0;
        """)
        msg_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum
        )
        msg_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        bubble_layout.addWidget(msg_label)

        outer.addWidget(bubble)

        # Timestamp
        if timestamp:
            ts_label = QLabel(timestamp)
            ts_label.setStyleSheet(f"""
                color: {_COLORS["timestamp"]};
                font-size: 10px;
                padding: 0 4px;
            """)
            ts_label.setAlignment(
                Qt.AlignmentFlag.AlignLeft
                if not is_user
                else Qt.AlignmentFlag.AlignRight
            )
            outer.addWidget(ts_label)


class ToolCallBubble(QFrame):
    """Compact expandable tool call indicator."""

    def __init__(self, name: str, args_json: str, result: str, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.NoFrame)
        self._expanded = False
        self._args = args_json
        self._result = result

        self.setStyleSheet("""
            ToolCallBubble {
                background: transparent;
                margin-right: 48px;
            }
        """)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 1, 8, 1)
        outer.setSpacing(0)

        # Collapsed header (clickable)
        self._header = QPushButton(f"  {name}()")
        self._header.setFlat(True)
        self._header.setCursor(Qt.CursorShape.PointingHandCursor)
        self._header.setStyleSheet(f"""
            QPushButton {{
                color: {_COLORS["tool_text"]};
                font-size: 11px;
                font-family: monospace;
                text-align: left;
                padding: 4px 10px;
                background-color: {_COLORS["tool_bg"]};
                border: 1px solid {_COLORS["tool_border"]};
                border-radius: 6px;
            }}
            QPushButton:hover {{
                background-color: #252540;
                border-color: #5555aa;
            }}
        """)
        self._header.clicked.connect(self._toggle)
        outer.addWidget(self._header)

        # Expandable detail
        self._detail = QLabel()
        self._detail.setWordWrap(True)
        self._detail.setTextFormat(Qt.TextFormat.PlainText)
        self._detail.setStyleSheet(f"""
            color: {_COLORS["tool_text"]};
            font-size: 11px;
            font-family: monospace;
            padding: 6px 12px;
            background-color: {_COLORS["tool_bg"]};
            border: 1px solid {_COLORS["tool_border"]};
            border-top: none;
            border-radius: 0 0 6px 6px;
        """)
        self._detail.hide()
        outer.addWidget(self._detail)

    def _toggle(self):
        self._expanded = not self._expanded
        if self._expanded:
            detail_text = f"Args: {self._args}\nResult: {self._result}"
            self._detail.setText(detail_text)
            self._detail.show()
            self._header.setStyleSheet(self._header.styleSheet())
        else:
            self._detail.hide()


class ErrorBubble(QFrame):
    """Red-tinted error message bubble."""

    def __init__(self, text: str, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.NoFrame)
        self.setStyleSheet(
            "ErrorBubble { background: transparent; margin-right: 48px; }"
        )

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 2, 8, 2)
        outer.setSpacing(2)

        role_label = QLabel("Error")
        role_label.setStyleSheet(f"""
            color: {_COLORS["error_text"]};
            font-size: 11px;
            font-weight: bold;
            padding: 0 4px;
        """)
        outer.addWidget(role_label)

        bubble = QFrame()
        bubble.setStyleSheet(f"""
            QFrame {{
                background-color: {_COLORS["error_bg"]};
                border: 1px solid {_COLORS["error_border"]};
                border-radius: 12px;
                padding: 10px 14px;
            }}
        """)
        bubble_layout = QVBoxLayout(bubble)
        bubble_layout.setContentsMargins(0, 0, 0, 0)

        msg_label = QLabel(text)
        msg_label.setWordWrap(True)
        msg_label.setTextFormat(Qt.TextFormat.PlainText)
        msg_label.setStyleSheet(f"""
            color: {_COLORS["error_text"]};
            font-size: 13px;
            background: transparent;
            padding: 0;
        """)
        msg_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum
        )
        msg_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        bubble_layout.addWidget(msg_label)

        outer.addWidget(bubble)


class ThinkingIndicator(QFrame):
    """Animated thinking dots indicator."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.NoFrame)
        self.setStyleSheet("background: transparent; margin-right: 48px;")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 4, 16, 4)
        layout.setAlignment(Qt.AlignmentFlag.AlignLeft)

        self._dots = []
        for i in range(3):
            dot = QLabel("•")
            dot.setStyleSheet(f"""
                color: {_COLORS["thinking_dot"]};
                font-size: 20px;
                padding: 0 2px;
            """)
            layout.addWidget(dot)
            self._dots.append(dot)

        layout.addStretch()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._animate)
        self._tick = 0

    def start(self):
        self._tick = 0
        self._timer.start(400)
        self.show()

    def stop(self):
        self._timer.stop()
        self.hide()

    def _animate(self):
        for i, dot in enumerate(self._dots):
            if i == self._tick % 3:
                dot.setStyleSheet(f"""
                    color: {_COLORS["thinking_dot"]};
                    font-size: 24px;
                    padding: 0 2px;
                """)
            else:
                dot.setStyleSheet("""
                    color: #3a5a7a;
                    font-size: 20px;
                    padding: 0 2px;
                """)
        self._tick += 1


# ---------------------------------------------------------------------------
# Main Chat Panel
# ---------------------------------------------------------------------------


class ChatPanel(QWidget):
    """Modern chat widget with scrollable message list and input area."""

    message_submitted = pyqtSignal(str)
    mode_change_requested = pyqtSignal(str)  # "plan" or "run"

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("chatPanelRoot")
        self.setStyleSheet(_PANEL_STYLESHEET)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # --- Header ---
        header = QFrame()
        header.setStyleSheet(f"""
            QFrame {{
                background-color: {_COLORS["panel_bg"]};
                border-bottom: 1px solid #333333;
                padding: 8px 12px;
            }}
        """)
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(12, 8, 12, 8)
        title = QLabel("Beambot Assistant")
        title.setStyleSheet("color: #e0e0e0; font-size: 14px; font-weight: bold;")
        header_layout.addWidget(title)

        # Mode toggle (Plan | Run). Plan is the default safe state.
        self._mode_group = QButtonGroup(self)
        self._mode_group.setExclusive(True)
        self._plan_radio = QRadioButton("Plan")
        self._run_radio = QRadioButton("Run")
        self._plan_radio.setChecked(True)
        radio_style = (
            "QRadioButton { color: #cccccc; font-size: 12px; padding: 0 8px; }"
        )
        self._plan_radio.setStyleSheet(radio_style)
        self._run_radio.setStyleSheet(radio_style)
        self._plan_radio.setToolTip(
            "Agent proposes; human reviews and presses Execute."
        )
        self._run_radio.setToolTip("Agent proposes and dispatches via the GUI.")
        self._mode_group.addButton(self._plan_radio)
        self._mode_group.addButton(self._run_radio)
        header_layout.addWidget(self._plan_radio)
        header_layout.addWidget(self._run_radio)
        # toggled fires twice on a switch (old=False, new=True). Filter to "True".
        self._plan_radio.toggled.connect(self._on_mode_radio_toggled)
        self._run_radio.toggled.connect(self._on_mode_radio_toggled)

        header_layout.addStretch()

        self._status_label = QLabel("Connecting...")
        self._status_label.setStyleSheet("color: #888888; font-size: 11px;")
        header_layout.addWidget(self._status_label)
        layout.addWidget(header)

        # --- Scrollable message area ---
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)

        self._messages_container = QWidget()
        self._messages_layout = QVBoxLayout(self._messages_container)
        self._messages_layout.setContentsMargins(4, 8, 4, 8)
        self._messages_layout.setSpacing(4)
        self._messages_layout.addStretch()

        self._scroll.setWidget(self._messages_container)
        layout.addWidget(self._scroll, stretch=1)

        # --- Thinking indicator ---
        self._thinking = ThinkingIndicator()
        self._thinking.hide()
        layout.addWidget(self._thinking)

        # --- Input area ---
        input_frame = QFrame()
        input_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {_COLORS["panel_bg"]};
                border-top: 1px solid #333333;
            }}
        """)
        input_outer = QVBoxLayout(input_frame)
        input_outer.setContentsMargins(12, 8, 12, 8)
        input_outer.setSpacing(6)

        # Multi-line input
        self.input_field = QTextEdit()
        self.input_field.setPlaceholderText("Ask Beambot something...")
        self.input_field.setMaximumHeight(80)
        self.input_field.setMinimumHeight(36)
        self.input_field.setStyleSheet(f"""
            QTextEdit {{
                background-color: {_COLORS["input_bg"]};
                color: #e0e0e0;
                border: 1px solid {_COLORS["input_border"]};
                border-radius: 8px;
                padding: 8px 12px;
                font-size: 13px;
            }}
            QTextEdit:focus {{
                border-color: {_COLORS["input_border_focus"]};
            }}
        """)
        self.input_field.installEventFilter(self)
        input_outer.addWidget(self.input_field)

        # Button row
        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)

        hint_label = QLabel("Enter to send, Shift+Enter for newline")
        hint_label.setStyleSheet("color: #666666; font-size: 10px;")
        btn_row.addWidget(hint_label)
        btn_row.addStretch()

        clear_btn = QPushButton("Clear")
        clear_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        clear_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {_COLORS["clear_btn"]};
                color: #cccccc;
                border: none;
                border-radius: 6px;
                padding: 5px 14px;
                font-size: 12px;
            }}
            QPushButton:hover {{
                background-color: {_COLORS["clear_btn_hover"]};
            }}
        """)
        clear_btn.clicked.connect(self.clear_chat)
        btn_row.addWidget(clear_btn)

        send_btn = QPushButton("Send")
        send_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        send_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {_COLORS["send_btn"]};
                color: white;
                border: none;
                border-radius: 6px;
                padding: 5px 18px;
                font-size: 12px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {_COLORS["send_btn_hover"]};
            }}
            QPushButton:disabled {{
                background-color: #3a3a3a;
                color: #666666;
            }}
        """)
        send_btn.clicked.connect(self._on_send)
        self._send_btn = send_btn
        btn_row.addWidget(send_btn)

        input_outer.addLayout(btn_row)
        layout.addWidget(input_frame)

    # -- Event filter for Enter key handling --

    def eventFilter(self, obj, event):
        if obj is self.input_field and event.type() == QEvent.Type.KeyPress:
            if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
                if event.modifiers() & Qt.KeyboardModifier.ShiftModifier:
                    return False  # allow newline
                self._on_send()
                return True
        return super().eventFilter(obj, event)

    # -- Public slots / methods --

    def set_status(self, text: str) -> None:
        self._status_label.setText(text)

    def append_user(self, text: str) -> None:
        ts = time.strftime("%H:%M")
        bubble = MessageBubble(text, "user", ts)
        self._add_message_widget(bubble)

    def append_assistant(self, text: str) -> None:
        ts = time.strftime("%H:%M")
        bubble = MessageBubble(text, "assistant", ts)
        self._add_message_widget(bubble)

    def append_tool_call(self, name: str, args_json: str, result: str) -> None:
        bubble = ToolCallBubble(name, args_json, result)
        self._add_message_widget(bubble)

    def append_error(self, text: str) -> None:
        bubble = ErrorBubble(text)
        self._add_message_widget(bubble)

    def append_execution_outcome(
        self, success: bool, error: str, completed: int, total: int
    ) -> None:
        """Render a colored summary bubble for an agent-initiated run."""
        if success:
            text = f"Execution succeeded — {completed}/{total} step(s) completed."
            bubble = MessageBubble(text, "assistant", time.strftime("%H:%M"))
        else:
            tail = f" — {error}" if error else ""
            text = f"Execution failed — {completed}/{total} step(s) completed{tail}."
            bubble = ErrorBubble(text)
        self._add_message_widget(bubble)

    def set_thinking(self, is_thinking: bool) -> None:
        if is_thinking:
            self._thinking.start()
            self._send_btn.setEnabled(False)
            self.input_field.setEnabled(False)
        else:
            self._thinking.stop()
            self._send_btn.setEnabled(True)
            self.input_field.setEnabled(True)
            self.input_field.setFocus()

    def clear_chat(self) -> None:
        while self._messages_layout.count() > 1:
            item = self._messages_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

    def set_mode_label(self, mode: str) -> None:
        """Re-enable mode radios and refresh the status label suffix."""
        self._plan_radio.setEnabled(True)
        self._run_radio.setEnabled(True)
        # Sync the radios in case the mode change came from elsewhere
        if mode == "plan" and not self._plan_radio.isChecked():
            self._plan_radio.blockSignals(True)
            self._plan_radio.setChecked(True)
            self._plan_radio.blockSignals(False)
        elif mode == "run" and not self._run_radio.isChecked():
            self._run_radio.blockSignals(True)
            self._run_radio.setChecked(True)
            self._run_radio.blockSignals(False)

    # -- Private helpers --

    def _on_mode_radio_toggled(self, checked: bool) -> None:
        # toggled fires for both the deselected and selected radio.
        # Only react to the radio that just became selected.
        if not checked:
            return
        mode = "plan" if self._plan_radio.isChecked() else "run"
        # Disable both radios until the bridge confirms via mode_changed.
        # Prevents a rapid double-toggle while the agent is being rebuilt.
        self._plan_radio.setEnabled(False)
        self._run_radio.setEnabled(False)
        self.mode_change_requested.emit(mode)

    def _on_send(self) -> None:
        text = self.input_field.toPlainText().strip()
        if not text:
            return
        self.input_field.clear()
        self.message_submitted.emit(text)

    def _add_message_widget(self, widget: QWidget) -> None:
        insert_idx = self._messages_layout.count() - 1
        self._messages_layout.insertWidget(insert_idx, widget)
        QTimer.singleShot(10, self._scroll_bottom)

    def _scroll_bottom(self) -> None:
        sb = self._scroll.verticalScrollBar()
        sb.setValue(sb.maximum())
