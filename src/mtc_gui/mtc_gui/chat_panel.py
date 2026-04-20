"""Chat panel: conversation display with message input for LLM interaction."""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit, QTextEdit,
)
from PyQt5.QtCore import pyqtSignal


class ChatPanel(QWidget):
    """Chat widget with conversation display and message input."""

    message_submitted = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # Conversation display
        self.display = QTextEdit()
        self.display.setReadOnly(True)
        layout.addWidget(self.display, stretch=1)

        # Thinking indicator
        self.thinking_label = QLabel("Thinking...")
        self.thinking_label.setStyleSheet("color: #888; font-style: italic; padding: 2px 4px;")
        self.thinking_label.hide()
        layout.addWidget(self.thinking_label)

        # Input bar
        input_row = QHBoxLayout()
        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText("Type a message...")
        self.input_field.returnPressed.connect(self._on_send)
        input_row.addWidget(self.input_field)

        send_btn = QPushButton("Send")
        send_btn.clicked.connect(self._on_send)
        input_row.addWidget(send_btn)

        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(self.clear_chat)
        input_row.addWidget(clear_btn)

        layout.addLayout(input_row)

    # -- Public slots / methods --

    def append_user(self, text: str) -> None:
        """Add a right-aligned user message bubble."""
        escaped = _esc(text)
        self.display.append(
            '<div style="text-align:right;margin:5px;">'
            '<span style="background-color:#2962ff;color:white;'
            'padding:8px 12px;border-radius:10px;display:inline-block;">'
            f'{escaped}</span></div>'
        )
        self._scroll_bottom()

    def append_assistant(self, text: str) -> None:
        """Add a left-aligned assistant message bubble."""
        escaped = _esc(text)
        self.display.append(
            '<div style="text-align:left;margin:5px;">'
            '<span style="background-color:#424242;color:#e0e0e0;'
            'padding:8px 12px;border-radius:10px;display:inline-block;">'
            f'{escaped}</span></div>'
        )
        self._scroll_bottom()

    def append_tool_call(self, name: str, args_json: str, result: str) -> None:
        """Add a compact tool-call info line."""
        args_short = (args_json[:80] + "...") if len(args_json) > 80 else args_json
        result_short = (result[:80] + "...") if len(result) > 80 else result
        self.display.append(
            f'<div style="margin:2px 20px;color:#888;font-size:11px;">'
            f'&#128295; {_esc(name)}({_esc(args_short)}) &rarr; {_esc(result_short)}'
            f'</div>'
        )
        self._scroll_bottom()

    def set_thinking(self, is_thinking: bool) -> None:
        """Show/hide thinking indicator and enable/disable input."""
        self.thinking_label.setVisible(is_thinking)
        self.input_field.setEnabled(not is_thinking)

    def clear_chat(self) -> None:
        """Reset the conversation display."""
        self.display.clear()

    # -- Private helpers --

    def _on_send(self) -> None:
        text = self.input_field.text().strip()
        if not text:
            return
        self.input_field.clear()
        self.append_user(text)
        self.message_submitted.emit(text)

    def _scroll_bottom(self) -> None:
        sb = self.display.verticalScrollBar()
        sb.setValue(sb.maximum())


def _esc(text: str) -> str:
    """Minimal HTML escape."""
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
