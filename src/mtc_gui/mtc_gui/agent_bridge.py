"""Async RobotAgent <-> PyQt6 bridge via background asyncio event loop."""

import asyncio
import json
import threading

from PyQt6.QtCore import QObject, pyqtSignal


class AgentBridge(QObject):
    """Thread-safe bridge between the async RobotAgent and Qt signals.

    Runs a dedicated asyncio event loop in a daemon thread so that
    agent.connect() and agent.chat() never block the Qt main thread.
    """

    response_received = pyqtSignal(str)       # final text from Claude
    tool_called = pyqtSignal(str, str, str)   # (tool_name, args_json, result_text)
    error_occurred = pyqtSignal(str)          # error message
    thinking_changed = pyqtSignal(bool)       # True=waiting for API, False=done
    connected = pyqtSignal(int)               # number of tools loaded

    def __init__(self, parent=None):
        super().__init__(parent)
        self._loop = None
        self._thread = None
        self._agent = None

    # --- Public API (called from Qt main thread) ---

    def connect_agent(self):
        """Spawn background event loop thread and connect the agent."""
        def _run_loop():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            self._loop.run_until_complete(self._async_connect())
            self._loop.run_forever()

        self._thread = threading.Thread(target=_run_loop, daemon=True)
        self._thread.start()

    def send_message(self, text: str):
        """Submit a chat request to the background event loop."""
        if not self._loop or not self._agent:
            self.error_occurred.emit("Agent not connected")
            return
        asyncio.run_coroutine_threadsafe(self._async_chat(text), self._loop)

    def clear_history(self):
        """Reset conversation history (sync, safe from any thread)."""
        if self._agent:
            self._agent.clear_history()

    def disconnect(self):
        """Stop the background event loop and thread."""
        if self._loop and self._loop.is_running():
            # Schedule async cleanup then stop the loop
            asyncio.run_coroutine_threadsafe(self._async_disconnect(), self._loop)
        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None

    # --- Async internals (run on background event loop) ---

    async def _async_connect(self):
        try:
            from beambot.agent.robot_agent import RobotAgent
            self._agent = RobotAgent()
            await self._agent.connect()
            self.connected.emit(len(self._agent.tools))
        except Exception as e:
            self.error_occurred.emit(f"Connection failed: {e}")

    async def _async_chat(self, text: str):
        self.thinking_changed.emit(True)
        try:
            def on_tool(name, args, result):
                self.tool_called.emit(
                    name, json.dumps(args)[:200], (result or "")[:500]
                )

            response = await self._agent.chat(text, on_tool_call=on_tool)
            self.response_received.emit(response)
        except Exception as e:
            self.error_occurred.emit(str(e))
        finally:
            self.thinking_changed.emit(False)

    async def _async_disconnect(self):
        try:
            if self._agent:
                await self._agent.disconnect()
                self._agent = None
        finally:
            self._loop.stop()
