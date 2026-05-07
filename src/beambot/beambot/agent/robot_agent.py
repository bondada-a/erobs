"""Lightweight agentic loop: Claude API + MCP client for robot control."""

import asyncio
import json
import logging
import os
from contextlib import AsyncExitStack

from mcp.client.session import ClientSession
from mcp.client.stdio import stdio_client, StdioServerParameters

logger = logging.getLogger(__name__)

# Path to the shared robot-operation prompt. Colocated with this module so the
# same file drives the CLI, the GUI chat panel (via RobotAgent), and the
# .claude/skills/robot-operation skill (which cats it into Claude Code's context).
# Read fresh on every RobotAgent() construction so prompt edits take effect on
# the next chat without restarting the host process.
_PROMPT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_operation.md")


def _load_system_prompt() -> str:
    with open(_PROMPT_PATH) as f:
        return f.read()


def _create_client():
    """Create the right Anthropic client based on environment.

    Bedrock: uses default AWS credential provider chain (~/.aws/credentials
    or AWS_ACCESS_KEY_ID/AWS_SECRET_ACCESS_KEY env vars). Region from
    AWS_REGION env var or defaults to us-east-1.

    Direct API: uses ANTHROPIC_API_KEY env var.
    """
    if os.environ.get("CLAUDE_CODE_USE_BEDROCK") == "1":
        from anthropic import AnthropicBedrock
        region = os.environ.get("AWS_REGION", os.environ.get("AWS_DEFAULT_REGION", "us-east-1"))
        client = AnthropicBedrock(aws_region=region)
        # Use regional model ID (us. prefix). For global routing use "global." prefix instead.
        default_model = "us.anthropic.claude-opus-4-6-v1"
        return client, default_model
    else:
        from anthropic import Anthropic
        return Anthropic(), "claude-sonnet-4-5-20250514"


class RobotAgent:
    """Direct Claude API → MCP tool loop. No Claude Code overhead."""

    def __init__(self, model=None, mcp_config_path=None):
        self.client, default_model = _create_client()
        self.model = model or default_model
        self.system_prompt = _load_system_prompt()
        self.tools = []              # Anthropic API tool format
        self.tool_to_session = {}    # tool_name -> ClientSession
        self.messages = []           # conversation history
        self._exit_stack = AsyncExitStack()
        self._mcp_config_path = mcp_config_path

    async def connect(self):
        """Connect to all MCP servers defined in .mcp.json."""
        config_path = self._mcp_config_path or self._find_mcp_config()
        if not config_path:
            raise FileNotFoundError("No .mcp.json found")

        with open(config_path) as f:
            config = json.load(f)

        await self._exit_stack.__aenter__()

        for name, server_cfg in config.get("mcpServers", {}).items():
            try:
                await self._connect_server(name, server_cfg)
            except Exception as e:
                logger.warning(f"Failed to connect to MCP server '{name}': {e}")

        logger.info(
            f"Connected: {len(self.tools)} tools from "
            f"{len(set(self.tool_to_session.values()))} server(s)"
        )

    async def _connect_server(self, name, server_cfg):
        """Connect to a single MCP server and register its tools."""
        params = StdioServerParameters(
            command=server_cfg["command"],
            args=server_cfg.get("args", []),
            env=server_cfg.get("env"),
        )

        # stdio_client is an async context manager — keep it alive via exit stack
        streams = await self._exit_stack.enter_async_context(stdio_client(params))
        read_stream, write_stream = streams

        session = await self._exit_stack.enter_async_context(ClientSession(read_stream, write_stream))
        await session.initialize()

        # Register tools
        result = await session.list_tools()
        for tool in result.tools:
            self.tools.append({
                "name": tool.name,
                "description": tool.description or "",
                "input_schema": tool.inputSchema,
            })
            self.tool_to_session[tool.name] = session

        logger.info(f"  {name}: {len(result.tools)} tools")

    async def chat(self, user_message: str, on_tool_call=None, on_text=None) -> str:
        """Send message, execute tool calls in a loop, return final text.

        Args:
            user_message: The user's request.
            on_tool_call: Optional callback(name, input, result) for each tool call.
            on_text: Optional callback(text) for streaming text chunks.
        """
        self.messages.append({"role": "user", "content": user_message})

        while True:
            response = self.client.messages.create(
                model=self.model,
                max_tokens=4096,
                system=self.system_prompt,
                tools=self.tools,
                messages=self.messages,
            )

            # Serialize response content for message history
            content = []
            for block in response.content:
                if block.type == "text":
                    content.append({"type": "text", "text": block.text})
                elif block.type == "tool_use":
                    content.append({
                        "type": "tool_use",
                        "id": block.id,
                        "name": block.name,
                        "input": block.input,
                    })
            self.messages.append({"role": "assistant", "content": content})

            # If no tool calls, return final text
            if response.stop_reason == "end_turn":
                text = "".join(b.text for b in response.content if b.type == "text")
                if on_text:
                    on_text(text)
                return text

            # Execute tool calls
            tool_results = []
            for block in response.content:
                if block.type == "tool_use":
                    result_text = await self._call_tool(block.name, block.input)
                    if on_tool_call:
                        on_tool_call(block.name, block.input, result_text)
                    tool_results.append({
                        "type": "tool_result",
                        "tool_use_id": block.id,
                        "content": result_text,
                    })

            if tool_results:
                self.messages.append({"role": "user", "content": tool_results})

    async def _call_tool(self, name: str, arguments: dict) -> str:
        """Route tool call to the correct MCP server session."""
        session = self.tool_to_session.get(name)
        if not session:
            return f"Error: tool '{name}' not found in any connected MCP server"

        try:
            result = await session.call_tool(name, arguments)
            parts = []
            for item in result.content:
                if hasattr(item, "text"):
                    parts.append(item.text)
            return "\n".join(parts) if parts else str(result)
        except Exception as e:
            return f"Error calling tool '{name}': {e}"

    def clear_history(self):
        """Reset conversation history."""
        self.messages = []

    async def disconnect(self):
        """Shut down all MCP server connections."""
        await self._exit_stack.aclose()

    def _find_mcp_config(self):
        """Walk up from cwd to find .mcp.json."""
        path = os.getcwd()
        for _ in range(10):
            candidate = os.path.join(path, ".mcp.json")
            if os.path.exists(candidate):
                return candidate
            parent = os.path.dirname(path)
            if parent == path:
                break
            path = parent
        return None
