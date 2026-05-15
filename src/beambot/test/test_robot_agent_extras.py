"""Tests for RobotAgent's extension hooks.

Covers system_prompt_prefix, extra_tools + extra_dispatch routing,
and tool_filter — added so the GUI bridge can inject local tools and
a mode banner without subclassing.

Avoids any real Anthropic client or MCP server: we stub the mcp imports
at sys.modules level before importing RobotAgent, then populate
self.tools / self.tool_to_session as connect() would and call _call_tool
directly.
"""

import asyncio
import os
import sys
import types
from unittest.mock import patch

import pytest

# Stub anthropic API key so _create_client construction doesn't blow up
# on the no-auth path (we patch _create_client itself anyway).
os.environ.setdefault("ANTHROPIC_API_KEY", "test-not-real")


def _install_mcp_stubs():
    """Insert minimal stubs for mcp.client.session and mcp.client.stdio.

    The real `mcp` SDK is only available where colcon test runs. Stubbing
    here lets these unit tests run in any environment.
    """
    if "mcp.client.session" in sys.modules:
        return
    mcp_pkg = types.ModuleType("mcp")
    client_pkg = types.ModuleType("mcp.client")
    session_mod = types.ModuleType("mcp.client.session")
    stdio_mod = types.ModuleType("mcp.client.stdio")

    class _ClientSession:
        async def initialize(self):
            return None

        async def list_tools(self):
            class _R:
                tools = []

            return _R()

        async def call_tool(self, name, args):
            class _R:
                content = []

            return _R()

    class _StdioServerParameters:
        def __init__(self, command=None, args=None, env=None):
            self.command, self.args, self.env = command, args, env

    async def _stdio_client(_params):
        yield (None, None)

    session_mod.ClientSession = _ClientSession
    stdio_mod.stdio_client = _stdio_client
    stdio_mod.StdioServerParameters = _StdioServerParameters

    sys.modules["mcp"] = mcp_pkg
    sys.modules["mcp.client"] = client_pkg
    sys.modules["mcp.client.session"] = session_mod
    sys.modules["mcp.client.stdio"] = stdio_mod


_install_mcp_stubs()


@pytest.fixture
def make_agent():
    """Factory that builds a RobotAgent with a stubbed Anthropic client."""
    from beambot.agent.robot_agent import RobotAgent

    def _factory(**kwargs):
        with patch(
            "beambot.agent.robot_agent._create_client",
            return_value=(object(), "test-model"),
        ):
            return RobotAgent(**kwargs)

    return _factory


def _run(coro):
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


def test_system_prompt_prefix_prepends_with_separator(make_agent):
    agent = make_agent(system_prompt_prefix="<gui_mode>HELLO</gui_mode>")
    assert agent.system_prompt.startswith("<gui_mode>HELLO</gui_mode>\n\n---\n\n")
    # Base prompt body is preserved after the separator
    assert len(agent.system_prompt) > len("<gui_mode>HELLO</gui_mode>\n\n---\n\n")


def test_no_prefix_leaves_prompt_unchanged(make_agent):
    bare = make_agent()
    prefixed_empty = make_agent(system_prompt_prefix="")
    assert bare.system_prompt == prefixed_empty.system_prompt


def test_extra_dispatch_routes_local_tools(make_agent):
    captured = {}

    async def dispatch(name, args):
        captured["name"] = name
        captured["args"] = args
        return f"local-result for {name}"

    agent = make_agent(
        extra_tools=[
            {
                "name": "propose_tasks",
                "description": "test",
                "input_schema": {"type": "object"},
            }
        ],
        extra_dispatch=dispatch,
    )
    # Simulate what connect() does for local tools, without spinning up MCP
    from beambot.agent.robot_agent import _LOCAL_SESSION_SENTINEL

    agent.tools.append(agent._extra_tools[0])
    agent.tool_to_session["propose_tasks"] = _LOCAL_SESSION_SENTINEL

    result = _run(
        agent._call_tool("propose_tasks", {"tasks": [{"task_type": "moveto"}]})
    )

    assert result == "local-result for propose_tasks"
    assert captured["name"] == "propose_tasks"
    assert captured["args"]["tasks"][0]["task_type"] == "moveto"


def test_local_tool_without_dispatcher_returns_error(make_agent):
    from beambot.agent.robot_agent import _LOCAL_SESSION_SENTINEL

    agent = make_agent(
        extra_tools=[
            {
                "name": "foo",
                "description": "",
                "input_schema": {"type": "object"},
            }
        ]
    )
    agent.tool_to_session["foo"] = _LOCAL_SESSION_SENTINEL

    result = _run(agent._call_tool("foo", {}))
    assert "no dispatcher" in result.lower()


def test_local_dispatch_exception_surfaces_as_error_string(make_agent):
    from beambot.agent.robot_agent import _LOCAL_SESSION_SENTINEL

    async def boom(name, args):
        raise RuntimeError("dispatch broke")

    agent = make_agent(
        extra_tools=[
            {
                "name": "bad",
                "description": "",
                "input_schema": {"type": "object"},
            }
        ],
        extra_dispatch=boom,
    )
    agent.tool_to_session["bad"] = _LOCAL_SESSION_SENTINEL

    result = _run(agent._call_tool("bad", {}))
    assert "dispatch broke" in result
    assert result.startswith("Error in local dispatch")


def test_tool_filter_predicate_stored(make_agent):
    """Confirm tool_filter is stored on the instance with the right arity."""

    def predicate(server_name, tool_name):
        return tool_name != "send_action_goal"

    agent = make_agent(tool_filter=predicate)
    assert agent._tool_filter is predicate
    assert predicate("ros-mcp-server", "send_action_goal") is False
    assert predicate("ros-mcp-server", "get_robot_state") is True


def test_unknown_tool_returns_existing_error_format(make_agent):
    agent = make_agent()
    result = _run(agent._call_tool("nonexistent_tool", {}))
    assert "not found in any connected MCP server" in result


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
