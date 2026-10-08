"""Tests for RobotAgent's extension hooks and CLI cleanup.

Covers system_prompt_prefix, extra_tools + extra_dispatch routing,
and tool_filter — added so the GUI bridge can inject local tools and
a mode banner without subclassing.

Avoids real clients and servers with scoped imports and fake transports.
"""

import asyncio
import importlib.util
import runpy
import sys
import types
from contextlib import asynccontextmanager
from pathlib import Path
from unittest.mock import AsyncMock, patch

import pytest

@pytest.fixture
def agent_module(monkeypatch):
    """Private production module; SDK import stubs never escape this import."""
    stubs = {
        name: types.ModuleType(name)
        for name in ("mcp", "mcp.client", "mcp.client.session", "mcp.client.stdio")
    }
    stubs["mcp.client.session"].ClientSession = object
    stubs["mcp.client.stdio"].stdio_client = object
    stubs["mcp.client.stdio"].StdioServerParameters = types.SimpleNamespace
    spec = importlib.util.spec_from_file_location(
        "_test_robot_agent", Path(__file__).parents[1] / "beambot" / "agent" / "robot_agent.py"
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    monkeypatch.setattr(module, "_create_client", lambda: (object(), "test-model"))
    return module


@pytest.fixture
def make_agent(agent_module):
    return agent_module.RobotAgent


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


def test_extra_dispatch_routes_local_tools(make_agent, agent_module):
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
    agent.tools.append(agent._extra_tools[0])
    agent.tool_to_session["propose_tasks"] = agent_module._LOCAL_SESSION_SENTINEL

    result = _run(
        agent._call_tool("propose_tasks", {"tasks": [{"task_type": "moveto"}]})
    )

    assert result == "local-result for propose_tasks"
    assert captured["name"] == "propose_tasks"
    assert captured["args"]["tasks"][0]["task_type"] == "moveto"


def test_local_tool_without_dispatcher_returns_error(make_agent, agent_module):
    agent = make_agent(
        extra_tools=[
            {
                "name": "foo",
                "description": "",
                "input_schema": {"type": "object"},
            }
        ]
    )
    agent.tool_to_session["foo"] = agent_module._LOCAL_SESSION_SENTINEL

    result = _run(agent._call_tool("foo", {}))
    assert "no dispatcher" in result.lower()


def test_local_dispatch_exception_surfaces_as_error_string(make_agent, agent_module):
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
    agent.tool_to_session["bad"] = agent_module._LOCAL_SESSION_SENTINEL

    result = _run(agent._call_tool("bad", {}))
    assert "dispatch broke" in result
    assert result.startswith("Error in local dispatch")


def test_tool_filter_excludes_disallowed_tools(make_agent, agent_module, monkeypatch):
    tools = [
        types.SimpleNamespace(name=name, description="", inputSchema={})
        for name in ("send_action_goal", "get_robot_state")
    ]
    session = types.SimpleNamespace(
        initialize=AsyncMock(),
        list_tools=AsyncMock(return_value=types.SimpleNamespace(tools=tools)),
    )

    @asynccontextmanager
    async def transport(_params):
        yield None, None

    @asynccontextmanager
    async def client_session(*_streams):
        yield session

    monkeypatch.setattr(agent_module, "stdio_client", transport)
    monkeypatch.setattr(agent_module, "ClientSession", client_session)
    agent = make_agent(tool_filter=lambda server, name: name != "send_action_goal")

    async def connect():
        async with agent._exit_stack:
            await agent._connect_server("ros-mcp-server", {"command": "unused"})

    _run(connect())
    assert [tool["name"] for tool in agent.tools] == ["get_robot_state"]
    assert agent.tool_to_session == {"get_robot_state": session}


def test_unknown_tool_returns_existing_error_format(make_agent):
    agent = make_agent()
    result = _run(agent._call_tool("nonexistent_tool", {}))
    assert "not found in any connected MCP server" in result


@pytest.mark.parametrize("error", [RuntimeError("Connection failed"), asyncio.CancelledError()])
def test_cli_disconnects_after_failed_startup(error):
    agent = types.SimpleNamespace(
        connect=AsyncMock(side_effect=error),
        disconnect=AsyncMock(),
    )
    stub = types.ModuleType("beambot.agent.robot_agent")
    stub.RobotAgent = lambda: agent

    with patch.dict(sys.modules, {"beambot.agent.robot_agent": stub}):
        with pytest.raises(type(error)) as raised:
            runpy.run_module("beambot.agent", run_name="__main__")

    assert raised.value is error
    agent.connect.assert_awaited_once_with()
    agent.disconnect.assert_awaited_once_with()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
