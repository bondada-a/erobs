---
name: robot-operation
description: Operate the UR5e robot through the beambot orchestrator, author task JSON, and diagnose /beambot_execution failures. Use for robot motion, sample pick/place, tool exchange, pipettor operations, and vision capture or detection.
---

Before constructing robot goals or calling robot tools, read the complete
[shared robot-operation reference](../../../src/beambot/beambot/agent/robot_operation.md).
Resolve this path relative to this SKILL.md file.

The reference is the single source of truth shared with the Claude skill,
the beambot agent CLI, and the GUI chat panel. Follow its `<core_rules>`
and `<safety_boundary>` and consult its task schema and error taxonomy.

Use the connected `beambot` and `ros-mcp-server` tools. If the reference
or required tools are unavailable, report the limitation before attempting
robot operations.
