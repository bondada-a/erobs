---
name: robot-operation
description: Operate the UR5e robot at the CMS beamline — send motion, pick/place, tool exchange, pipettor, and vision goals through the beambot orchestrator. Use when the user asks the robot to move, pick or place a sample, swap a gripper, run the pipettor, capture or detect an image, or when diagnosing a /beambot_execution failure.
when_to_use: Trigger on prompts about moving the arm, picking/placing samples, tool exchange, pipettor operations (SUCK/EXPEL/EJECT_TIP), ArUco/vision capture, or interpreting an error_message from /beambot_execution. Also trigger on references to beambot, MTCExecution, task JSON, ePick vacuum, HandE gripper, 2fg7, scan poses, safe_tool_exchange, and tag IDs.
---

The content below is the full robot-operation reference — rules, task JSON schema,
MCP tool inventory, error taxonomy, and gotchas. It is the single source of truth
shared with the `beambot.agent` CLI and the GUI chat panel. Follow the rules in
`<core_rules>` and `<safety_boundary>` strictly.

!`cat ${CLAUDE_SKILL_DIR}/../../../src/beambot/beambot/agent/robot_operation.md`
