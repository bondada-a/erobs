"""Load the robot control system prompt from CLAUDE.md."""

import os

_PREAMBLE = """You control a UR5e robot at the CMS beamline via MCP tool calls.

Rules:
- Call get_robot_state first to check system status and current gripper.
- Send tasks through send_action_goal to /beambot_execution action server.
- Always read error_message from results before deciding next action.
- start_gripper must match the physically attached gripper.
- Joint poses are in DEGREES in the task JSON.
- Direction vectors (forward, backward, left, right, up, down) are in the flange frame — use them literally.
- After ePick vacuum pick, call get_vacuum_status to verify the object was grasped.

Below is the full task reference:
"""


def load_system_prompt() -> str:
    """Load CLAUDE.md and prepend the robot control preamble."""
    # Walk up from this file to find project root CLAUDE.md
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        candidate = os.path.join(here, "CLAUDE.md")
        if os.path.exists(candidate):
            with open(candidate) as f:
                return _PREAMBLE + f.read()
        here = os.path.dirname(here)

    # Fallback: try cwd
    if os.path.exists("CLAUDE.md"):
        with open("CLAUDE.md") as f:
            return _PREAMBLE + f.read()

    return _PREAMBLE + "(CLAUDE.md not found — operating without task reference)"
