# EROBS — Development Reference

This file is Claude Code's persistent brief for **developing** the EROBS stack
(ROS 2 / MoveIt / Python orchestrator + PyQt5 GUI that controls a UR5e at the
NSLS-II CMS beamline). For **operating** the robot — task JSON, error
recovery, gripper conventions — use the `robot-operation` skill instead
(auto-loads on operator prompts, or invoke `/robot-operation`). Don't
duplicate ops content here.

## Routing: dev vs. ops

- **Dev** (edit Python/C++/launch/config, run tests, review PRs, update
  docs): you're in the right place. Read this file, use `docs/development.md`
  for architecture + setup. For action field details, read the `.action`
  files in `src/beambot_interfaces/action/` and the corresponding `_call_*`
  methods in `orchestrator.py` — those are authoritative.
- **Ops** (send `/beambot_execution` goals, author task JSON, diagnose a
  robot-side error): invoke the `robot-operation` skill. Its content lives
  at `src/beambot/beambot/agent/robot_operation.md` — don't load it by
  hand; the skill handles that.

If a user prompt mixes the two ("the pick_sample action server keeps
failing with PLANNING_FAILED — trace the orchestrator and also run the
move on the real robot"), do the code work under these dev rules and defer
the live-robot step to the ops skill.

## Repo layout (where things live)

| Path | What it is |
|---|---|
| `src/beambot/beambot/action_servers/` | One Python action server per task type (`move_to`, `end_effector`, `sample`, `tool_exchange`, `vision`, `pipettor`) plus the `orchestrator.py` that dispatches goals from `/beambot_execution` |
| `src/beambot/beambot/stages/` | MTC stage implementations (`base_stages.py` is shared; per-task stages next to it) |
| `src/beambot/beambot/core/` | `moveit_lifecycle_manager.py` (per-gripper MoveIt relaunches), `vacuum_monitor.py` (ePick watchdog) |
| `src/beambot/beambot/detection/` | Shared OpenCV detection (ArUco, Hough circles, contours, YOLO). **Single source of truth** — do not re-duplicate into `camera/zivid.py` or the MCP server |
| `src/beambot/beambot/camera/` | Camera drivers — `zivid.py` (active), `zed.py` (broken, don't trust) |
| `src/beambot/beambot/agent/` | **Experimental** direct Claude API + MCP loop. Reused by the GUI chat panel. Not production |
| `src/beambot/mcp/beambot_mcp_server.py` | FastMCP server exposing ops tools (vision, pose registry, robot state). Entry for the `beambot` MCP server in `.mcp.json` |
| `src/beambot/config/default_beamline.yaml` | Single config source: gripper list, MoveIt packages, tool voltages, dock numbers, vision targets, camera frames, `poses_file` path |
| `src/beambot/launch/beambot_bringup.launch.py` | Launches all action servers + Zivid + orchestrator. Takes `enable_vision`, `enable_pipettor`, `use_mock_hardware`, `enable_batching` |
| `src/beambot_interfaces/action/` | 9 `.action` definitions. When adding fields, update the corresponding `_create_*_goal` / `_call_*` method in `orchestrator.py` |
| `src/mtc_gui/` | PyQt5 operator cockpit (primary manual interface). `main_window.py` is the entry; task dialogs in `task_forms.py`; chat panel in `chat_panel.py` + `agent_bridge.py` (wires RobotAgent into Qt) |
| `src/custom-ur-descriptions/cms_moveit_config/` | URDF / SRDF / MoveIt configs. SRDF has a separate xacro per gripper (`hande.srdf.xacro`, `epick.srdf.xacro`, `2fg7.srdf.xacro`, `pipettor.srdf.xacro`, `none.srdf.xacro`) stitched by `ur.srdf.xacro` |
| `src/cms/` | CMS beamline assets: `poses.yaml` (pose registry — referenced by `poses_file` in `default_beamline.yaml`, auto-resolved by the orchestrator), `beamtime_poses.yaml`, `experiments.md` (session protocols), `tasks/` (JSON task sequences) |
| `src/bluesky_ros/` | Ophyd wrapper for `/beambot_execution`. **Currently broken / not kept in sync with PickSample/PlaceSample split.** Don't assume it works |
| `src/end_effectors/`, `src/vision/` | `vcs import`ed subtrees — gitignored. Edits here don't commit at this repo level |
| `.claude/skills/robot-operation/SKILL.md` | Skill that pulls `src/beambot/beambot/agent/robot_operation.md` into context when a robot-ops prompt matches. The source file is the single truth |
| `docs/development.md` | Architecture overview, calibration history, known issues |
| `docs/archive/` | Historical audits, legacy plans, and the retired `mcp_ros_reference.md` (content now lives in `robot_operation.md`) |
| `docs/robot_operation.md` | **Does not exist.** The ops reference lives at `src/beambot/beambot/agent/robot_operation.md` |

## Build, test, lint

```bash
# Source ROS 2 Jazzy first (robot machine: /opt/ros/jazzy)
source /opt/ros/jazzy/setup.bash

# Build (skip epick_moveit_studio — incompatible with Jazzy ros2_control)
colcon build --packages-skip epick_moveit_studio
source install/setup.bash

# Test (unit tests only — no hardware-in-the-loop tests in this repo)
./test.sh                  # colcon test --merge-install + colcon test-result --verbose
# Python tests: src/beambot/test/test_*.py (orchestrator parsing, batch planner, algorithms, base_stages)

# Lint (pre-commit runs ruff + ament linters; CI runs ruff.yml + super-linter.yml + ros.yaml on push/PR to jazzy_dev)
pre-commit run --all-files
ruff check --fix && ruff format
```

**Build gotcha**: `ament_python_install_package(beambot)` copies only `.py`
files from `src/beambot/beambot/` into `install/`. Non-`.py` data files
(e.g. `robot_operation.md`) are NOT copied unless explicitly installed. If
the agent loader breaks after a fresh build, that's why — resolve the
path from the repo root via `git rev-parse --show-toplevel` rather than
relying on `__file__` walk-up (matches the pattern `.mcp.json` already
uses).

## Invariants — don't silently change these

- **MoveIt relaunches on every `tool_exchange`** by design (see
  `core/moveit_lifecycle_manager.py`). Cycling `ur_ros2_control_node` is
  how per-tool voltage + Modbus activation works cleanly; pipettor has
  its own driver. Do not migrate to AttachURDF or unified-config patterns
  without a strong trigger — the reasoning and upstream alternatives are
  captured in the `moveit_tool_changing.md` auto-memory (verified
  2026-05, recheck Q3 2026).
- **Orchestrator owns MoveIt launch.** Don't add separate `move_group`
  launches in any launch file or bringup script.
- **Detection algorithms live only in `beambot/detection/`.** The old
  duplication across `camera/zivid.py` and the MCP server was resolved;
  any import of `detect_hough_circles`, `detect_contours_in_image`,
  `get_3d_position`, or the `*DetectionParams` dataclasses should come
  from `beambot.detection`, not be re-inlined.
- **Gripper config is driven by `default_beamline.yaml`.** Adding a
  gripper is a four-touch change: entry in that YAML, SRDF xacro in
  `cms_moveit_config/srdf/`, MoveIt config folder, and the `_GRIPPER_IK_FRAMES`
  dict in `orchestrator.py:909`. Forgetting any of these fails silently
  (wrong IK frame, missing SRDF state, no controller).
- **`src/beambot/beambot/agent/robot_operation.md` is the shared ops
  prompt** read by (a) the `robot-operation` skill, (b) `system_prompt.py`
  for the agent CLI, (c) the GUI chat panel (via RobotAgent). Renaming
  or moving it breaks all three. If you need to change its content,
  edit it directly — there's no generated-from-source mirror.
- **`batching_enabled = self._enable_batching and start_gripper != "epick"`**
  in `orchestrator.py` — hard-coded. ePick batching is disabled so the
  vacuum watchdog can run between every step. Do not refactor this to
  an "opt-out config" without explicit sign-off.

## Branch and CI state

- **Active branch: `jazzy_dev`.** Becoming the new main. `humble-dev` is the
  legacy archive for any deployment still on Humble. Other branches
  (`ai-dev`, `humble-experimental`, `refactor/codebase-cleanup`, `pr/*`) are
  historical — don't build new work on them.
- ROS 2 distro: **Jazzy** (Ubuntu 24.04). `start_mcp.sh` and `.mcp.json`
  both source `/opt/ros/jazzy/setup.bash`.
- CI: `ros.yaml` runs ament C++ tests + lint; `ruff.yml` runs `ruff check`
  and `ruff format`; `super-linter.yml` runs multi-language lint; all
  three trigger on push/PR to `main`, `humble`, `jazzy_dev`. Docker publish
  is `workflow_dispatch` only.

## When editing specific surfaces

**Editing the task JSON schema** (new field or task type): touch the
`.action` file in `beambot_interfaces`, the corresponding `_create_*_goal`
/ `_call_*` method in `orchestrator.py`, the stage handler, AND
`src/beambot/beambot/agent/robot_operation.md` §2/§3 so the ops agent
knows about it. Miss the last one and the agent keeps constructing
outdated JSON.

**Editing `beambot.agent`** (the direct Claude API loop): this is
experimental; don't assume it's stable. The GUI chat panel imports the
same `RobotAgent` class via `mtc_gui/agent_bridge.py`, so changes ripple
to both surfaces. The system prompt is loaded by `system_prompt.py` from
`robot_operation.md` — see the build-gotcha above about non-`.py` install
paths.

**Editing `.claude/skills/robot-operation/SKILL.md`**: the body is a
`!`cat ${CLAUDE_SKILL_DIR}/...`` injection of the ops doc. To change what
the skill delivers, edit the ops doc directly; change the SKILL.md only
for frontmatter tuning (description, trigger keywords).

**Don't touch** `src/end_effectors/{serial,robotiq_hande_*,ros2_epick_gripper,pipettor}`
or `src/vision/zivid-ros` — they're `vcs import`ed from
`src/{end_effectors,vision}/*.repos`. Fix upstream, then bump the ref.

## Further reading

- [`docs/development.md`](./docs/development.md) — full architecture, calibration history, known issues
- [`src/beambot/beambot/agent/robot_operation.md`](./src/beambot/beambot/agent/robot_operation.md) — robot-operation reference (consumed by the skill + agent CLI + GUI chat; don't read directly unless you're editing it)
- [`docs/archive/`](./docs/archive) — historical diagrams, PDFs, prior-design notes, retired audits
