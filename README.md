# EROBS — Extensible Robotic Beamline Scientist

A platform for UR-arm–based autonomous sample handling at synchrotron beamlines.
Currently deployed at **NSLS-II CMS**; designed to generalize to other UR-based beamline setups.

The system pairs a UR5e with swappable end effectors, a Zivid 3D camera, and a ROS 2 /
MoveIt Task Constructor stack behind a JSON task interface, so experiments can be
authored manually, driven by an LLM over MCP, or — eventually — orchestrated by Bluesky.

## Hardware at a glance

- **Robot**: Universal Robots UR5e 6-DOF arm (fixed 20 % velocity / acceleration scaling)
- **Precision camera**: Zivid 2+ 3D, eye-in-hand, single-shot (ArUco + contour detection)
- **External camera**: ZED stereo — *present, not reliably wired up today*
- **End effectors** (swappable via `tool_exchange`): Robotiq Hand-E, Robotiq ePick (vacuum),
  OnRobot 2FG7, and a custom pipettor

## Software stack

- **ROS 2 Jazzy** on Ubuntu 24.04 (branch: `jazzy_dev`, migrating to main; `humble-dev`
  kept as legacy archive)
- **MoveIt 2** + **MoveIt Task Constructor** for planning
- **Python orchestrator** (`beambot` package): one action server per task type plus a
  central orchestrator that tracks gripper state, manages MoveIt lifecycle across tool
  swaps, and batches consecutive simple tasks
- **JSON task format** dispatched through `/beambot_execution` — see
  [`docs/mcp_ros_reference.md`](./docs/mcp_ros_reference.md) for the schema and
  [`CLAUDE.md`](./CLAUDE.md) for the operator-facing quick reference

## How to interact

Four entry points — honest labels:

1. **PyQt5 GUI** — *primary manual interface.* `ros2 run mtc_gui mtc_gui_client`.
   Per-task dialogs, camera overlays, pose editor, experiment runner. Also hosts an
   **experimental** chat panel backed by the `beambot.agent` module.
2. **MCP + Claude Code** — *LLM-assisted operation.* `./start_mcp.sh` launches
   `rosbridge` + `beambot_bringup`. Two MCP servers (generic `ros-mcp-server` and the
   custom `beambot-mcp-server`) expose task dispatch, vision, pose registry, and
   diagnostics to an LLM driving the robot. `.mcp.json` wires them up for Claude Code.
3. **`beambot.agent` CLI** — *experimental.* A direct Anthropic / Bedrock + MCP loop
   (`python -m beambot.agent`). Same system prompt as Claude Code, no Claude Code
   overhead. Parallel to path 2, not production-hardened yet.
4. **Bluesky RunEngine** — *planned / currently broken.* `src/bluesky_ros/` holds an
   Ophyd device wrapper for the orchestrator but it has not been kept in sync with the
   current action set; running it end-to-end will fail. Targeted for revival.

## Packages

| Path | What it contains |
|------|------------------|
| [`src/beambot`](./src/beambot) | Orchestrator, per-task action servers, MTC stages, MCP server, beambot.agent, detection algorithms, batch planner |
| [`src/beambot_interfaces`](./src/beambot_interfaces) | 9 ROS 2 action definitions (MTCExecution, MoveTo, EndEffector, PickSample, PlaceSample, ToolExchange, VisionMoveTo, VisionScan, Pipettor) |
| [`src/mtc_gui`](./src/mtc_gui) | PyQt5 operator GUI (see its [README](./src/mtc_gui/README.md)) |
| [`src/custom-ur-descriptions`](./src/custom-ur-descriptions) | UR5e URDF/xacro and MoveIt configs (one generic config that branches per gripper) |
| [`src/end_effectors`](./src/end_effectors) | Gripper drivers + `epick_config` overlay ([README](./src/end_effectors/README.md)) |
| [`src/vision`](./src/vision) | External vision repos (Zivid; ZED listed but not currently launched) |
| [`src/cms`](./src/cms) | CMS beamline assets — `poses.yaml`, `beamtime_poses.yaml`, `experiments.md`, task JSONs. CMS is the live beamline; some paths are hardcoded here today |
| [`src/lix`](./src/lix) | Placeholder for LIX beamline |
| [`src/demos`](./src/demos) | `hello_orchestrator_py` tutorial package |
| [`src/bluesky_ros`](./src/bluesky_ros) | Ophyd + Bluesky integration (see "How to interact" — currently broken) |

## Quick setup

Prerequisites: ROS 2 Jazzy, MoveIt 2, Zivid SDK (for vision), Python 3.12.

```bash
git clone https://github.com/bondada-a/erobs.git
cd erobs

# Import external dependencies
vcs import src           < src/ros2.repos
vcs import src/end_effectors < src/end_effectors/end_effectors.repos
vcs import src/vision    < src/vision/vision.repos

# Build (skip epick_moveit_studio — unused on Jazzy)
colcon build --packages-skip epick_moveit_studio
source install/setup.bash
```

See [`src/end_effectors/README.md`](./src/end_effectors/README.md) and
[`src/vision/README.md`](./src/vision/README.md) for SDK and driver requirements.

## Launch

```bash
# Real hardware, vision + pipettor enabled (defaults)
ros2 launch beambot beambot_bringup.launch.py

# Simulation (no real UR required)
ros2 launch beambot beambot_bringup.launch.py use_mock_hardware:=true

# Disable vision/pipettor if those subsystems aren't plugged in
ros2 launch beambot beambot_bringup.launch.py enable_vision:=false enable_pipettor:=false

# Primary operator GUI
ros2 run mtc_gui mtc_gui_client

# MCP stack (rosbridge on :9090 + beambot_bringup + rosbag recording)
./start_mcp.sh
```

`beambot_bringup.launch.py` starts all action servers and the Zivid camera
(conditionally); the orchestrator launches MoveIt lazily on the first goal based on the
attached gripper. See [`docs/development.md`](./docs/development.md) for the full
architecture, calibration history, and troubleshooting guide.

## Further reading

- [`CLAUDE.md`](./CLAUDE.md) — operator quick reference consumed by Claude Code (task
  JSON, error taxonomy, MCP gotchas)
- [`docs/development.md`](./docs/development.md) — architecture, build, calibration,
  known issues
- [`docs/mcp_ros_reference.md`](./docs/mcp_ros_reference.md) — per-task field reference
- [`src/cms/experiments.md`](./src/cms/experiments.md) — active experiment protocols
- [`docs/archive/`](./docs/archive) — historical diagrams, PDFs, and prior-design notes

## License

BSD-3-Clause (NSLS-II / Brookhaven National Laboratory). See [`LICENSE`](./LICENSE)
and [`LICENSE_README`](./LICENSE_README).
