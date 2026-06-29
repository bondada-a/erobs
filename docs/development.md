# EROBS Development Reference

> **Operating the robot** (task JSON, error taxonomy, MCP gotchas) is in
> [`src/beambot/beambot/agent/robot_operation.md`](../src/beambot/beambot/agent/robot_operation.md),
> loaded on demand via the `robot-operation` skill. Don't duplicate that
> content here — this doc is for *building* the stack, not driving it.
>
> For the quick orientation a new Claude Code session needs, see
> [`CLAUDE.md`](../CLAUDE.md). This file is the deeper developer reference
> behind the same content: architecture, build, calibration history, known
> issues.

## Overview

Robotic sample-handling stack for NSLS-II beamlines. A ROS 2 / MoveIt
Task Constructor pipeline drives a UR5e with swappable end effectors,
controlled through a JSON task interface that can be driven by the GUI,
by an LLM over MCP, or (eventually) by Bluesky.

Currently deployed at **CMS**. The beamline config layer (`cms_beamline.yaml`,
selected at runtime via `$BEAMBOT_BEAMLINE_CONFIG`)
and the per-gripper MoveIt configs are designed for reuse across UR-arm
beamlines, but no second site is live today.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  ENTRY POINTS (four interfaces, all speak MTCExecution JSON) │
├─────────────────────────────────────────────────────────────┤
│  mtc_gui (PyQt5)      Claude Code + MCP    beambot.agent    │
│  — primary manual     — LLM-assisted       — experimental    │
│                                                              │
│  Bluesky / Ophyd — planned, currently broken                 │
└────────────────────────────┬────────────────────────────────┘
                             │ full_json string
                             ▼
┌─────────────────────────────────────────────────────────────┐
│  ORCHESTRATOR   /beambot_execution  (action server)          │
│  — task parsing, batching, pause/resume                      │
│  — MoveIt lifecycle (relaunch per gripper)                   │
│  — vacuum watchdog (ePick)                                   │
└────────────────────────────┬────────────────────────────────┘
                             │ per-task goals
                             ▼
┌─────────────────────────────────────────────────────────────┐
│  ACTION SERVERS  (8, one per task type)                      │
│  move_to, end_effector, tool_exchange, pipettor              │
│  vision_moveto, vision_scan, pick_sample, place_sample       │
└────────────────────────────┬────────────────────────────────┘
                             │ MTC stages
                             ▼
┌─────────────────────────────────────────────────────────────┐
│  MoveIt Task Constructor + MoveIt 2                          │
│   planners: Pilz LIN / PTP, OMPL, CartesianPath              │
└────────────────────────────┬────────────────────────────────┘
                             │ trajectories
                             ▼
┌─────────────────────────────────────────────────────────────┐
│  HARDWARE                                                    │
│  UR5e · Zivid 2+ (active) · ZED (present, broken)            │
│  Grippers: hande / epick / 2fg7 / pipettor / none            │
└─────────────────────────────────────────────────────────────┘
```

**Task types**: 8 in `src/beambot_interfaces/action/` — `MoveTo`,
`EndEffector`, `ToolExchange`, `VisionMoveTo`, `VisionScan`,
`PickSample`, `PlaceSample`, `Pipettor`. `MTCExecution` is the
orchestrator action that composes them.

## Packages

| Package | What it is |
|---|---|
| **beambot** | Orchestrator + per-task action servers + MTC stages + detection + MCP server + experimental agent module |
| **beambot_interfaces** | 9 `.action` definitions (8 task types + `MTCExecution`) |
| **mtc_gui** | PyQt5 operator cockpit — primary manual interface. Entry: `ros2 run mtc_gui mtc_gui_client` |
| **custom-ur-descriptions** | UR5e URDF + one MoveIt config that branches per gripper via SRDF xacro |
| **vision** | `vcs import`ed — `zivid-ros` driver. Listed in `vision.repos` but ZED is broken / not launched |
| **end_effectors** | `vcs import`ed — Hand-E, ePick, pipettor, 2FG7 drivers plus `epick_config` site overlay |
| **bluesky_ros** | Ophyd wrapper for `/beambot_execution`. **Currently broken** — not synced with PickSample / PlaceSample split. Tracked for eventual revival |
| **cms** | CMS beamline assets: `poses.yaml`, `beamtime_poses.yaml`, `experiments.md`, `tasks/*.json` |
| **demos** | `hello_orchestrator_py` tutorial package |

`src/end_effectors/{serial,robotiq_hande_*,ros2_epick_gripper,pipettor,onrobot_2fg7_*}`
and `src/vision/zivid-ros` are `vcs import`ed (see `*.repos` files) and
gitignored at this repo level. Fix upstream, then bump the ref.

## Hardware

- **Robot**: Universal Robots UR5e (6-DOF). Fixed 20 % velocity /
  acceleration scaling; see the `hardware_capabilities` review for why
  that's intentional.
- **Primary camera**: Zivid 2+ 3D, eye-in-hand, single-shot. ArUco +
  contour detection drives vision-guided picks.
- **Secondary camera**: ZED stereo. Hardware mounted, driver not
  launched by us, subscriptions in `beambot_mcp_server.py` are
  effectively dead. Don't build on it until the driver is stabilised.
- **Grippers** (swappable via `tool_exchange`): Robotiq **Hand-E**
  (mechanical), Robotiq **ePick** (vacuum), OnRobot **2FG7**, custom
  **pipettor**. Each has an SRDF xacro + MoveIt config folder.

### Hand-Eye Calibration History

Zivid is mounted on the arm; the transform `tool0 → zivid_optical_frame`
lives in `cms_robot_description/urdf/zivid_camera_mount.xacro`.
Re-run calibration when the robot moves, the mount is disturbed, or
vision accuracy degrades.

| Date | xyz (m) | rpy (rad) | Notes |
|---|---|---|---|
| **2026-03-27** | 0.05635 0.10228 0.06025 | -0.03622 0.05218 3.13437 | Current. Beamline recalibration, 12 poses (pose 4 excluded). Residuals: rot < 0.33°, trans < 1.53 mm |
| 2026-03-25 | 0.05475 0.10491 0.06013 | -0.03489 0.05317 3.13637 | After URDF chain changes. Residuals: rot < 0.15°, trans < 1.0 mm |
| 2026-01-15 | 0.05675 0.10322 0.05489 | -0.00615 0.04362 3.13541 | Residuals: rot < 0.22°, trans < 0.47 mm |
| 2026-01-13 | 0.05646 0.10182 0.05680 | -0.03542 0.04745 3.13222 | Robot moved to new room |
| 2025-12-17 | 0.05659 0.10548 0.05660 | -0.01432 0.04829 3.13430 | Original location |
| 2025-10-09 | 0.02803 0.07664 0.0 | 0.53964 -1.53712 -2.13794 | Initial calibration (likely different mount) |

Tool: Zivid Studio → Tools → Hand-Eye Calibration, or
`zivid-python-samples/.../hand_eye_calibration/hand_eye_gui.py`.

## Build, launch, test

Host: Ubuntu 24.04 with ROS 2 **Jazzy**. The active branch is
`jazzy_dev`; `humble-dev` is kept as a legacy archive for any deployment
still on Humble.

```bash
# 1. Source ROS 2 (robot machine)
source /opt/ros/jazzy/setup.bash

# 2. Import vcs subtrees (first time only, or on ref bumps)
vcs import src           < src/ros2.repos
vcs import src/end_effectors < src/end_effectors/end_effectors.repos
vcs import src/vision    < src/vision/vision.repos

# 3. Build (skip epick_moveit_studio — incompatible with Jazzy ros2_control)
colcon build --packages-skip epick_moveit_studio
source install/setup.bash

# 4. Launch
ros2 launch beambot beambot_bringup.launch.py                             # real hardware
ros2 launch beambot beambot_bringup.launch.py use_mock_hardware:=true     # simulation
ros2 launch beambot beambot_bringup.launch.py enable_vision:=false        # skip Zivid
ros2 launch beambot beambot_bringup.launch.py enable_pipettor:=false      # skip pipettor
ros2 run mtc_gui mtc_gui_client                                           # operator GUI

# 5. MCP stack (rosbridge + bringup + rosbag recording)
./start_mcp.sh
```

### Realtime scheduling (new robot machine, one-time)

`ros2_control_node` runs the 500 Hz control loop and wants FIFO realtime
priority. Without it the log shows `Could not enable FIFO RT scheduling
policy ... Operation not permitted` and `Overrun detected` lines. Grant the
user RT limits once per machine:

```bash
echo -e "@realtime - rtprio 98\n@realtime - memlock 8388608" | sudo tee /etc/security/limits.d/30-realtime.conf
sudo groupadd -f realtime
sudo usermod -aG realtime "$USER"
# then REBOOT (or full logout) — group + limits only apply to a fresh login
```

Verify after reboot: `ulimit -r` prints `98` (not `0`), and the bringup log
shows `Successful set up FIFO RT scheduling policy with priority 50`.
Persists across reboots. (Residual `Write time`-heavy overruns under the
Hand-E gripper are a separate issue — blocking Modbus I/O in the control
loop, not scheduling — and don't affect execution.)

### Tests and lint

```bash
./test.sh                   # colcon test --merge-install + colcon test-result --verbose
pre-commit run --all-files  # ruff + ament linters
ruff check --fix && ruff format
```

Unit tests live in `src/beambot/test/test_*.py` (orchestrator parsing,
batch planner, algorithms, base_stages). No hardware-in-the-loop tests
in the repo.

CI on push / PR to `main`, `humble`, `jazzy_dev`:
`ruff.yml`, `super-linter.yml`. Docker publish is `workflow_dispatch` only.
(`ros.yaml` — ament C++ test/lint — was removed; the repo is Python-only
now, so it had no C++ to lint and failed on rosdep.)

## Debugging

```bash
ros2 action list                              # check action servers registered
ros2 run tf2_tools view_frames                # dump TF tree (vision)
ros2 topic echo /joint_states                 # joint states
ros2 topic echo /beambot/current_gripper      # what the orchestrator thinks is attached
ros2 topic echo /beambot/execution_state      # IDLE / EXECUTING / PAUSED
ros2 service call /beambot/pause std_srvs/srv/Trigger
ros2 service call /beambot/resume std_srvs/srv/Trigger
tail -f /tmp/beambot_launch.log               # written by start_mcp.sh for get_recent_logs
```

## File locations

| What | Where |
|---|---|
| Action definitions | `src/beambot_interfaces/action/` |
| Orchestrator | `src/beambot/beambot/action_servers/orchestrator.py` |
| Action servers | `src/beambot/beambot/action_servers/` |
| MTC stage implementations | `src/beambot/beambot/stages/` |
| MoveIt lifecycle + vacuum watchdog | `src/beambot/beambot/core/` |
| Detection (OpenCV + YOLO) | `src/beambot/beambot/detection/` |
| Camera drivers | `src/beambot/beambot/camera/` (zivid active; zed broken) |
| Experimental agent module | `src/beambot/beambot/agent/` |
| Agent system prompt | `src/beambot/beambot/agent/robot_operation.md` |
| MCP server (beambot) | `src/beambot/mcp/beambot_mcp_server.py` |
| Beamline config | `src/beambot/config/cms_beamline.yaml` (active beamline via `$BEAMBOT_BEAMLINE_CONFIG`) |
| Pose registry | `src/cms/poses.yaml` (path hardcoded in MCP server) |
| MoveIt configs (per gripper) | `src/custom-ur-descriptions/cms_moveit_config/config/` |
| SRDF xacros (per gripper) | `src/custom-ur-descriptions/cms_moveit_config/srdf/` |
| Launch files | `src/beambot/launch/` |
| Per-action field reference | [`src/beambot/beambot/agent/robot_operation.md`](../src/beambot/beambot/agent/robot_operation.md) (authoritative — also powers the `robot-operation` skill) |
| Hardware capabilities audit | [`docs/hardware_capabilities.md`](./hardware_capabilities.md) |
| Isaac Sim integration notes | [`docs/isaac_sim_integration.md`](./isaac_sim_integration.md) |

## Design decisions

### Orchestrator owns MoveIt lifecycle

`core/moveit_lifecycle_manager.py` kills and relaunches `move_group` on
every `tool_exchange`. This also cycles `ur_ros2_control_node`, which is
how per-tool voltage + Modbus activation works cleanly. No free /
upstream alternative exists today (verified 2026-05; see the
`moveit_tool_changing` memory for the full research). Don't migrate to
AttachURDF or unified-config patterns without a strong trigger.

### Unified PickSample / PlaceSample (issue #47, 2026-04-06)

Replaced `pick_and_place` + `vision_pick_place` with unified
`pick_sample` + `place_sample` actions. The new actions:
- use a `use_vision` flag to unify hardcoded and vision modes in one
  action definition;
- use deterministic IK (#51) to eliminate KDL jitter in vision mode;
- include a vacuum-status check after pick (retreat-then-check);
- run inside a single dual action server (`sample_server.py`),
  following the `vision_server.py` pattern;
- support contour detection via the MCP `detect_sample` tool, which
  returns `marker_offset_x / y` that get passed back into the same
  `pick_sample` goal — not a separate MCP step.

### ePick batching disabled

`orchestrator.py` hard-codes
`batching_enabled = self._enable_batching and start_gripper != "epick"`.
When ePick is active the vacuum watchdog must run between every step,
so batching collapses them in a way that hides drops. Do not refactor
this to an opt-out config without explicit sign-off.

### Detection lives in `beambot.detection`

Single source of truth. The old duplication between `camera/zivid.py`
and the MCP server was resolved. `detect_hough_circles`,
`detect_contours_in_image`, `get_3d_position`, and the
`*DetectionParams` dataclasses all live in `beambot/detection/` — do
not re-inline.

## Known issues

### UR driver reverse-interface timeouts under network jitter

UR goals from containers over a VM have intermittently lost the
reverse-interface connection. On Jazzy, the relevant xacro arg is
`robot_receive_timeout` (the Humble-era `keep_alive_count` was renamed
in the Jazzy migration). See
[ur_robot_driver#941](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/941).

### Bluesky-ROS integration broken

`src/bluesky_ros/mtc_ophyd_device.py` is out of date with the PickSample
/ PlaceSample split and the current `MTCExecution` fields. Running
Bluesky end-to-end today will fail. Tracked for eventual revival; not
in scope without explicit work.

### ZED camera broken

Hardware mounted, driver not launched by `beambot_bringup.launch.py`,
subscriptions in the MCP server are effectively dead. Use Zivid for
anything that requires 3D in production.

## References

- [Digital Discovery Paper (2025)](https://doi.org/10.1039/d5dd00036j) — full architecture overview
- [ICRA 2024 Paper](https://doi.org/10.1109/ICRA57147.2024.10611706) — Bluesky-ROS integration
- [MoveIt Task Constructor](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor.html)
