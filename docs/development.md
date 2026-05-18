# EROBS Development Reference

> Operational MCP reference (task JSON, error handling, gotchas) is in the project root `CLAUDE.md`.
> This document covers architecture, development setup, and current work items.
>
> For current task priorities, see [GitHub Issues](https://github.com/bondada-a/erobs/issues).
> Use `gh issue list --label P0` to see critical tasks. Close issues when done, open new ones when you find work.
> Decisions and architectural context are in [`STATUS.md`](../STATUS.md).

## Overview

Autonomous robotic sample handling system for synchrotron beamlines at NSLS-II. Integrates ROS2 robotics with Bluesky experiment orchestration to enable **self-driving beamlines** that can run 24/7 without human intervention.

**Goal**: Make this framework beamline-agnostic so any beamline can use UR robots for their sample manipulation needs.

## Architecture

```
Bluesky RunEngine (experiment orchestration)
         ↓
Ophyd Device (ROS2 Action Client wrapper)
         ↓                                    Claude (LLM via MCP)
MTCOrchestratorActionServer  ←────────────────  beambot-mcp-server / ros-mcp-server
         ↓
Specialized Action Servers (8 types)
    ├── move_to, end_effector, tool_exchange
    ├── vision_moveto, vision_scan
    └── pick_sample, place_sample, pipettor
         ↓
MoveIt Task Constructor (motion planning)
         ↓
UR5e Robot + Grippers
```

**Deployment**: Host machine running ROS2 natively with one of two control interfaces:
- **beambot bringup** (`beambot_bringup.launch.py`): MTC pipeline servers, MoveIt, Zivid SDK, action servers
- **MCP interface** (`start_mcp.sh`): rosbridge + beambot bringup for LLM-driven control via `beambot-mcp-server`
- **GUI interface** (`mtc_gui_client`): Manual task execution via Qt GUI
- **bsui** (separate container): Bluesky/experiment orchestration, sends JSON task goals via ROS2 DDS

## Key Packages

| Package | Purpose |
|---------|---------|
| **beambot** | Python action servers, orchestrator, detection algorithms, MCP server |
| **beambot_interfaces** | Action definitions (8 actions) |
| **mtc_gui** | GUI client for task execution |
| **custom-ur-descriptions** | MoveIt configs per gripper type |
| **vision** | Camera drivers + ROS2 nodes (Zivid 3D eye-in-hand, ZED stereo external) |
| **end_effectors** | Gripper drivers (Hand-E, ePick, pipettor) |
| **bluesky_ros** | Bluesky-ROS integration (Ophyd devices) |
| **cms** | CMS beamline task JSONs and pose registry (`poses.yaml`) |

## Hardware

- **Robot**: UR5e 6-DOF arm
- **Cameras**: Zivid 2+ 3D (eye-in-hand, single-shot), ZED (external, streaming)
- **Grippers** (swappable): Robotiq Hand-E, Robotiq ePick, Pipettor

### Hand-Eye Calibration History

The Zivid camera is mounted on the robot arm (eye-in-hand). The transform `tool0 → zivid_optical_frame` is stored in `cms_robot_description/urdf/zivid_camera_mount.xacro`.

**Re-run calibration when**: Robot is moved to a different location, camera mount is disturbed, or vision accuracy degrades.

| Date | xyz (meters) | rpy (radians) | Notes |
|------|--------------|---------------|-------|
| **2026-03-27** | 0.05635 0.10228 0.06025 | -0.03622 0.05218 3.13437 | Current. Beamline recalibration, 12 poses (pose 4 outlier excluded). Residuals: rot < 0.33°, trans < 1.53mm |
| 2026-03-25 | 0.05475 0.10491 0.06013 | -0.03489 0.05317 3.13637 | Recalibration after URDF chain changes. Residuals: rot < 0.15°, trans < 1.0mm |
| 2026-01-15 | 0.05675 0.10322 0.05489 | -0.00615 0.04362 3.13541 | Residuals: rot < 0.22°, trans < 0.47mm |
| 2026-01-13 | 0.05646 0.10182 0.05680 | -0.03542 0.04745 3.13222 | After robot moved to new room |
| 2025-12-17 | 0.05659 0.10548 0.05660 | -0.01432 0.04829 3.13430 | Original location |
| 2025-10-09 | 0.02803 0.07664 0.0 | 0.53964 -1.53712 -2.13794 | Initial calibration (different mount?) |

**Calibration tool**: `zivid-python-samples/source/applications/advanced/hand_eye_calibration/hand_eye_gui.py` or Zivid Studio → Tools → Hand-Eye Calibration

## Build & Launch
### Build
```bash
colcon build --packages-skip epick_moveit_studio && source install/setup.bash
```
### Option A: Launch Framework+GUI for Manual 
```bash
ros2 launch beambot beambot_bringup.launch.py
ros2 launch beambot beambot_bringup.launch.py use_fake_hardware:=true  # simulation
ros2 run mtc_gui mtc_gui_client  # GUI
```
### Option B: Launch Framework+Rosbridge for MCP based control
```bash
./start_mcp.sh  # Launches rosbridge (port 9090) + beambot bringup for LLM control via MCP
```

Note: Always `source install/setup.bash` after building. Vision requires Zivid camera connected and calibrated.

## Debugging

```bash
ros2 action list                          # Check action servers
ros2 run tf2_tools view_frames            # TF tree (vision issues)
ros2 topic echo /joint_states             # Joint states
ros2 service call /beambot/pause std_srvs/srv/Trigger  # Pause execution
ros2 topic echo /beambot/execution_state  # Monitor state
```

## File Locations

| What | Where |
|------|-------|
| Action definitions | `beambot_interfaces/action/` |
| Action servers | `beambot/beambot/action_servers/` |
| Stage implementations | `beambot/beambot/stages/` |
| Detection algorithms | `beambot/beambot/detection/` |
| MCP server (beambot) | `beambot/mcp/beambot_mcp_server.py` |
| Beamline configs | `beambot/config/default_beamline.yaml` |
| Pose registry | `cms/poses.yaml` |
| MoveIt configs | `custom-ur-descriptions/cms_moveit_configs/` |
| Launch files | `beambot/launch/` |
| MCP detailed reference | `docs/mcp_ros_reference.md` |
| Isaac Sim integration | `docs/isaac_sim_integration.md` |
| Hardware capabilities | `docs/hardware_capabilities.md` |

## Design Decisions

### PickSample / PlaceSample Architecture (issue #47)

**Decision**: Replace `pick_and_place` + `vision_pick_place` with unified `pick_sample` + `place_sample` actions (2026-04-06).

**Context**: The old `pick_and_place` was redundant with the batch planner, and `vision_pick_place` had the KDL jitter bug (#51) and no vacuum check. The new actions:
- Use `use_vision` flag to unify hardcoded and vision modes in one action
- Use deterministic IK (#51) to eliminate jitter in vision mode
- Include vacuum status check after pick (retreat-then-check pattern)
- Run as a dual action server (`sample_server.py`) following the `vision_server.py` pattern
- Support contour detection via MCP's `detect_sample` → `marker_offset_x/y` parameters

**To switch**: In `pick_place_server.py`, change `self._stages.run()` to `self._stages.run_with_gripper_settle()`

## Current Work

See [GitHub Issues](https://github.com/bondada-a/erobs/issues) for active tasks and priorities. Decisions and context in [`STATUS.md`](../STATUS.md).

## Known Issues

### UR Driver Connection in Docker/VM Environments
When sending robot goals from containers over a VM, the UR driver loses connection intermittently due to network latency/jitter. The keepalive mechanism times out before packets arrive. Increased `keep_alive_count` to 10 (200ms timeout) in URDF xacro files — preliminary testing suggests this helps but not fully validated. See [ur_robot_driver#941](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/941).

## References

- [Digital Discovery Paper (2025)](https://doi.org/10.1039/d5dd00036j) - Full architecture
- [ICRA 2024 Paper](https://doi.org/10.1109/ICRA57147.2024.10611706) - Bluesky-ROS integration
- [MoveIt Task Constructor](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor.html)
