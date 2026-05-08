# EROBS Jazzy Migration Plan

**From:** ROS2 Humble Hawksbill (Ubuntu 22.04)
**To:** ROS2 Jazzy Jalisco (Ubuntu 24.04 Noble Numbat)
**Branch:** `jazzy_dev`
**Started:** 2026-03-03
**Completed:** 2026-03-05

---

## Status: COMPLETE

All source code migrated. Production Docker image builds 37/37 packages.
Smoke test passes 31/31 checks. Branch pushed to GitHub.

---

## Table of Contents

1. [Migration Summary](#1-migration-summary)
2. [Python API Changes](#2-python-api-changes)
3. [UR Driver XACRO Changes](#3-ur-driver-xacro-changes)
4. [C++ Build Fixes](#4-c-build-fixes-discovered-during-colcon-build)
5. [External Repository References](#5-external-repository-references)
6. [Dockerfiles](#6-dockerfiles)
7. [Shell Scripts & Configuration](#7-shell-scripts--configuration)
8. [CI/CD Workflows](#8-cicd-workflows)
9. [End Effector Compatibility](#9-end-effector-compatibility)
10. [Known Issues & Notes](#10-known-issues--notes)
11. [Build Results](#11-build-results)
12. [Testing Checklist](#12-testing-checklist)

---

## 1. Migration Summary

| Phase | Description | Files | Status |
|-------|-------------|-------|--------|
| 1 | Python `warn()` -> `warning()` | 15 | Done |
| 2 | UR XACRO `keep_alive_count` -> `robot_receive_timeout` | 12 | Done |
| 3 | Isaac URDF paths `/opt/ros/humble` -> `/opt/ros/jazzy` | 5 | Done |
| 4 | `.repos` file branch updates | 2 | Done |
| 5 | Shell scripts `ROS_DISTRO` and paths | 8 | Done |
| 6 | Config files (`.mcp.json`, `pixi.toml`, `.devcontainer`) | 3 | Done |
| 7 | Jazzy Dockerfile + `.dockerignore` | 2 new | Done |
| 8 | C++ build fixes (MoveIt `.hpp`, cv_bridge, trajectory) | 9 | Done |
| 9 | CI/CD workflow updates | 3 | Done |
| 10 | End effector driver research | — | Done |
| 11 | Docker build test (37/37 packages) | — | Done |
| 12 | Smoke test script | 1 new | Done |

---

## 2. Python API Changes

### `warn()` -> `warning()` (47+ instances across 15 files)

The `get_logger().warn()` method is deprecated in Jazzy. Changed to `.warning()`.

| File | Instances |
|------|-----------|
| `src/beambot/beambot/action_servers/orchestrator.py` | 7 |
| `src/beambot/beambot/stages/vision_stages.py` | 11 |
| `src/beambot/beambot/camera/zivid.py` | 8 |
| `src/beambot/beambot/pointcloud_relay.py` | 3 |
| `src/beambot/beambot/action_servers/base_action_server.py` | 1 |
| `src/beambot/beambot/action_servers/vision_server.py` | 1 |
| `src/beambot/beambot/action_servers/vision_pick_place_server.py` | 1 |
| `src/beambot/beambot/core/moveit_lifecycle_manager.py` | 2 |
| `src/beambot/scripts/live_stitcher.py` | 2 |
| `src/beambot/scripts/test_contour_detection.py` | 3 |
| `src/beambot/scripts/test_wafer_detection.py` | 3 |
| `src/beambot/scripts/test_pointcloud_stability.py` | 3 |
| `src/pdf/pdf_beamtime/src/pdf_beamtime_fidpose_redis_client.py` | 1 |
| `src/pdf/pdf_beamtime/src/pdf_beamtime_client.py` | 1 |
| `src/pdf/pdf_beamtime/src/pdf_beamtime_fidpose_client.py` | 1 |

---

## 3. UR Driver XACRO Changes

### 3.1 `keep_alive_count` -> `robot_receive_timeout`

The UR driver renamed this parameter in the Jazzy-compatible `ur_robot_driver`. The old parameter was an integer count (multiplied by 20ms internally); the new parameter is a float in seconds.

```xml
<!-- Humble: keep_alive_count (integer, count * 20ms) -->
<xacro:arg name="keep_alive_count" default="2"/>  <!-- 40ms -->

<!-- Jazzy: robot_receive_timeout (float, seconds) -->
<xacro:arg name="robot_receive_timeout" default="0.2"/>  <!-- 200ms -->
```

**Files updated (12):**
- 4 XACRO source files in `src/custom-ur-descriptions/ur5e_robot_description/urdf/`
- 4 URDF generated files (matching)
- 4 Isaac URDF files (matching)

### 3.2 Isaac URDF Hardcoded Paths

Updated `/opt/ros/humble/share/ur_description/meshes/...` to `/opt/ros/jazzy/share/...` in:
- 4 `*_isaac.urdf` files
- `convert_urdf_for_isaac.sh`

---

## 4. C++ Build Fixes (discovered during colcon build)

| Change | Files | Details |
|--------|-------|---------|
| `cv_bridge/cv_bridge.h` -> `.hpp` | 1 | `aruco_pose/aruco_pose.hpp` |
| MoveIt `.h` includes -> `.hpp` | 7 | All MoveIt headers renamed in Jazzy 2.12.x |
| `plan.trajectory_` -> `plan.trajectory` | 1 | Member renamed in `MoveGroupInterface::Plan` |
| `computeCartesianPath` jump_threshold removed | 1 | `jump_threshold` parameter deprecated |

**MoveIt header files updated:**
- `src/pdf/pdf_beamtime/include/pdf_beamtime/pdf_beamtime_server.hpp`
- `src/pdf/pdf_beamtime/include/pdf_beamtime/tf_utilities.hpp`
- `src/pdf/pdf_beamtime/include/pdf_beamtime/inner_state_machine.hpp`
- `src/demos/hello_moveit/src/hello_moveit.cpp`
- `src/demos/hello_moveit/src/pick_place_repeat_server.cpp`
- `src/demos/hello_moveit/src/pose_subscriber.cpp`
- `src/demos/hello_orchestrator/src/move_server.cpp`

---

## 5. External Repository References

### `src/ros2.repos`
- `moveit_task_constructor`: `humble` -> `jazzy`

### `src/end_effectors/end_effectors.repos`
- `robotiq_hande_driver`: `humble-devel` -> `jazzy-devel`
- `robotiq_hande_description`: stays on `humble-devel` (pure description pkg, works on Jazzy)
- `ros2_epick_gripper`: `main` (PickNikRobotics) -> `jazzy` (bondada-a fork, patched)
- `serial`: stays on `ros2` branch
- `pipettor`: stays on `main`

### `src/vision/vision.repos`
- `zivid-ros`: no changes needed (no branch pin)
- `zed-ros2-wrapper`: no changes needed (skipped in build, no hardware)

---

## 6. Dockerfiles

**New:** `docker/jazzy/Dockerfile` — production Jazzy image based on `osrf/ros:jazzy-desktop-full`

**Also new:** `.dockerignore` — reduces build context from 735MB to ~1KB (Dockerfile clones from git)

**Existing Humble Dockerfiles (NOT modified):**
- `docker/erobs-common-img/Dockerfile`
- `docker/beambot_img/Dockerfile`
- `docker/ur-driver/Dockerfile`, `docker/ur-moveit/Dockerfile`
- `docker/bsui/Dockerfile`, `docker/bsui-minimal/Dockerfile`
- `docker/erobs_hello_moveit/Dockerfile`, `docker/ur-example/Dockerfile`

---

## 7. Shell Scripts & Configuration

### Launch Scripts (`scripts/pdf-launch-scripts/`)
Updated `ROS_DISTRO=humble` -> `jazzy` in 6 scripts:
`ur-driver-launch.sh`, `mtc-moveit-launch.sh`, `sample-movement-server-launch.sh`,
`hello-talker.sh`, `ur-hande-driver-launch.sh`, `robotiq-driver-launch.sh`

### Other Config Files
| File | Change |
|------|--------|
| `start_mcp.sh` | `/opt/ros/humble/setup.bash` -> `/opt/ros/jazzy/setup.bash` |
| `.mcp.json` | Same path update |
| `pixi.toml` | `robostack-humble` -> `robostack-jazzy` |
| `.devcontainer/Dockerfile` | `althack/ros2:humble-full` -> `althack/ros2:jazzy-full` |
| `src/bluesky_ros/archive/local_bsui.sh` | Setup.bash path |

---

## 8. CI/CD Workflows

Added `jazzy_dev` to branch triggers in:
- `.github/workflows/ros.yaml`
- `.github/workflows/ruff.yml`
- `.github/workflows/super-linter.yml`

---

## 9. End Effector Compatibility

### Robotiq HandE (robotiq_hande_driver)
- **Repo:** `AGH-CEAI/robotiq_hande_driver` (formerly referenced as PickNikRobotics)
- **Branch:** `jazzy-devel` (v0.2.0-jazzy, released 2025-08-25)
- **Status:** Builds and works on Jazzy. Tested on real hardware by upstream.
- **Notes:** Updated in `end_effectors.repos`

- **Jazzy-specific changes in the driver:**
  - Controller type changed: `position_controllers/GripperActionController` ->
    `parallel_gripper_action_controller/GripperActionController`
  - `on_init` signature: `HardwareInfo&` -> `HardwareComponentInterfaceParams&`
  - New controller params: `max_effort_interface`, `max_velocity_interface`, `max_velocity`
  - Deprecation warnings suppressed via `#pragma GCC diagnostic` for `loaned_state_interface.hpp`

- **Warning: Missing bug fixes.** The `jazzy-devel` branch (v0.2.0) is 16 commits behind
  `humble-devel` (v0.2.2). Important fixes NOT yet in `jazzy-devel`:
  - **PR #34:** Double write of desired position — `write_output_bytes()` called with locked
    mutex in time-critical block, causing ros2_control frequency hiccups with UR robots
  - **PR #37:** Wait for ready state before starting communication — early comms hit
    uninitialized device after activation
  - If these cause issues, cherry-pick from `humble-devel` or request upstream port

- **Known issue #36:** Effort control may not work correctly — `cmd_force_` constantly
  overwritten to 1.0 regardless of commanded effort (reported against Jazzy config)

### Robotiq HandE Description (robotiq_hande_description)
- **Repo:** `macmacal/robotiq_hande_description` (in `end_effectors.repos`)
- **Branch:** `humble-devel` (no `jazzy-devel` branch at this remote)
- **Status:** Works on Jazzy — pure URDF/mesh description package with no ROS-version-specific APIs
- **Note:** Upstream `AGH-CEAI/robotiq_hande_description` does have `jazzy-devel` and a
  `v0.2.0-jazzy` tag, but the current remote works fine

### Robotiq EPick (ros2_epick_gripper)
- **Repo:** `bondada-a/ros2_epick_gripper` (forked from PickNikRobotics)
- **Branch:** `jazzy`
- **Status:** PATCHED for Jazzy — build fix applied
- **Root Cause:** `ros2_control` API breaking change (PR ros-controls/ros2_control#1683, merged 2024-08-26)

  In Jazzy's `ros2_control` (v4.17.0+), the method `set_state(rclcpp_lifecycle::State)` for
  setting lifecycle state was renamed to `set_lifecycle_state()`. The name `set_state()` was
  repurposed as a template method for setting hardware state interface values.

  The `epick_driver` calls `set_state(rclcpp_lifecycle::State(...))` in 4 locations
  (`epick_gripper_hardware_interface.cpp` lines 173, 198, 262, 282), which no longer
  matches any overload. Exact compiler error:

  ```
  error: no matching function for call to
    'EpickGripperHardwareInterface::set_state(rclcpp_lifecycle::State)'
  note: candidate: 'template<class T> bool
    HardwareComponentInterface::set_state(const SharedPtr&, const T&, bool)'
  note: candidate expects 3 arguments, 1 provided
  ```

  Additionally deprecated (still compiles with warnings):
  - `on_init(const HardwareInfo&)` -> `on_init(const HardwareComponentInterfaceParams&)`
  - `export_state_interfaces()` -> `on_export_state_interfaces()` (shared pointer return)
  - `export_command_interfaces()` -> `on_export_command_interfaces()` (shared pointer return)

- **Upstream status:** No Jazzy issues or PRs filed. Last commit was Feb 2024.
  Repo is dormant — 3 open issues with no responses, no activity in over a year.

- **Fix applied (bondada-a/ros2_epick_gripper, jazzy branch):**
  1. **Done:** Replaced 4x `set_state(rclcpp_lifecycle::State(...))` with
     `set_lifecycle_state(rclcpp_lifecycle::State(...))`
  2. **Done:** Changed `on_init` to use `info_` (base-class-populated) for factory creation,
     forward-compatible with `HardwareComponentInterfaceParams` signature
  3. **Deferred:** `export_state_interfaces()` -> `on_export_state_interfaces()` (deprecated but
     still compiles; even robotiq_hande_driver jazzy-devel keeps the old signature)

- **Previously skipped packages in Docker build (now buildable):**
  `epick_driver`, `epick_description`, `epick_controllers`, `epick_hardware_tests`,
  `epick_moveit_plugin`
- **Still skipped:** `epick_moveit_studio` (depends on paywalled MoveIt Studio)

### Serial
- **Branch:** `ros2`
- **Status:** Builds fine on Jazzy

### Pipettor
- **Branch:** `main`
- **Status:** Builds fine on Jazzy (pure Python, no ROS-version-specific APIs)

---

## 10. Known Issues & Notes

- **MTC segfault in Jazzy:** Fixed with lazy initialization of rclcpp node (commit 499af1d)
- **zivid-ros:** Not in ROS index — builds from source successfully in Docker
- **Zivid SDK:** Version 2.17.2 u24 packages confirmed for Ubuntu 24.04
- **numpy<2 pin:** Not needed in Jazzy
- **`robot_receive_timeout`:** Set to 0.2s (200ms) to handle Docker/VM network latency. Needs hardware validation. See `docs/development_notes.md`.

### Intentionally Kept Humble References
- Existing Humble Dockerfiles in `docker/` (still used for Humble deployments)
- Documentation/README files mentioning Humble
- CI branch lists (still trigger on Humble branches)
- `super-linter.yml` DEFAULT_BRANCH: `humble`
- `robotiq_hande_description`: `humble-devel` (no jazzy branch, works fine)

---

## 11. Build Results

### Production Docker Image
- **Image:** `erobs-jazzy:latest` (11.4GB)
- **Base:** `osrf/ros:jazzy-desktop-full`
- **Result:** 37/37 packages built, 0 failures

### Packages Built Successfully (37)
Includes all EROBS source packages plus:
- `zivid_camera`, `zivid_interfaces`, `zivid_description`, `zivid_rviz_plugin`, `zivid_samples`
- `robotiq_hande_driver` (jazzy-devel), `robotiq_hande_description` (humble-devel)
- `serial`, `pipette_driver`, `pipette_description`
- All `moveit_task_constructor_*` packages (from jazzy branch)

### Packages Skipped (10) — Intentional
- **ZED** (no hardware/SDK): `zed_components`, `zed_ros2`, `zed_wrapper`, `zed_debug`
- **EPick** (ros2_control API incompatible): `epick_moveit_studio`, `epick_driver`, `epick_description`, `epick_controllers`, `epick_hardware_tests`, `epick_moveit_plugin`

### Build Warnings (non-blocking)
- `pdf_beamtime`: unused parameter, overloaded-virtual (pre-existing)
- `rviz_marker_tools`, `moveit_task_constructor_*`: cmake build type info messages

### Smoke Test
Run `./scripts/jazzy-smoke-test.sh` — tests 31 checks including package presence, Python imports, Zivid SDK, MoveIt, and UR driver availability. All pass.

---

## 12. Testing Checklist

Manual testing for when hardware is available:

### Docker Image Basics
- [ ] `docker run --rm erobs-jazzy:latest bash` — container starts
- [ ] `./scripts/jazzy-smoke-test.sh` — all 31 checks pass
- [ ] VNC accessible on port 5901 (for RViz)

### UR Robot (requires UR5e on network)
- [ ] Launch UR driver: `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=<IP>`
- [ ] Verify `robot_receive_timeout=0.2` works (no "Connection to reverse interface dropped" errors)
- [ ] If connection drops persist, try increasing `robot_receive_timeout` to 0.5 or 1.0 in XACRO
- [ ] MoveIt planning works: `ros2 launch ur5e_moveit_configs ur5e_moveit.launch.py`
- [ ] Execute a simple joint move via MoveIt
- [ ] Execute a Cartesian path via MoveIt

### MTC Pipeline
- [ ] Launch MTC demo: verify `moveit_task_constructor_demo` node starts without segfault
- [ ] Run a pick-and-place pipeline through beambot action server
- [ ] Verify MTC planning scene publishes correctly

### End Effectors
- [ ] **HandE gripper:** Launch `robotiq_hande_driver`, verify open/close commands work
- [ ] **HandE gripper:** Watch for ros2_control frequency hiccups (PR #34 fix missing from jazzy-devel)
- [ ] **HandE gripper:** Verify activation sequence doesn't fail on startup (PR #37 fix missing)
- [ ] **HandE gripper:** Test effort/force control — known issue #36 may cause max force regardless of command
- [ ] **Pipettor:** Launch pipettor node, verify serial communication
- [ ] **EPick gripper:** Not available until upstream fix or fork (see [Section 9](#9-end-effector-compatibility))

### Zivid Camera (requires Zivid hardware)
- [ ] `ros2 launch zivid_camera zivid_camera.launch.py` — camera node starts
- [ ] Verify point cloud topic publishes (`ros2 topic echo /zivid/points`)
- [ ] Verify 2D image topic publishes

### Beambot Integration
- [ ] Launch full beambot stack
- [ ] Run vision pipeline (aruco detection, wafer detection)
- [ ] Run orchestrator action server
- [ ] End-to-end pick-and-place with vision

### PDF Beamtime
- [ ] Launch `pdf_beamtime` server
- [ ] Verify Redis client connection
- [ ] Run a sample movement sequence

### Network / Multi-container
- [ ] Verify ROS2 DDS discovery works between containers
- [ ] Test with `ROBOT_IP` and `REVERSE_IP` environment variables set correctly
- [ ] Confirm `socat` port forwarding if needed for VM setups
