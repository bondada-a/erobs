# EROBS Jazzy Migration Plan

**From:** ROS2 Humble Hawksbill (Ubuntu 22.04)
**To:** ROS2 Jazzy Jalisco (Ubuntu 24.04 Noble Numbat)
**Branch:** `jazzy_dev`
**Date:** 2026-03-03

---

## 1. Python API Changes

### 1.1 `warn()` → `warning()` (56+ instances across 15 files)

The `get_logger().warn()` method is deprecated in Jazzy. Must change to `.warning()`.

**Files to update:**

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

## 2. UR Driver XACRO Changes

### 2.1 `keep_alive_count` → `robot_receive_timeout`

The UR driver parameter was renamed in the Jazzy-compatible ur_robot_driver.

**Auto-generated URDF files** (will be regenerated from XACRO):
- `src/custom-ur-descriptions/ur5e_robot_description/urdf/ur_with_zivid_pipettor.urdf`
- `src/custom-ur-descriptions/ur5e_robot_description/urdf/ur_with_zivid_hande.urdf`
- `src/custom-ur-descriptions/ur5e_robot_description/urdf/ur_with_zivid_epick.urdf`
- `src/custom-ur-descriptions/ur5e_robot_description/urdf/ur_standalone.urdf`
- `src/custom-ur-descriptions/ur5e_robot_description/urdf/*_isaac.urdf` (4 files)

**Source XACRO files** (need to check if `keep_alive_count` is parameterized via `ur.ros2_control.xacro` include):
- `src/custom-ur-descriptions/ur3e_hande_moveit_config/config/ur.ros2_control.xacro`
- `src/custom-ur-descriptions/ur5e_moveit_configs/*/config/ur.ros2_control.xacro` (4 files)

### 2.2 Isaac URDF Hardcoded Paths

4 Isaac URDF files contain hardcoded `/opt/ros/humble/share/ur_description/meshes/...` paths.
- Need to update to `/opt/ros/jazzy/share/ur_description/meshes/...`
- Also update `convert_urdf_for_isaac.sh` which generates these

---

## 3. External Repository References

### 3.1 `src/ros2.repos`
- `moveit_task_constructor`: version `humble` → `jazzy`

### 3.2 `src/end_effectors/end_effectors.repos`
- `robotiq_hande_driver`: version `humble-devel` → check for `jazzy-devel` branch
- `robotiq_hande_description`: version `humble-devel` → check for `jazzy-devel` branch
- `ros2_epick_gripper`: stays on `main` (check compatibility)
- `serial`: stays on `ros2` branch
- `pipettor`: stays on `main`

### 3.3 Vision Repos
- Check `src/vision/vision.repos` for humble-pinned branches
- `zivid-ros`: Not in ROS index, needs source build (skip for now, document)

---

## 4. Dockerfiles

All existing Dockerfiles use `osrf/ros:humble-desktop-full`. The new Jazzy Dockerfile will use `osrf/ros:jazzy-desktop-full` with Ubuntu 24.04 (Noble Numbat).

**Existing Dockerfiles (NOT modified — Humble-specific):**
- `docker/erobs-common-img/Dockerfile`
- `docker/beambot_img/Dockerfile`
- `docker/ur-driver/Dockerfile`
- `docker/ur-moveit/Dockerfile`
- `docker/bsui/Dockerfile`
- `docker/bsui-minimal/Dockerfile`
- `docker/erobs_hello_moveit/Dockerfile`
- `docker/ur-example/Dockerfile`
- `docker/azure-kinect/Dockerfile.txt`

**New:** `docker/jazzy/Dockerfile` — built for Jazzy + Ubuntu 24.04

---

## 5. Shell Scripts & Configuration

### 5.1 Launch Scripts (`scripts/pdf-launch-scripts/`)
6 scripts with `ROS_DISTRO=humble`:
- `ur-driver-launch.sh`
- `mtc-moveit-launch.sh`
- `sample-movement-server-launch.sh`
- `hello-talker.sh`
- `ur-hande-driver-launch.sh`
- `robotiq-driver-launch.sh`

### 5.2 Other Config Files
- `start_mcp.sh`: sources `/opt/ros/humble/setup.bash`
- `.mcp.json`: sources `/opt/ros/humble/setup.bash`
- `pixi.toml`: uses `robostack-humble` channel
- `.devcontainer/Dockerfile`: uses `althack/ros2:humble-full`
- `src/bluesky_ros/archive/local_bsui.sh`: references Humble

---

## 6. CI/CD Workflows

- `.github/workflows/ros.yaml`: branches `[main, humble]`
- `.github/workflows/ruff.yml`: branches `[main, humble]`
- `.github/workflows/super-linter.yml`: branches `[main, humble]`, DEFAULT_BRANCH: `humble`

These should be updated to include `jazzy_dev` / `jazzy` branch references.

---

## 7. Package Dependencies (package.xml)

No Humble-specific references in any package.xml files. All 19 packages use standard ROS2 APIs:
- 18 use `ament_cmake`
- 1 uses `ament_python` (mtc_gui)

Key dependencies to verify for Jazzy availability:
- `moveit_ros_planning_interface`
- `moveit_task_constructor_core`
- `ur_robot_driver`
- `cv_bridge`
- `zivid_interfaces` (custom — will build from source)

---

## 8. CMakeLists.txt

No Humble-specific cmake flags found. All packages use standard CMake 3.8+ patterns.

---

## 9. Known Issues & Notes

- **MTC segfault in Jazzy:** Fixed with lazy initialization of rclcpp node (commit 499af1d)
- **zivid-ros:** Not in ROS index — needs source build. Skip for now.
- **mtc_gui README:** References `mtc_pipeline` — actually renamed to `beambot`. Note only, don't fix.
- **numpy<2 pin:** Not needed in Jazzy — removed from Jazzy Dockerfile.
- **Zivid SDK:** Version 2.17.2 u24 packages confirmed available for Ubuntu 24.04.

---

## 10. Migration Priority Order

1. ✅ Python API: `warn()` → `warning()` — 15 files, 47+ instances
2. ✅ UR Driver XACRO: `keep_alive_count` → `robot_receive_timeout` — 12 files
3. ✅ Isaac URDFs: update hardcoded humble paths — 5 files
4. ✅ `.repos` files: update branch versions
5. ✅ Shell scripts: update ROS_DISTRO and setup.bash paths — 8 files
6. ✅ Config files: `.mcp.json`, `pixi.toml`, `.devcontainer`
7. ✅ Create Jazzy Dockerfile (`docker/jazzy/Dockerfile`)
8. ✅ Docker build test — **20/20 packages build successfully**
9. ✅ Research end effector driver Jazzy branches
   - `robotiq_hande_driver`: ✅ `jazzy-devel` branch exists, updated
   - `robotiq_hande_description`: stays on `humble-devel` (no jazzy branch, but pure description pkg — works fine on Jazzy)
   - `ros2_epick_gripper`: stays on `main`
   - `serial`: stays on `ros2`
   - `pipettor`: stays on `main`
10. ✅ CI/CD workflow updates

---

## 11. Additional Build Fixes (discovered during colcon build)

- **`cv_bridge` include:** `cv_bridge/cv_bridge.h` → `cv_bridge/cv_bridge.hpp` (Jazzy)
- **MoveIt includes:** All `.h` headers → `.hpp` (deprecated in Jazzy MoveIt 2.12.x)
- **`MoveGroupInterface::Plan`:** `trajectory_` member → `trajectory` (renamed in Jazzy)
- **`computeCartesianPath`:** `jump_threshold` parameter deprecated — removed

---

## Build Notes

- **2026-03-04/05:** All migration phases completed. Docker colcon build: 20/20 packages pass.
- **Zivid SDK:** u24 packages confirmed at `downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u24/amd64/`.
- **Packages skipped in build** (external deps, not part of EROBS):
  - ZED: `zed_components`, `zed_ros2`, `zed_wrapper`, `zed_debug`
  - Zivid: `zivid_camera`, `zivid_interfaces` (need source from zivid-ros)
  - End effectors: `robotiq_hande_*`, `epick_*`, `serial`, `pipette_driver`
  - MTC: `moveit_task_constructor_*` (installed from apt as `ros-jazzy-moveit-task-constructor-core`)
- **Remaining warnings** (non-blocking, pre-existing):
  - `pdf_beamtime`: unused parameter warning, overloaded-virtual warning

