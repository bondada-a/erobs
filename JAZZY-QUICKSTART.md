# EROBS Jazzy Quick Start

Quick guide for testing the Humble -> Jazzy migration on the `jazzy_dev` branch.

## 1. Run the Smoke Test (no hardware needed)

The Docker image `erobs-jazzy:latest` is pre-built on the dev machine.

```bash
# Verify the image exists
docker images erobs-jazzy:latest

# Run automated smoke test (31 checks)
./scripts/jazzy-smoke-test.sh
```

Expected output: `Results: 31 passed, 0 failed`

## 2. Interactive Docker Shell

```bash
docker run --rm -it \
  --network host \
  -e DISPLAY=$DISPLAY \
  erobs-jazzy:latest bash

# Inside the container:
source /root/ws/erobs/install/setup.bash
ros2 pkg list | grep beambot
ros2 topic list
```

## 3. Test with UR Robot

```bash
docker run --rm -it \
  --network host \
  -e ROBOT_IP=10.69.26.90 \
  -e REVERSE_IP=10.69.26.42 \
  erobs-jazzy:latest bash

# Inside container:
source /root/ws/erobs/install/setup.bash

# Launch UR driver
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e robot_ip:=$ROBOT_IP

# In another terminal (same container or new one):
ros2 launch ur5e_moveit_configs ur5e_moveit.launch.py
```

**Key thing to watch:** Connection stability. The `robot_receive_timeout` is set to 0.2s.
If you see "Connection to reverse interface dropped" errors, increase it in the XACRO files.

## 4. Test with VNC (for RViz)

```bash
docker run --rm -it \
  --network host \
  -p 5901:5901 \
  erobs-jazzy:latest

# VNC viewer: connect to localhost:5901
```

## 5. Rebuild the Docker Image (if needed)

Only needed if you've made code changes on `jazzy_dev`:

```bash
# Force fresh clone from GitHub (bust cache)
docker build -f docker/jazzy/Dockerfile \
  --build-arg CACHEBUST=$(date +%s) \
  -t erobs-jazzy:latest .
```

Build takes ~8 minutes. The `.dockerignore` keeps the context small.

## What Changed

See `docs/jazzy-migration-plan.md` for the full migration details.

**Summary of API changes:**
- Python: `get_logger().warn()` -> `.warning()` (15 files)
- UR Driver: `keep_alive_count` -> `robot_receive_timeout` (12 files)
- MoveIt C++: `.h` includes -> `.hpp` (7 files)
- `plan.trajectory_` -> `plan.trajectory` (1 file)
- `cv_bridge/cv_bridge.h` -> `cv_bridge/cv_bridge.hpp` (1 file)
- `computeCartesianPath`: removed deprecated `jump_threshold` param

## Known Issues

**EPick gripper:** Skipped in build. The `ros2_epick_gripper` package (PickNikRobotics) is
incompatible with Jazzy's ros2_control. Fix is a 4-line change (`set_state` -> `set_lifecycle_state`).
See migration plan Section 9 for details.

**HandE gripper:** Works, but `jazzy-devel` (v0.2.0) is missing two bug fixes from
`humble-devel` (v0.2.2) — a mutex double-write fix and an activation readiness wait.
Watch for ros2_control frequency hiccups during UR integration.

## Full Testing Checklist

See `docs/jazzy-migration-plan.md` Section 12 for the complete hardware testing checklist.
