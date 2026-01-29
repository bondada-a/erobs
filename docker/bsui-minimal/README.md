# beambot_bsui_minimal

Lightweight Bluesky-ROS2 client container for EROBS.

## Quick Start

```bash
# For ROS2 communication with beambot_img container
docker run -it --rm \
    --network host \
    --ipc=host \
    --pid=host \
    beambot_bsui_minimal
```

**Important**: The `--ipc=host` and `--pid=host` flags are required for ROS2 DDS shared memory communication between containers.

## Usage from Python

```python
import rclpy
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice

# Initialize ROS2
rclpy.init()

# Create the Ophyd device (connects to beambot_execution action server)
mtc = MTCExecutionDevice()

# Use with Bluesky RunEngine
from bluesky import RunEngine
import bluesky.plan_stubs as bps

RE = RunEngine({})

RE(bps.abs_set(mtc, '/ros2_ws/src/cms/tasks/bsui_test.json'))
```

## Pre-saved Task Files

Located in `/ros2_ws/src/cms/tasks/` (also accessible via `$EROBS_TASKS`):

```
tasks/
├── bsui_test.json              # Basic test for bsui integration
├── pick_place_test.json        # Pick and place sequence
├── tool_exchange.json          # Gripper swap operations
├── vision_test_simple.json     # Vision-guided movement
├── complete_sequence.json      # Full workflow with poses
├── beamtime/                   # Beamtime-specific sequences
│   ├── sample_to_hotplate_cached.json
│   ├── vision_scan.json
│   └── ...
└── ...
```

## ROS2 Interface

This container provides a client for the `beambot_execution` action server:

| Action | Type | Description |
|--------|------|-------------|
| `/beambot_execution` | `beambot_interfaces/MTCExecution` | Main task execution interface |

### MTCExecution.action

```
# Goal
string full_json           # Complete task script JSON

# Result
bool success
string error_message
int32 completed_steps
int32 total_steps

# Feedback
int32 current_step
string current_action
float32 progress_percentage
string status_message
string current_gripper
```

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `EROBS_TASKS` | `/ros2_ws/src/cms/tasks` | Path to pre-saved task JSON files |
| `BS_ENV` | `/nsls2/conda/envs/2025-2.2-py310-tiled` | Beamline conda environment path |
| `BS_PROFILE` | `collection` | IPython profile for bsui |
| `ROS_DOMAIN_ID` | `0` | ROS2 domain for DDS communication |
