# EROBS Container Documentation

Documentation on deploying and testing the EROBS (Extensible Robotic Beamline Scientist) container infrastructure.

> **Status (2026-05):**
> - `erobs-jazzy` is the current production ROS image.
> - `beambot_bsui_minimal` is a **reference** Bluesky client. The
>   `bluesky_ros` Ophyd wrapper it depends on is currently bit-rotted —
>   the container builds but the end-to-end Bluesky → ROS path needs
>   repair before the examples below are expected to work. Other
>   approaches for sending client commands are also being explored.

## Architecture Overview

EROBS uses a two-container architecture for separating concerns:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Beamline Workstation                            │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                     Bluesky RunEngine                            │   │
│  │                    (experiment orchestration)                    │   │
│  └─────────────────────────────┬───────────────────────────────────┘   │
│                                │                                        │
│                                ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │              beambot_bsui_minimal Container                      │   │
│  │  ┌─────────────────────────────────────────────────────────┐    │   │
│  │  │  MTCExecutionDevice (Ophyd)                             │    │   │
│  │  │  - Wraps ROS2 ActionClient                              │    │   │
│  │  │  - Sends JSON task goals                                │    │   │
│  │  │  - Receives feedback/results                            │    │   │
│  │  └─────────────────────────────┬───────────────────────────┘    │   │
│  └────────────────────────────────┼────────────────────────────────┘   │
│                                   │ ROS2 DDS                           │
│                                   │ (MTCExecution Action)              │
│                                   ▼                                    │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    erobs-jazzy Container                        │   │
│  │  ┌─────────────────────────────────────────────────────────┐    │   │
│  │  │  MTCOrchestratorActionServer                            │    │   │
│  │  │  - Parses JSON task goals                               │    │   │
│  │  │  - Dispatches to specialized action servers             │    │   │
│  │  │  - Manages MoveIt lifecycle                             │    │   │
│  │  └─────────────────────────────┬───────────────────────────┘    │   │
│  │                                │                                │   │
│  │  ┌─────────────────────────────▼───────────────────────────┐    │   │
│  │  │  Specialized Action Servers                             │    │   │
│  │  │  - MoveToActionServer      (joint/cartesian moves)      │    │   │
│  │  │  - PickPlaceActionServer   (pick and place sequences)   │    │   │
│  │  │  - EndEffectorActionServer (gripper control)            │    │   │
│  │  │  - VisionMoveToActionServer (ArUco-guided moves)        │    │   │
│  │  │  - ToolExchangeActionServer (gripper swapping)          │    │   │
│  │  │  - PipettorActionServer    (liquid handling)            │    │   │
│  │  └─────────────────────────────┬───────────────────────────┘    │   │
│  │                                │                                │   │
│  │  ┌─────────────────────────────▼───────────────────────────┐    │   │
│  │  │  MoveIt Task Constructor (MTC)                          │    │   │
│  │  │  - Motion planning                                      │    │   │
│  │  │  - Trajectory execution                                 │    │   │
│  │  └─────────────────────────────┬───────────────────────────┘    │   │
│  └────────────────────────────────┼────────────────────────────────┘   │
│                                   │                                    │
└───────────────────────────────────┼────────────────────────────────────┘
                                    │ UR RTDE Protocol
                                    ▼
                          ┌─────────────────┐
                          │   UR5e Robot    │
                          │  + Grippers     │
                          │  + Zivid Camera │
                          └─────────────────┘
```

## Container Images

### erobs-jazzy (Full ROS Stack)

**Purpose**: Runs the complete robotics stack - motion planning, action servers, hardware drivers.

**Registry**: `ghcr.io/bondada-a/erobs-jazzy:latest`

**Contains**:
- ROS 2 Jazzy (full desktop, Ubuntu 24.04 base)
- MoveIt 2 + MoveIt Task Constructor
- UR robot driver
- Zivid camera SDK and ROS 2 driver
- Gripper drivers (Robotiq Hand-E, ePick, Pipettor)
- All beambot action servers
- rosbridge server
- VNC server for RViz visualization
- Claude Code CLI

**Dockerfile**: [`docker/jazzy/Dockerfile`](../docker/jazzy/Dockerfile)

---

### beambot_bsui_minimal (Bluesky Client — reference)

**Purpose**: Lightweight container for Bluesky ↔ ROS 2 communication. Only contains what's needed to send action goals.

**Registry**: `ghcr.io/bondada-a/beambot_bsui_minimal:latest`

**Contains**:
- ROS 2 Jazzy (ros-core only)
- `rclpy` (ROS 2 Python client)
- `beambot_interfaces` (action message definitions)
- `bluesky_ros` (Ophyd device wrapper)
- Pre-saved task JSONs (`cms/tasks/`)
- Bluesky, Ophyd, nslsii
- EPICS base
- Miniconda

**Dockerfile**: [`docker/bsui-minimal/Dockerfile`](../docker/bsui-minimal/Dockerfile)

---

## Building the Images

### Pulling Pre-built Images

```bash
# Pull from GitHub Container Registry
docker pull ghcr.io/bondada-a/erobs-jazzy:latest
docker pull ghcr.io/bondada-a/beambot_bsui_minimal:latest
```

---

> **NOTE**: On NSLS2 devices (workstations and VMs), replace `docker` with `podman` in all commands below. Podman is the container runtime installed on NSLS2 infrastructure and uses the same command syntax as Docker.

---

## Running the Containers

### erobs-jazzy (ROS Stack)

#### Simulation Mode (No Hardware)

```bash
# Allow X server connections (run once per session)
xhost +local:docker

docker run -it --rm \
    --network host \
    --ipc=host \
    --pid=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --name beambot_ros \
    erobs-jazzy \
    /bin/bash -c "source /root/ws/erobs/install/setup.bash && \
                  ros2 launch beambot beambot_bringup.launch.py use_fake_hardware:=true enable_vision:=false"
```
---

### beambot_bsui_minimal (Bluesky Client)

#### Interactive Shell

```bash
docker run -it --rm \
    --network host \
    --ipc=host \
    --pid=host \
    --name beambot_bsui \
    beambot_bsui_minimal \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
                  source /ros2_ws/install/setup.bash && \
                  bash"
```

#### With BSUI

```bash
docker run -it --rm \
    --network host \
    --ipc=host \
    --pid=host \
    -v /nsls2/conda/envs:/nsls2/conda/envs:ro \
    --name beambot_bsui \
    beambot_bsui_minimal \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
                  source /ros2_ws/install/setup.bash && \
                  bsui"
```
---

## ROS2 Interface

### Primary Action: MTCExecution

This is the main interface for Bluesky to control the robot.

**Action Name**: `/beambot_execution`
**Action Type**: `beambot_interfaces/action/MTCExecution`

#### Goal

```
string full_json    # Complete task script as JSON string
```

#### Result

```
bool success
string error_message
int32 completed_steps
int32 total_steps
```

#### Feedback

```
int32 current_step
string current_action
float32 progress_percentage
string status_message
string current_gripper
```

### Task JSON Format

```json
{
  "start_gripper": "hande",
  "poses": {
    "home": [0, -90, 90, -90, -90, 0],
    "pickup_approach": [10, -80, 100, -110, -90, 10],
    "pickup": [10, -70, 110, -130, -90, 10]
  },
  "tasks": [
    {"task_type": "moveto", "target": "home"},
    {"task_type": "end_effector", "end_effector_action": "open"},
    {"task_type": "moveto", "target": "pickup_approach"},
    {"task_type": "moveto", "target": "pickup", "planning_type": "cartesian"},
    {"task_type": "end_effector", "end_effector_action": "close"}
  ]
}
```

**Note**: Joint poses are in **degrees**, converted to radians internally. The `"poses"` dict is optional — the orchestrator auto-resolves named poses (e.g. `"home"`, `"pickup_approach"`) from the beamline's pose registry (`poses_file` in `default_beamline.yaml`) when they are not supplied inline.


## Using from Bluesky

### Basic Usage

```python
import rclpy
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
from bluesky import RunEngine
import bluesky.plan_stubs as bps

# Initialize ROS2
rclpy.init()

# Create the Ophyd device
robot = MTCExecutionDevice()

# Create RunEngine
RE = RunEngine({})

# Execute a pre-saved task file
RE(bps.abs_set(robot, '/ros2_ws/src/cms/tasks/beamtime/hotplate_to_spincoat.json'))

```
---

## Pre-saved Task Files

Located in `/ros2_ws/src/cms/tasks/` 

**Note**: Tasks with pipettor / camera don't work in simulation.

---


## Quick Start (Testing)

### Terminal 1: Start ROS Stack (Simulation)

```bash
# Allow X server connections (for RViz)
xhost +local:docker

docker run -it --rm --network host --ipc=host --pid=host \
    -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    --name beambot_ros erobs-jazzy \
    /bin/bash -c "source /root/ws/erobs/install/setup.bash && \
                  ros2 launch beambot beambot_bringup.launch.py use_fake_hardware:=true"
```

### Terminal 2: Start Bluesky Client

```bash
docker run -it --rm \
    --network host \
    --ipc=host \
    --pid=host \
    -v /nsls2/conda/envs:/nsls2/conda/envs:ro \
    --name beambot_bsui \
    beambot_bsui_minimal \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
                  source /ros2_ws/install/setup.bash && \
                  bsui"
```

### In BSUI

```python
import rclpy
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
from bluesky import RunEngine
import bluesky.plan_stubs as bps

rclpy.init()
robot = MTCExecutionDevice()
RE = RunEngine({})

# Test with pre-saved task
RE(bps.abs_set(robot, '/ros2_ws/src/cms/tasks/beamtime/hotplate_to_spincoat.json'))
```

---

- **Repository**: https://github.com/bondada-a/erobs
- **Branch**: `jazzy_dev`
