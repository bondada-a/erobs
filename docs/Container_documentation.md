# EROBS Container Documentation

Documentation for DSSI and collaborators on deploying and testing the EROBS (Extensible Robotic Beamline Scientist) container infrastructure.

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
│  │                    beambot_img Container                         │   │
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

### beambot_img (Full ROS Stack)

**Purpose**: Runs the complete robotics stack - motion planning, action servers, hardware drivers.

**Registry**: `ghcr.io/bondada-a/beambot_img:latest`

**Contains**:
- ROS2 Humble (full desktop)
- MoveIt 2 + MoveIt Task Constructor
- UR robot driver
- Zivid camera SDK and ROS2 driver
- Gripper drivers (Robotiq Hand-E, ePick, Pipettor)
- All beambot action servers
- VNC server for RViz visualization

**Size**: ~8 GB

**Hardware Requirements**:
- GPU recommended for Zivid SDK (can run in CPU mode)
- Network access to robot (10.69.26.90) and camera (10.68.81.52)

---

### beambot_bsui_minimal (Bluesky Client)

**Purpose**: Lightweight container for Bluesky ↔ ROS2 communication. Only contains what's needed to send action goals.

**Registry**: `ghcr.io/bondada-a/beambot_bsui_minimal:latest`

**Contains**:
- ROS2 Humble (ros-core only)
- `rclpy` (ROS2 Python client)
- `beambot_interfaces` (action message definitions)
- `bluesky_ros` (Ophyd device wrapper)
- Pre-saved task JSONs (`cms/tasks/`)
- Bluesky, Ophyd, nslsii
- EPICS base
- Miniconda

**Size**: ~1.5 GB

**Hardware Requirements**: None (pure software)

---

## Building the Images

### From Source

```bash
# Clone the repository
git clone https://github.com/bondada-a/erobs.git -b humble-experimental
cd erobs

# Build beambot_img (full ROS stack)
docker build -f docker/erobs-common-img/Dockerfile -t beambot_img .

# Build beambot_bsui_minimal (Bluesky client)
docker build -f docker/bsui-minimal/Dockerfile -t beambot_bsui_minimal .
```

### Using the Build Script

```bash
cd erobs

# Build specific image
./docker/img_build.sh beambot_img
./docker/img_build.sh beambot_bsui_minimal

# Build all images
./docker/img_build.sh all
```

### Pulling Pre-built Images

```bash
# Pull from GitHub Container Registry
docker pull ghcr.io/bondada-a/beambot_img:latest
docker pull ghcr.io/bondada-a/beambot_bsui_minimal:latest

# Tag for local use
docker tag ghcr.io/bondada-a/beambot_img:latest beambot_img
docker tag ghcr.io/bondada-a/beambot_bsui_minimal:latest beambot_bsui_minimal
```

---

## Running the Containers

### beambot_img (ROS Stack)

#### Simulation Mode (No Hardware)

```bash
docker run -it --rm \
    --network host \
    --ipc=host \
    --pid=host \
    --name beambot_ros \
    beambot_img \
    /bin/bash -c "source /root/ws/erobs/install/setup.bash && \
                  ros2 launch beambot beambot_bringup.launch.py use_fake_hardware:=true"
```

#### With Real Hardware

```bash
docker run -it --rm \
    --network host \
    --ipc=host \
    --pid=host \
    --privileged \
    -e ROBOT_IP=10.69.26.90 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --name beambot_ros \
    beambot_img \
    /bin/bash -c "source /root/ws/erobs/install/setup.bash && \
                  ros2 launch beambot beambot_bringup.launch.py"
```

#### With VNC (for RViz)

```bash
docker run -it --rm \
    --network host \
    -p 5901:5901 \
    --name beambot_ros \
    beambot_img

# Connect via VNC client to localhost:5901
```

---

### beambot_bsui_minimal (Bluesky Client)

**Important**: The `--ipc=host` and `--pid=host` flags are required for ROS2 DDS shared memory communication between containers.

#### Interactive Shell

```bash
docker run -it --rm \
    --network host \
    --ipc=host \
    --pid=host \
    --name beambot_bsui \
    beambot_bsui_minimal \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  source /ros2_ws/install/setup.bash && \
                  bash"
```

#### With IPython

```bash
docker run -it --rm \
    --network host \
    --ipc=host \
    --pid=host \
    --name beambot_bsui \
    beambot_bsui_minimal \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  source /ros2_ws/install/setup.bash && \
                  ipython"
```

#### With Beamline Conda Environment (at NSLS-II)

```bash
docker run -it --rm \
    --network host \
    --ipc=host \
    --pid=host \
    -v /nsls2/conda/envs:/nsls2/conda/envs:ro \
    -e BS_ENV="/nsls2/conda/envs/2025-2.2-py310-tiled" \
    -e BS_PROFILE="collection" \
    --name beambot_bsui \
    beambot_bsui_minimal
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

**Note**: Joint poses are in **degrees**, converted to radians internally.

### Available Task Types

| Task Type | Description | Key Parameters |
|-----------|-------------|----------------|
| `moveto` | Move to named pose | `target`, `planning_type` (joint/cartesian) |
| `end_effector` | Control gripper | `end_effector_action` (open/close) |
| `pick_and_place` | Full pick-place sequence | `pick_approach`, `pick_target`, `place_approach`, `place_target` |
| `vision_moveto` | Move to ArUco marker | `tag_id`, `z_offset` |
| `vision_pick_place` | Vision-guided pick + hardcoded place | `tag_id`, `sample_approach`, `place_target` |
| `tool_exchange` | Swap grippers | `operation` (dock/load), `gripper`, `dock_number` |
| `pipettor` | Liquid handling | `operation` (SUCK/EJECT_TIP), `volume_pct` |

---

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
mtc = MTCExecutionDevice()

# Create RunEngine
RE = RunEngine({})

# Execute a pre-saved task file
RE(bps.abs_set(mtc, '/ros2_ws/src/cms/tasks/bsui_test.json'))

# Or execute inline JSON
task = '''
{
  "start_gripper": "hande",
  "poses": {"home": [0, -90, 90, -90, -90, 0]},
  "tasks": [{"task_type": "moveto", "target": "home"}]
}
'''
RE(bps.abs_set(mtc, task))
```

### Using TaskBuilder

```python
from bluesky_ros.task_builder import TaskBuilder

# Create builder with your locations file
builder = TaskBuilder('src/cms/tasks/complete_sequence.json')

# See available locations
builder.list_locations()

# Build and execute tasks
json_file = builder.move_to('home')
RE(bps.abs_set(mtc, json_file))

# Pick sequence
json_file = builder.pick_sequence(
    approach='pickup_approach',
    grasp='pickup',
    retreat='post_pickup'
)
RE(bps.abs_set(mtc, json_file))
```

---

## Pre-saved Task Files

Located in `/ros2_ws/src/cms/tasks/` (environment variable: `$EROBS_TASKS`)

| File | Description |
|------|-------------|
| `bsui_test.json` | Basic test for Bluesky integration |
| `pick_place_test.json` | Pick and place sequence |
| `tool_exchange.json` | Gripper swap operations |
| `vision_test_simple.json` | Vision-guided movement |
| `complete_sequence.json` | Full workflow with all poses defined |

---

## Environment Variables

### beambot_img

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBOT_IP` | `10.69.26.90` | UR5e robot IP address |
| `REVERSE_IP` | `10.69.26.42` | Host IP for reverse connection |
| `UR_TYPE` | `ur5e` | Robot type |
| `LAUNCH_RVIZ` | `false` | Launch RViz on startup |

### beambot_bsui_minimal

| Variable | Default | Description |
|----------|---------|-------------|
| `EROBS_TASKS` | `/ros2_ws/src/cms/tasks` | Path to pre-saved task JSONs |
| `BS_ENV` | `/nsls2/conda/envs/2025-2.2-py310-tiled` | Beamline conda environment |
| `BS_PROFILE` | `collection` | IPython profile for bsui |
| `ROS_DOMAIN_ID` | `0` | ROS2 domain for DDS isolation |

---

## Network Configuration

### ROS2 DDS Communication

Both containers must be on the same network for ROS2 DDS discovery:

```bash
# Option 1: Host networking (simplest)
docker run --network host ...

# Option 2: Shared Docker network
docker network create ros_net
docker run --network ros_net ...
```

### ROS_DOMAIN_ID

Use `ROS_DOMAIN_ID` to isolate ROS2 communication:

```bash
# Both containers must use the same domain
docker run -e ROS_DOMAIN_ID=42 ... beambot_img
docker run -e ROS_DOMAIN_ID=42 ... beambot_bsui_minimal
```

---

## Troubleshooting

### Check if ROS2 nodes are communicating

```bash
# In beambot_bsui_minimal container
ros2 node list
# Should see: /beambot_orchestrator, /moveit_node, etc.

ros2 action list
# Should see: /beambot_execution

ros2 action info /beambot_execution
# Should show action type and server node
```

### Test action server manually

```bash
# Send a simple goal from command line
ros2 action send_goal /beambot_execution beambot_interfaces/action/MTCExecution \
  "{full_json: '{\"start_gripper\": \"hande\", \"poses\": {\"home\": [0, -90, 90, -90, -90, 0]}, \"tasks\": [{\"task_type\": \"moveto\", \"target\": \"home\"}]}'}"
```

### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| "Action server not available" | beambot_img not running | Start beambot_img container first |
| "No nodes discovered" | Missing `--ipc=host` | Add `--ipc=host --pid=host` to docker run (required for DDS shared memory) |
| "No nodes discovered" | Different ROS_DOMAIN_ID | Ensure same domain ID on both containers |
| "Network unreachable" | Not using host networking | Add `--network host` to docker run |

---

## Quick Start (Testing)

### Terminal 1: Start ROS Stack (Simulation)

```bash
docker run -it --rm --network host --ipc=host --pid=host --name beambot_ros beambot_img \
    /bin/bash -c "source /root/ws/erobs/install/setup.bash && \
                  ros2 launch beambot beambot_bringup.launch.py use_fake_hardware:=true"
```

### Terminal 2: Start Bluesky Client

```bash
docker run -it --rm --network host --ipc=host --pid=host --name beambot_bsui beambot_bsui_minimal \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  source /ros2_ws/install/setup.bash && \
                  ipython"
```

### In IPython

```python
import rclpy
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
from bluesky import RunEngine
import bluesky.plan_stubs as bps

rclpy.init()
mtc = MTCExecutionDevice()
RE = RunEngine({})

# Test with pre-saved task
RE(bps.abs_set(mtc, '/ros2_ws/src/cms/tasks/bsui_test.json'))
```

---

## Contact

- **Repository**: https://github.com/bondada-a/erobs
- **Branch**: `humble-experimental`
- **Maintainer**: abondada@bnl.gov
