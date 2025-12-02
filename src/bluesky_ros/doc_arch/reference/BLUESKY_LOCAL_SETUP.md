# Bluesky/ROS Local Setup Guide

This guide explains how to run the Bluesky/ROS integration locally without Docker.

## ✅ What's Already Installed

Your system already has all the core components needed:

- **ROS 2 Humble** - Robot Operating System
- **Python 3.10** - Programming environment
- **Bluesky 1.14.4** - Data acquisition orchestration
- **Ophyd 1.11.0** - Hardware abstraction layer
- **IPython 8.37.0** - Interactive Python shell
- **Databroker 1.2.5** - Data management
- **Tiled 0.2.2** - Data access service

## 🚀 Quick Start

### 1. Source the Environment

Use the provided launcher script to set up all environment variables:

```bash
cd /home/aditya/work/github_ws/erobs
./local_bsui.sh
```

This will:
- Source ROS 2 Humble
- Source your workspace (if built)
- Set up PYTHONPATH for bluesky_ros modules
- Display version information

### 2. Run the Test Suite

Verify everything is working:

```bash
python3 test_bluesky_local.py
```

This tests:
- Package imports
- Bluesky RunEngine functionality
- ROS 2 node creation

### 3. Run a Simple Example

Execute an MTC task with Bluesky:

```bash
# Make sure ROS environment is sourced
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the simple example
python3 src/bluesky_ros/simple_mtc_bluesky.py task_sequences/complete_sequence.json
```

Or specify a custom robot IP:

```bash
python3 src/bluesky_ros/simple_mtc_bluesky.py \
    task_sequences/complete_sequence.json \
    --robot-ip 192.168.1.100
```

## 📁 Key Files

### Scripts

- **`local_bsui.sh`** - Main launcher script for local environment
- **`test_bluesky_local.py`** - Test suite to verify installation
- **`src/bluesky_ros/simple_mtc_bluesky.py`** - Simple MTC task executor
- **`src/bluesky_ros/mtc_bluesky_example.py`** - Full Ophyd device example
- **`src/bluesky_ros/mtc_ophyd_device.py`** - Ophyd device for MTC integration
- **`src/bluesky_ros/ophyd_ros.py`** - Base classes for ROS/Ophyd integration

### Configuration

- **`docker/bsui/bsui.bash`** - Original Docker bsui script (reference)
- **`docker/bsui/Dockerfile`** - Original Docker setup (reference)

## 🔧 Usage Examples

### Interactive IPython Session

Start an interactive session with Bluesky:

```bash
./local_bsui.sh --ipython
```

Then in IPython:

```python
import rclpy
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
import bluesky.plan_stubs as bps

# Initialize ROS 2
rclpy.init()

# Create RunEngine
RE = RunEngine({})

# Create MTC device
mtc = MTCExecutionDevice(
    name="mtc_executor",
    robot_ip="10.68.82.41"  # Your robot IP
)

# Execute a task
def simple_plan(json_file):
    yield from bps.abs_set(mtc, json_file, wait=True)
    print("Task complete!")

# Run it
RE(simple_plan("/path/to/your/task.json"))

# Clean up
rclpy.shutdown()
```

### Running Multiple Tasks

Create a custom plan to run multiple MTC tasks sequentially:

```python
#!/usr/bin/env python3
import rclpy
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
import bluesky.plan_stubs as bps

def multi_task_plan(mtc_device, json_files):
    """Execute multiple MTC tasks in sequence"""
    for i, json_file in enumerate(json_files):
        print(f"Task {i+1}/{len(json_files)}: {json_file}")
        yield from bps.abs_set(mtc_device, json_file, wait=True)
        print(f"✓ Task {i+1} completed")

# Initialize
rclpy.init()
RE = RunEngine({})
mtc = MTCExecutionDevice(name="mtc", robot_ip="10.68.82.41")

# Run tasks
tasks = [
    "task_sequences/task1.json",
    "task_sequences/task2.json",
    "task_sequences/task3.json"
]

RE(multi_task_plan(mtc, tasks))

# Clean up
rclpy.shutdown()
```

### Using the Simple Wrapper (No Ophyd Device)

The `simple_mtc_bluesky.py` script uses subprocess to call the existing MTC client:

```bash
# Single task
python3 src/bluesky_ros/simple_mtc_bluesky.py complete_sequence.json

# Multiple tasks
python3 src/bluesky_ros/simple_mtc_bluesky.py task1.json task2.json task3.json

# Custom robot IP
python3 src/bluesky_ros/simple_mtc_bluesky.py \
    --robot-ip 192.168.1.100 \
    task.json
```

## 🔄 Differences from Docker Setup

### What We Skipped

1. **EPICS** - Only needed if you're using EPICS IOCs for hardware control
2. **Conda** - Using system Python packages instead
3. **Specific conda environment** - Docker uses `2025-2.2-py310-tiled`, we use local pip packages

### What's the Same

- All core Bluesky packages (bluesky, ophyd, databroker, tiled)
- ROS 2 Humble setup
- Integration with MTC pipeline
- Workspace structure

### Path Updates

The `simple_mtc_bluesky.py` script now auto-detects paths:

```python
# Auto-detect Docker vs local
if os.path.exists("/root/ws/erobs"):
    WORKSPACE_ROOT = "/root/ws/erobs"  # Docker
else:
    WORKSPACE_ROOT = os.path.expanduser("~/work/github_ws/erobs")  # Local
```

## 🧪 Architecture

```
┌─────────────────────────────────────────┐
│         Bluesky RunEngine               │
│  (Data acquisition orchestration)       │
└───────────────┬─────────────────────────┘
                │
                │ Plans & Commands
                │
┌───────────────▼─────────────────────────┐
│     MTCExecutionDevice (Ophyd)          │
│  - Implements Bluesky protocols         │
│  - Wraps ROS 2 Action Client            │
└───────────────┬─────────────────────────┘
                │
                │ ROS 2 Actions
                │
┌───────────────▼─────────────────────────┐
│   MTC Pipeline Action Server (C++)      │
│  - Receives task JSON                   │
│  - Executes MoveIt Task Constructor     │
└───────────────┬─────────────────────────┘
                │
                │ Motion Commands
                │
┌───────────────▼─────────────────────────┐
│         UR Robot + Gripper              │
└─────────────────────────────────────────┘
```

## 🐛 Troubleshooting

### Import Errors

If you get import errors, make sure to:

1. Source ROS 2:
```bash
source /opt/ros/humble/setup.bash
```

2. Source your workspace:
```bash
cd /home/aditya/work/github_ws/erobs
source install/setup.bash
```

3. Set PYTHONPATH:
```bash
export PYTHONPATH="/home/aditya/work/github_ws/erobs/src:$PYTHONPATH"
```

Or just use the launcher script:
```bash
./local_bsui.sh
```

### NumPy Version Conflicts

You may see warnings about numpy version conflicts. This is due to:
- `isaacsim-core` requires numpy<2.0.0
- `opencv-python` requires numpy>=2
- `ophyd-async` requires numpy>=2

**Solution**: The core Bluesky + Ophyd functionality works fine with numpy 1.26.4. You can ignore warnings about ophyd-async unless you specifically need it.

### ROS 2 Not Found

If `ros2` command is not found:

```bash
# Check if ROS is installed
ls /opt/ros/humble

# If present, source it
source /opt/ros/humble/setup.bash

# Add to .bashrc for persistence
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Workspace Not Built

If you see "ROS workspace not built yet":

```bash
cd /home/aditya/work/github_ws/erobs

# Build interface packages first
colcon build --packages-select zivid_interfaces pipette_driver

# Build remaining packages (skip Zivid if you don't have the camera)
colcon build --packages-skip zed_components zed_ros2 zed_wrapper \
    epick_moveit_studio zivid_camera zivid_samples
```

## 📚 Additional Resources

- [Bluesky Documentation](https://blueskyproject.io/bluesky/main/index.html)
- [Ophyd Documentation](https://blueskyproject.io/ophyd/main/index.html)
- [Tiled Documentation](https://blueskyproject.io/tiled/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [MoveIt Task Constructor](https://moveit.picknik.ai/main/doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial.html)

## 🎯 Next Steps

1. **Verify your setup** - Run `./local_bsui.sh` to see version info
2. **Test basic functionality** - Run `python3 test_bluesky_local.py`
3. **Try the simple example** - Execute an MTC task with Bluesky
4. **Build custom plans** - Create your own Bluesky plans for complex workflows
5. **Add data collection** - Integrate with Tiled/Databroker for data management

## 💡 Key Concepts

### Bluesky Plans

Plans are Python generators that describe data acquisition sequences:

```python
def my_plan(device, *args):
    """A simple plan"""
    yield from bps.abs_set(device, value, wait=True)
    yield from bps.trigger(detector)
    # etc...
```

### Ophyd Devices

Ophyd devices abstract hardware behind a standard interface:

```python
class MyDevice(Movable):
    def set(self, value):
        # Command hardware
        # Return Status object
        pass
```

### RunEngine

The RunEngine executes plans and manages data flow:

```python
RE = RunEngine({})
RE(my_plan(device, args))
```

---

**Last Updated**: 2025-12-02
**For Issues**: Contact repository maintainer or open an issue on GitHub
