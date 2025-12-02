# Bluesky/ROS Integration Package

This package provides Bluesky integration for the MoveIt Task Constructor (MTC) pipeline, enabling data acquisition orchestration with robot manipulation tasks.

## 📁 Directory Structure

```
src/bluesky_ros/
├── 📚 Core Implementation
│   ├── mtc_ophyd_device.py              - Original MTC Ophyd device (blocking)
│   ├── mtc_ophyd_device_async.py        - Async MTC Ophyd device (non-blocking) ⭐
│   ├── ophyd_ros.py                     - Base ROS/Ophyd integration classes
│   └── task_builder.py                  - Task construction utilities
│
├── 🚀 Example Scripts
│   ├── simple_mtc_bluesky.py            - Simple MTC task executor
│   ├── mtc_bluesky_example.py           - Full Ophyd device example
│   ├── quick_bluesky_interactive.py     - Quick interactive setup ⭐
│   └── interactive_bluesky.py           - Interactive exploration script
│
├── 🧪 Testing
│   ├── test_async_device.py             - Async device test suite ⭐
│   └── test_bluesky_local.py            - Local installation tests
│
├── 📖 Documentation
│   ├── README_BLUESKY.md                - Quick reference card
│   ├── ASYNC_DEVICE_GUIDE.md            - Async device usage guide ⭐
│   ├── INTERACTIVE_BLUESKY_GUIDE.md     - Interactive workflow guide
│   ├── BLUESKY_LOCAL_SETUP.md           - Local setup documentation
│   └── BLUESKY_QUICKSTART.md            - Quick command reference
│
└── 📂 Archives
    ├── archive/                          - Old implementations
    └── unused/                           - Deprecated code

⭐ = Recently added/updated
```

---

## 🚀 Quick Start

### Option 1: Use the Async Device (Recommended)

```python
import rclpy
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device_async import MTCExecutionDeviceAsync
import bluesky.plan_stubs as bps

rclpy.init()
RE = RunEngine({})
robot = MTCExecutionDeviceAsync(name="ur5e_robot", robot_ip="192.168.56.101")

# Non-blocking execution
RE(bps.abs_set(robot, "task.json", wait=False))

# Or blocking
RE(bps.abs_set(robot, "task.json", wait=True))

rclpy.shutdown()
```

### Option 2: Use the Quick Interactive Script

```bash
cd ~/work/github_ws/erobs
python3 src/bluesky_ros/quick_bluesky_interactive.py
```

### Option 3: Use the Simple Executor

```bash
python3 src/bluesky_ros/simple_mtc_bluesky.py task_sequences/complete_sequence.json
```

---

## 📚 Documentation Guide

### Getting Started
1. **[README_BLUESKY.md](README_BLUESKY.md)** - Start here! Quick overview and examples
2. **[BLUESKY_QUICKSTART.md](BLUESKY_QUICKSTART.md)** - Common commands reference

### Setup & Installation
3. **[BLUESKY_LOCAL_SETUP.md](BLUESKY_LOCAL_SETUP.md)** - Complete local setup guide
4. See also: `../../INSTALLATION_COMPLETE.md` - Full installation summary

### Usage Guides
5. **[INTERACTIVE_BLUESKY_GUIDE.md](INTERACTIVE_BLUESKY_GUIDE.md)** - Interactive workflow patterns
6. **[ASYNC_DEVICE_GUIDE.md](ASYNC_DEVICE_GUIDE.md)** - Async device usage and features

### Testing
7. Run `python3 test_async_device.py` - Test async device functionality
8. Run `python3 test_bluesky_local.py` - Verify installation

---

## 🔧 Core Components

### Devices

#### MTCExecutionDeviceAsync (Recommended)
- **File**: `mtc_ophyd_device_async.py`
- **Features**:
  - ✅ True async execution (`wait=False` works)
  - ✅ Task cancellation via `robot.cancel_goal()`
  - ✅ Background task execution
  - ✅ Proper Bluesky integration
- **Use when**: You need non-blocking execution or cancellation

#### MTCExecutionDevice (Legacy)
- **File**: `mtc_ophyd_device.py`
- **Features**:
  - ✅ Simple, straightforward execution
  - ❌ `wait=False` doesn't work (always blocks)
  - ⚠️ Limited cancellation support
- **Use when**: You only need sequential blocking execution

### Utilities

#### task_builder.py
Task construction utilities for creating MTC task JSON programmatically.

#### ophyd_ros.py
Base classes for ROS/Ophyd integration, including `ActionMovable` base class.

---

## 📊 File Purposes

| File | Purpose | When to Use |
|------|---------|-------------|
| **mtc_ophyd_device_async.py** | Async device implementation | Production use, need wait=False |
| **mtc_ophyd_device.py** | Original blocking device | Legacy code, simple use cases |
| **simple_mtc_bluesky.py** | Command-line task executor | Running single tasks from shell |
| **mtc_bluesky_example.py** | Example integration code | Learning how to use devices |
| **quick_bluesky_interactive.py** | Auto-setup interactive session | Quick testing, development |
| **test_async_device.py** | Async device test suite | Validating async behavior |
| **test_bluesky_local.py** | Installation verification | After setup or updates |

---

## 🎯 Common Use Cases

### 1. Run a Single Task from Command Line

```bash
python3 src/bluesky_ros/simple_mtc_bluesky.py task_sequences/my_task.json
```

### 2. Interactive Development

```bash
python3 src/bluesky_ros/quick_bluesky_interactive.py
```

Then in the interactive session:
```python
>>> RE(bps.abs_set(robot, "task.json", wait=True))
```

### 3. Custom Bluesky Plan

```python
from bluesky_ros.mtc_ophyd_device_async import MTCExecutionDeviceAsync
import bluesky.plan_stubs as bps

def my_workflow(robot, tasks):
    for task in tasks:
        yield from bps.abs_set(robot, task, wait=True)

RE(my_workflow(robot, ["task1.json", "task2.json", "task3.json"]))
```

### 4. Non-Blocking with Cancellation

```python
import time
from bluesky_ros.mtc_ophyd_device_async import MTCExecutionDeviceAsync

robot = MTCExecutionDeviceAsync(name="ur5e", robot_ip="192.168.56.101")

# Start task
status = RE(bps.abs_set(robot, "long_task.json", wait=False))
print("Task started!")

# Do other work...
time.sleep(5)

# Cancel if needed
robot.cancel_goal()
```

---

## 🧪 Testing

### Test Async Device

```bash
cd ~/work/github_ws/erobs
python3 src/bluesky_ros/test_async_device.py
```

Choose option 1 for full test suite or option 2 for quick comparison.

### Test Installation

```bash
python3 src/bluesky_ros/test_bluesky_local.py
```

Verifies all packages are installed correctly.

---

## 🔍 Troubleshooting

### Import Errors

**Problem**: `ModuleNotFoundError: No module named 'bluesky_ros'`

**Solution**: Make sure PYTHONPATH includes the src directory:
```bash
export PYTHONPATH="/home/aditya/work/github_ws/erobs/src:$PYTHONPATH"

# Or use the launcher
./local_bsui.sh --ipython
```

### Device Not Found

**Problem**: `ModuleNotFoundError: No module named 'mtc_ophyd_device_async'`

**Solution**: Use full import path:
```python
from bluesky_ros.mtc_ophyd_device_async import MTCExecutionDeviceAsync
```

### Task Doesn't Cancel

**Problem**: `cancel_goal()` doesn't stop the robot

**Solution**: Make sure you're using `MTCExecutionDeviceAsync` (not the original device):
```python
from bluesky_ros.mtc_ophyd_device_async import MTCExecutionDeviceAsync
```

---

## 🎓 Learning Path

1. **Start here**: [README_BLUESKY.md](README_BLUESKY.md)
2. **Quick commands**: [BLUESKY_QUICKSTART.md](BLUESKY_QUICKSTART.md)
3. **Try examples**: Run `simple_mtc_bluesky.py`
4. **Interactive mode**: Run `quick_bluesky_interactive.py`
5. **Learn async**: Read [ASYNC_DEVICE_GUIDE.md](ASYNC_DEVICE_GUIDE.md)
6. **Custom workflows**: Read [INTERACTIVE_BLUESKY_GUIDE.md](INTERACTIVE_BLUESKY_GUIDE.md)
7. **Advanced features**: Explore `mtc_ophyd_device_async.py` code

---

## 📖 External Resources

- [Bluesky Documentation](https://blueskyproject.io/bluesky/main/index.html)
- [Ophyd Documentation](https://blueskyproject.io/ophyd/main/index.html)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [MoveIt Task Constructor](https://moveit.picknik.ai/main/doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial.html)

---

## 🤝 Contributing

When adding new features:
1. Follow existing code style
2. Add docstrings to all functions
3. Update this README
4. Add tests to `test_async_device.py`
5. Document in appropriate `.md` file

---

## 📝 Version History

- **2025-12-02**: Added async device implementation
- **2025-12-02**: Reorganized files into src/bluesky_ros/
- **2025-12-02**: Created comprehensive documentation
- **Earlier**: Initial Bluesky integration

---

## ✅ Quick Reference Card

```bash
# Test installation
python3 src/bluesky_ros/test_bluesky_local.py

# Test async device
python3 src/bluesky_ros/test_async_device.py

# Quick interactive session
python3 src/bluesky_ros/quick_bluesky_interactive.py

# Run single task
python3 src/bluesky_ros/simple_mtc_bluesky.py task.json

# In Python
from bluesky_ros.mtc_ophyd_device_async import MTCExecutionDeviceAsync
robot = MTCExecutionDeviceAsync(name="ur5e", robot_ip="192.168.56.101")
RE(bps.abs_set(robot, "task.json", wait=False))  # Async!
robot.cancel_goal()  # Cancel!
```

---

**Last Updated**: 2025-12-02
**Maintainer**: Repository owner
**Status**: Production Ready ✅
