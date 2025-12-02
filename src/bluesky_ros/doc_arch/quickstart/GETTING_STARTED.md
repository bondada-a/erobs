# Getting Started with Bluesky/ROS

**Quick guide to start using Bluesky with your MTC pipeline**

## 🚀 Fastest Way to Start

```bash
cd ~/work/github_ws/erobs
python3 src/bluesky_ros/quick_bluesky_interactive.py
```

This script automatically:
- Initializes ROS 2
- Creates RunEngine
- Creates robot device
- Drops you into interactive Python

Then just run:
```python
>>> RE(bps.abs_set(robot, "task_sequences/complete_sequence.json", wait=True))
```

---

## 📖 Documentation Overview

| File | Purpose | Read When |
|------|---------|-----------|
| **[README.md](README.md)** | Package overview | First time here |
| **[README_BLUESKY.md](README_BLUESKY.md)** | Quick reference | Need quick examples |
| **[BLUESKY_QUICKSTART.md](BLUESKY_QUICKSTART.md)** | Command cheatsheet | Forget a command |
| **[BLUESKY_LOCAL_SETUP.md](BLUESKY_LOCAL_SETUP.md)** | Setup guide | Installation issues |
| **[INTERACTIVE_BLUESKY_GUIDE.md](INTERACTIVE_BLUESKY_GUIDE.md)** | Interactive patterns | Learn workflows |
| **[ASYNC_DEVICE_GUIDE.md](ASYNC_DEVICE_GUIDE.md)** | Async features | Need wait=False |

---

## 🎯 Choose Your Path

### Path 1: "I just want to run a task"

```bash
python3 src/bluesky_ros/simple_mtc_bluesky.py task_sequences/my_task.json
```

### Path 2: "I want to experiment interactively"

```bash
python3 src/bluesky_ros/quick_bluesky_interactive.py
```

### Path 3: "I want full control"

```python
import rclpy
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device_async import MTCExecutionDeviceAsync
import bluesky.plan_stubs as bps

rclpy.init()
RE = RunEngine({})
robot = MTCExecutionDeviceAsync(name="ur5e_robot", robot_ip="192.168.56.101")

# Your code here
RE(bps.abs_set(robot, "task.json", wait=True))

rclpy.shutdown()
```

---

## 🧪 Verify Your Setup

```bash
python3 src/bluesky_ros/test_bluesky_local.py
```

---

## 💡 Key Concepts

### Devices
- `MTCExecutionDevice` - Original (blocking)
- `MTCExecutionDeviceAsync` - New (non-blocking, cancellable) ⭐ Recommended

### Plans
Python generators that describe workflows:
```python
def my_plan(robot):
    yield from bps.abs_set(robot, "task1.json", wait=True)
    yield from bps.abs_set(robot, "task2.json", wait=True)
```

### RunEngine
Executes plans:
```python
RE = RunEngine({})
RE(my_plan(robot))
```

---

## 🎓 Next Steps

1. ✅ Run quick interactive script
2. ✅ Try running a task
3. ⬜ Read [INTERACTIVE_BLUESKY_GUIDE.md](INTERACTIVE_BLUESKY_GUIDE.md)
4. ⬜ Test async device: `python3 src/bluesky_ros/test_async_device.py`
5. ⬜ Build custom plans
6. ⬜ Integrate with data collection

---

**Need help?** Check [README.md](README.md) for full documentation index.

**Have questions?** All guides have troubleshooting sections.
