# Bluesky Local Setup - Quick Reference

## 🚀 One-Line Setup

```bash
./local_bsui.sh
```

That's it! This sets up your entire Bluesky/ROS environment.

## 📋 Common Commands

### 1. Check Environment Status
```bash
./local_bsui.sh
```

### 2. Run Tests
```bash
python3 test_bluesky_local.py
```

### 3. Start Interactive Session
```bash
./local_bsui.sh --ipython
```

### 4. Execute MTC Task with Bluesky
```bash
# Using simple wrapper (subprocess)
python3 src/bluesky_ros/simple_mtc_bluesky.py task_sequences/complete_sequence.json

# With custom robot IP
python3 src/bluesky_ros/simple_mtc_bluesky.py --robot-ip 192.168.1.100 task.json
```

## 🧪 Quick Python Test

```python
import rclpy
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
import bluesky.plan_stubs as bps

# Initialize
rclpy.init()
RE = RunEngine({})
mtc = MTCExecutionDevice(name="mtc", robot_ip="10.68.82.41")

# Run a task
def execute_task(json_path):
    yield from bps.abs_set(mtc, json_path, wait=True)

RE(execute_task("/path/to/task.json"))

# Cleanup
rclpy.shutdown()
```

## 📁 Key Files Created

- **`local_bsui.sh`** - Environment launcher
- **`test_bluesky_local.py`** - Installation test
- **`BLUESKY_LOCAL_SETUP.md`** - Full documentation
- **`BLUESKY_QUICKSTART.md`** - This file

## ✅ What's Installed

- ✓ Bluesky 1.14.4
- ✓ Ophyd 1.11.0
- ✓ Ophyd-Async 0.13.0
- ✓ IPython 8.37.0
- ✓ Tiled 0.2.2
- ✓ Databroker 1.2.5
- ✓ ROS 2 Humble
- ✓ NSLSII packages

## 🔧 Troubleshooting

### Can't import modules?
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"
```

### Or just use the launcher:
```bash
./local_bsui.sh
```

## 📚 Documentation

- Full guide: [BLUESKY_LOCAL_SETUP.md](BLUESKY_LOCAL_SETUP.md)
- Original Docker setup: `docker/bsui/`
- Example code: `src/bluesky_ros/`

## 🎯 Next Steps

1. Test: `python3 test_bluesky_local.py`
2. Explore: `./local_bsui.sh --ipython`
3. Run: `python3 src/bluesky_ros/simple_mtc_bluesky.py <task.json>`
4. Build: Create your own Bluesky plans!
