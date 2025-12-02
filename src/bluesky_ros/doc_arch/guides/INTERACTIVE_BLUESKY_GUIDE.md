# Interactive Bluesky/ROS Usage Guide

This guide shows how to use Bluesky interactively with your MTC robot, matching your previous workflow.

## 📝 Your Previous Workflow

```python
export PYTHONPATH=/root/ws/erobs/src/bluesky_ros:$PYTHONPATH

import rclpy
rclpy.init()

from mtc_ophyd_device import MTCExecutionDevice
from bluesky import RunEngine
import bluesky.plan_stubs as bps

RE = RunEngine({})
robot = MTCExecutionDevice(name="ur5e_robot", robot_ip="10.68.82.41")

# Non-blocking execution
RE(bps.abs_set(robot, "/root/ws/erobs/tool_exchange_test.json", wait=False))
```

## 🚀 How to Use This Workflow Now

### Option 1: Local Setup (IPython)

#### Start IPython with Environment

```bash
cd /home/aditya/work/github_ws/erobs
./local_bsui.sh --ipython
```

#### In IPython:

```python
# Initialize ROS 2
import rclpy
rclpy.init()

# Import Bluesky components
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
import bluesky.plan_stubs as bps

# Create RunEngine
RE = RunEngine({})

# Create robot device
robot = MTCExecutionDevice(name="ur5e_robot", robot_ip="10.68.82.41")

# Execute task (non-blocking)
RE(bps.abs_set(robot, "/home/aditya/work/github_ws/erobs/tool_exchange_test.json", wait=False))

# Or execute task (blocking - wait for completion)
RE(bps.abs_set(robot, "/home/aditya/work/github_ws/erobs/tool_exchange_test.json", wait=True))

# When done, cleanup
rclpy.shutdown()
```

---

### Option 2: Docker Setup (bsui)

#### Start Docker Container

```bash
docker run -it --network host \
    -e ROBOT_IP=10.68.82.41 \
    ghcr.io/bondada-a/bsui-img-new:latest
```

#### In Container (bsui shell):

```python
# Initialize ROS 2
import rclpy
rclpy.init()

# Import Bluesky components
from bluesky import RunEngine
from mtc_ophyd_device import MTCExecutionDevice  # Already in PYTHONPATH
import bluesky.plan_stubs as bps

# Create RunEngine
RE = RunEngine({})

# Create robot device
robot = MTCExecutionDevice(name="ur5e_robot", robot_ip="10.68.82.41")

# Execute task
RE(bps.abs_set(robot, "/root/ws/erobs/tool_exchange_test.json", wait=False))

# Cleanup
rclpy.shutdown()
```

---

## 🔧 Key Differences: wait=True vs wait=False

`★ Insight ─────────────────────────────────────`
**Bluesky Execution Modes**:
- `wait=True` (default): RunEngine blocks until task completes
- `wait=False`: RunEngine returns immediately, task runs in background
- Use `wait=False` for parallel operations or when you need the prompt back
- Use `wait=True` when you want to ensure task completes before next step
`─────────────────────────────────────────────────`

### Blocking Execution (wait=True)

```python
# This will wait until task completes
RE(bps.abs_set(robot, "/path/to/task.json", wait=True))
print("Task is done!")  # Only prints after completion
```

### Non-Blocking Execution (wait=False)

```python
# This returns immediately
status = RE(bps.abs_set(robot, "/path/to/task.json", wait=False))
print("Task submitted!")  # Prints immediately

# Task is running in background
# You can check status:
# status.done  # True if finished
# status.success  # True if successful
```

---

## 📋 Complete Interactive Session Examples

### Example 1: Single Task (Blocking)

**Local:**
```bash
./local_bsui.sh --ipython
```

**In IPython:**
```python
import rclpy
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
import bluesky.plan_stubs as bps

rclpy.init()
RE = RunEngine({})
robot = MTCExecutionDevice(name="ur5e", robot_ip="10.68.82.41")

# Execute and wait
print("Starting task...")
RE(bps.abs_set(robot, "task_sequences/complete_sequence.json", wait=True))
print("Task completed!")

rclpy.shutdown()
```

---

### Example 2: Multiple Tasks in Sequence

```python
import rclpy
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
import bluesky.plan_stubs as bps

rclpy.init()
RE = RunEngine({})
robot = MTCExecutionDevice(name="ur5e", robot_ip="10.68.82.41")

# Define a plan for multiple tasks
def multi_task_plan(robot, task_list):
    for i, task in enumerate(task_list):
        print(f"Task {i+1}/{len(task_list)}: {task}")
        yield from bps.abs_set(robot, task, wait=True)
        print(f"  ✓ Task {i+1} completed")

# Execute plan
tasks = [
    "task_sequences/task1.json",
    "task_sequences/task2.json",
    "task_sequences/task3.json"
]

RE(multi_task_plan(robot, tasks))

rclpy.shutdown()
```

---

### Example 3: Non-Blocking with Manual Spin

```python
import rclpy
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
import bluesky.plan_stubs as bps
import time

rclpy.init()
RE = RunEngine({})
robot = MTCExecutionDevice(name="ur5e", robot_ip="10.68.82.41")

# Start task non-blocking
print("Submitting task...")
RE(bps.abs_set(robot, "tool_exchange_test.json", wait=False))
print("Task submitted! Doing other work...")

# You can do other things here while task runs
for i in range(5):
    print(f"Other work: {i+1}/5")
    time.sleep(1)

# Wait for completion manually by spinning
print("Waiting for task to complete...")
# (In practice, you'd check robot status or use callbacks)

rclpy.shutdown()
```

---

### Example 4: Custom Plan with Error Handling

```python
import rclpy
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
import bluesky.plan_stubs as bps

def safe_execution_plan(robot, task_file):
    """Plan with error handling"""
    print(f"Executing: {task_file}")
    try:
        yield from bps.abs_set(robot, task_file, wait=True)
        print("✓ Task succeeded")
    except Exception as e:
        print(f"✗ Task failed: {e}")
        # Could retry or handle error

rclpy.init()
RE = RunEngine({})
robot = MTCExecutionDevice(name="ur5e", robot_ip="10.68.82.41")

RE(safe_execution_plan(robot, "tool_exchange_test.json"))

rclpy.shutdown()
```

---

## 🎨 Creating Reusable Plans

### Simple Execution Plan

```python
def execute_mtc_task(robot, json_file, wait=True):
    """Execute a single MTC task"""
    print(f"Starting: {json_file}")
    yield from bps.abs_set(robot, json_file, wait=wait)
    if wait:
        print(f"Completed: {json_file}")
```

### Sequential Plan

```python
def sequential_tasks(robot, task_list):
    """Execute tasks one after another"""
    for idx, task in enumerate(task_list, 1):
        print(f"[{idx}/{len(task_list)}] {task}")
        yield from bps.abs_set(robot, task, wait=True)
        print(f"  ✓ Task {idx} done")
```

### Conditional Plan

```python
def conditional_execution(robot, task_file, condition_func):
    """Execute task only if condition is met"""
    if condition_func():
        print(f"Condition met, executing: {task_file}")
        yield from bps.abs_set(robot, task_file, wait=True)
    else:
        print(f"Condition not met, skipping: {task_file}")
```

### Retry Plan

```python
def retry_execution(robot, task_file, max_retries=3):
    """Execute task with retry logic"""
    for attempt in range(1, max_retries + 1):
        print(f"Attempt {attempt}/{max_retries}")
        try:
            yield from bps.abs_set(robot, task_file, wait=True)
            print("✓ Success!")
            break
        except Exception as e:
            print(f"✗ Failed: {e}")
            if attempt == max_retries:
                print("Max retries reached")
                raise
```

---

## 🔄 Path Differences: Local vs Docker

### Local Paths

```python
# Local setup uses your home directory
task_file = "/home/aditya/work/github_ws/erobs/tool_exchange_test.json"

# Or relative (if you're in the workspace)
task_file = "tool_exchange_test.json"
task_file = "task_sequences/complete_sequence.json"
```

### Docker Paths

```python
# Docker uses /root/ws/erobs
task_file = "/root/ws/erobs/tool_exchange_test.json"

# Or relative from workspace
task_file = "tool_exchange_test.json"
task_file = "task_sequences/complete_sequence.json"
```

### Universal Approach (Auto-detect)

```python
import os

# Auto-detect workspace
if os.path.exists("/root/ws/erobs"):
    WORKSPACE = "/root/ws/erobs"
else:
    WORKSPACE = os.path.expanduser("~/work/github_ws/erobs")

# Use it
task_file = os.path.join(WORKSPACE, "tool_exchange_test.json")
```

---

## 🛠️ Quick Start Scripts

### Create a Quick Start Script

**Local:** Create `/home/aditya/work/github_ws/erobs/quick_bluesky.py`:

```python
#!/usr/bin/env python3
"""Quick Bluesky setup for interactive use"""

import os
import rclpy
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
import bluesky.plan_stubs as bps

# Auto-detect workspace
if os.path.exists("/root/ws/erobs"):
    WORKSPACE = "/root/ws/erobs"
else:
    WORKSPACE = os.path.expanduser("~/work/github_ws/erobs")

# Initialize
print("Initializing Bluesky/ROS...")
rclpy.init()
RE = RunEngine({})

# Create robot device
ROBOT_IP = os.environ.get("ROBOT_IP", "10.68.82.41")
robot = MTCExecutionDevice(name="ur5e_robot", robot_ip=ROBOT_IP)

print(f"✓ Ready! Workspace: {WORKSPACE}")
print(f"✓ Robot IP: {ROBOT_IP}")
print()
print("Usage:")
print("  RE(bps.abs_set(robot, 'task.json', wait=True))")
print()

# Enter interactive mode
import code
code.interact(local=locals())

# Cleanup on exit
rclpy.shutdown()
```

Then use it:
```bash
./local_bsui.sh
python3 quick_bluesky.py
```

---

## 📊 Comparison Table

| Aspect | Local Setup | Docker Setup |
|--------|-------------|--------------|
| **Start Command** | `./local_bsui.sh --ipython` | `docker run -it ghcr.io/bondada-a/bsui-img-new:latest` |
| **Import Path** | `from bluesky_ros.mtc_ophyd_device import ...` | `from mtc_ophyd_device import ...` |
| **Workspace Path** | `/home/aditya/work/github_ws/erobs` | `/root/ws/erobs` |
| **PYTHONPATH** | Auto-set by local_bsui.sh | Pre-configured in image |
| **Conda** | Optional (can use or not) | Available but not activated by default |
| **EPICS** | Available at ~/EPICS | Available at /root/EPICS |

---

## 🎯 Recommended Workflow

### For Development (Local)

```bash
# 1. Start IPython with environment
./local_bsui.sh --ipython

# 2. In IPython, set up once per session
import rclpy
rclpy.init()
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
import bluesky.plan_stubs as bps

RE = RunEngine({})
robot = MTCExecutionDevice(name="ur5e", robot_ip="10.68.82.41")

# 3. Test tasks interactively
RE(bps.abs_set(robot, "task_sequences/test.json", wait=True))

# 4. When done
rclpy.shutdown()
```

### For Production (Docker)

```bash
# 1. Start container
docker run -it --network host \
    -e ROBOT_IP=10.68.82.41 \
    ghcr.io/bondada-a/bsui-img-new:latest

# 2. Inside container, same workflow
# (Paths are /root/ws/erobs instead)
```

---

## 🐛 Common Issues

### Issue: "ModuleNotFoundError: No module named 'mtc_ophyd_device'"

**Solution (Local):**
```bash
# Make sure you ran local_bsui.sh first
./local_bsui.sh --ipython
```

**Solution (Docker):**
```bash
# PYTHONPATH is already set, but verify:
echo $PYTHONPATH
# Should include /root/ws/erobs/src/bluesky_ros
```

### Issue: "Cannot connect to action server"

**Solution:**
```bash
# Make sure MTC action server is running
ros2 action list | grep mtc_execution

# If not, start it in another terminal
ros2 run mtc_pipeline mtc_action_server
```

### Issue: wait=False doesn't work as expected

**Explanation:**
The `MTCExecutionDevice.set()` method contains a spin loop that waits for completion. When using `wait=False`, Bluesky submits the goal but the status handling is managed differently. For true async operation, you may need to modify the device or use Bluesky's concurrent execution patterns.

---

## 📚 Next Steps

1. ✅ Understand the basic workflow
2. ✅ Try single task execution
3. ⬜ Create custom plans for your workflow
4. ⬜ Add error handling and retry logic
5. ⬜ Integrate with data collection (Tiled/Databroker)
6. ⬜ Build complex multi-step procedures

---

**See Also:**
- [BLUESKY_LOCAL_SETUP.md](BLUESKY_LOCAL_SETUP.md) - Full local setup guide
- [DUAL_SETUP_GUIDE.md](DUAL_SETUP_GUIDE.md) - Local + Docker comparison
- [mtc_ophyd_device.py](src/bluesky_ros/mtc_ophyd_device.py) - Device implementation

---

**Last Updated**: 2025-12-02
**Status**: Production Ready ✅
