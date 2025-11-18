# Interactive Bluesky Shell for Robot Control

This directory provides an interactive Bluesky environment for controlling the UR5e robot.

## Quick Start

### 1. Start the MTC Action Server (Terminal 1)

```bash
ros2 run mtc_pipeline mtc_orchestrator_action_server
```

### 2. Start Bluesky Interactive Shell (Terminal 2)

```bash
cd /home/aditya/work/github_ws/erobs/src/bluesky_ros
./start_bluesky.sh
```

Or with custom robot IP:
```bash
ROBOT_IP=192.168.1.100 ./start_bluesky.sh
```

Or manually:
```bash
ipython -i bluesky_startup.py
```

## Usage in IPython Shell

Once in the shell, you'll have a fully configured Bluesky environment:

### Basic Commands

```python
# Execute a single task
RE(mv(robot, "beamline_test.json"))

# Use shorthand (auto-adds .json)
RE(mv(robot, "beamline_test"))

# Execute multiple tasks
RE(mv(robot, "task1.json"))
RE(mv(robot, "task2.json"))

# Use full path
RE(mv(robot, "/path/to/custom_task.json"))
```

### Available Objects

| Object | Type | Description |
|--------|------|-------------|
| `robot` | MTCExecutionDevice | UR5e robot device |
| `RE` | RunEngine | Bluesky execution engine |
| `task(name)` | function | Convert filename to full path |
| `mv(dev, file)` | function | Move device to execute task |

### Standard Bluesky Plans

All standard Bluesky plans are available:

```python
# Import more plans if needed
import bluesky.plans as bp
import bluesky.plan_stubs as bps

# Sleep
def my_plan():
    yield from mv(robot, "approach.json")
    yield from bps.sleep(2)  # Wait 2 seconds
    yield from mv(robot, "grasp.json")

RE(my_plan())
```

### Example: Multi-Step Workflow

```python
def sample_exchange_workflow():
    """Complete sample exchange sequence"""
    print("Step 1: Moving to safe position")
    yield from mv(robot, "safe_position.json")

    print("Step 2: Approaching sample dock")
    yield from mv(robot, "dock_approach.json")

    print("Step 3: Picking sample")
    yield from mv(robot, "pick_sample.json")

    print("Step 4: Moving to beamline")
    yield from mv(robot, "beamline_position.json")

    print("Step 5: Placing sample")
    yield from mv(robot, "place_sample.json")

    print("✓ Sample exchange complete!")

# Execute
RE(sample_exchange_workflow())
```

### Example: Error Handling

```python
def safe_execution(task_file, retries=3):
    """Execute task with retry logic"""
    for attempt in range(retries):
        try:
            yield from mv(robot, task_file)
            print(f"✓ Success on attempt {attempt + 1}")
            return
        except Exception as e:
            print(f"✗ Attempt {attempt + 1} failed: {e}")
            if attempt < retries - 1:
                print("Retrying in 5 seconds...")
                yield from bps.sleep(5)
            else:
                print("✗ All retries exhausted")
                raise

RE(safe_execution("beamline_test.json"))
```

### Example: Conditional Execution

```python
def conditional_workflow(sample_type):
    """Different workflows based on sample type"""

    if sample_type == "liquid":
        yield from mv(robot, "pipette_sequence.json")
    elif sample_type == "solid":
        yield from mv(robot, "gripper_sequence.json")
    elif sample_type == "vision":
        yield from mv(robot, "vision_pick_place.json")
    else:
        raise ValueError(f"Unknown sample type: {sample_type}")

RE(conditional_workflow("liquid"))
```

## Environment Configuration

### Robot IP

Set via environment variable:
```bash
export ROBOT_IP=10.68.82.41
./start_bluesky.sh
```

Or in Python after startup:
```python
# This requires recreating the device
robot = MTCExecutionDevice(name="ur5e_robot", robot_ip="192.168.1.100")
```

### Task Directory

Default: `/home/aditya/work/github_ws/erobs/`

Tasks are automatically found in this directory. To use a different location:

```python
# In bluesky_startup.py, edit:
TASK_DIR = Path("/path/to/your/tasks")
```

## Advanced Usage

### Inspect Robot Status

```python
# Check if robot is ready (requires implementing status signals)
robot.name              # 'ur5e_robot'
robot.robot_ip          # '10.68.82.41'
```

### Direct ROS 2 Action Control

For advanced users who want direct control:

```python
# Access underlying action client
robot._action_client

# Manual goal construction
goal = robot.construct_goal_message("custom_task.json")

# Send directly
status = robot.set("my_task.json")
```

### Logging and Debugging

Enable verbose logging:

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# Now all ROS 2 and Bluesky logs will be verbose
RE(mv(robot, "test.json"))
```

### Save/Load Plans

```python
# Save a plan for later
def my_standard_workflow():
    yield from mv(robot, "task1.json")
    yield from bps.sleep(2)
    yield from mv(robot, "task2.json")

# Save to file
import pickle
with open('my_workflow.pkl', 'wb') as f:
    pickle.dump(my_standard_workflow, f)

# Load later
with open('my_workflow.pkl', 'rb') as f:
    loaded_plan = pickle.load(f)

RE(loaded_plan())
```

## Troubleshooting

### "Action server not available"

**Problem:** Robot device can't connect to action server.

**Solution:**
1. Check if action server is running:
   ```bash
   ros2 node list | grep mtc
   ```

2. Start the server:
   ```bash
   ros2 run mtc_pipeline mtc_orchestrator_action_server
   ```

3. Verify connection:
   ```bash
   ros2 action list | grep mtc_execution
   ```

### "Task file not found"

**Problem:** JSON file path is incorrect.

**Solution:**
```python
# List available tasks
import os
from pathlib import Path
for f in Path("/home/aditya/work/github_ws/erobs").glob("*.json"):
    print(f.name)

# Use full path
RE(mv(robot, "/full/path/to/task.json"))
```

### "Goal rejected"

**Problem:** MTC server rejected the task (invalid JSON or constraints).

**Solution:**
1. Validate JSON syntax:
   ```python
   import json
   with open("task.json") as f:
       data = json.load(f)  # Will error if invalid
   ```

2. Test with C++ client first:
   ```bash
   ros2 run mtc_pipeline mtc_action_client_example task.json 10.68.82.41 300
   ```

### Exit the Shell

```python
exit()  # or Ctrl+D
```

ROS 2 will automatically shutdown on exit.

## Files

- `bluesky_startup.py` - Main startup script
- `start_bluesky.sh` - Convenience launcher
- `mtc_ophyd_device.py` - Robot device implementation
- `ophyd_ros.py` - Base ROS 2 integration

## Real Beamline Usage Pattern

This setup mimics how Bluesky is used at NSLS-II:

```python
# At a real beamline, you'd have something like:
from bluesky.plans import count, scan
from ophyd import EpicsMotor

# Your robot acts just like any other "motor"
# Except instead of positions, you give it task files

# Combined with detectors:
def measure_at_position(detector, robot, task_file):
    """Move robot and take measurement"""
    yield from mv(robot, task_file)        # Move robot
    yield from bps.sleep(1)                 # Settle time
    yield from count([detector], num=10)    # Take 10 readings

# This is the power of Bluesky - unified interface!
```

---

**Questions?** See `/home/aditya/work/github_ws/erobs/src/bluesky_ros/BLUESKY_ROS_CONTROL_FLOW.md` for detailed technical documentation.
