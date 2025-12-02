# How to Load Robot Control in bsui

## Method 1: Using %run (Recommended)

**From within bsui IPython shell:**

```python
%run /path/to/erobs/src/bluesky_ros/load_robot.py
```

**Example:**
```python
# Start your normal bsui
$ bsui

# Then in the bsui shell:
In [1]: %run ~/work/github_ws/erobs/src/bluesky_ros/load_robot.py

============================================================
✅ Robot Control Loaded!
============================================================
📍 Loaded 19 locations
🤖 Robot IP: 10.69.26.90

Available objects:
  • builder  - TaskBuilder instance
  • mtc      - MTC Device
  • ROBOT_IP - Robot IP address
============================================================

# Now use it:
In [2]: builder.list_locations()

In [3]: json_file = builder.move_to('pickup_approach')

In [4]: RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))
```

---

## Method 2: Using exec() (Alternative)

```python
exec(open('/path/to/erobs/src/bluesky_ros/load_robot.py').read())
```

---

## Method 3: Copy-Paste (Quick Test)

Just copy and paste this into bsui:

```python
import sys
sys.path.insert(0, '/path/to/erobs/src')

import rclpy
from bluesky_ros.simple_mtc_bluesky import MTCDevice
from bluesky_ros.task_builder import TaskBuilder

try:
    rclpy.init()
except:
    pass

mtc = MTCDevice("robot")
builder = TaskBuilder()
ROBOT_IP = '10.69.26.90'

print("✅ Robot loaded! Use: builder, mtc, ROBOT_IP")
```

---

## Quick Usage Once Loaded

```python
# See locations
builder.list_locations()

# Move
json_file = builder.move_to('pickup_approach')
RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))

# Pick sequence
json_file = builder.pick_sequence('pickup_approach', 'pickup', 'post_pickup_camera_safety')
RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))

# Place sequence
json_file = builder.place_sequence('place_approach', 'place', 'post_pickup_camera_safety')
RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))
```

---

## For Users on Beamline

**Tell them to run:**

```python
%run /path/to/your/erobs/src/bluesky_ros/load_robot.py
```

**Or give them the full path:**

```python
%run /nsls2/data/pdf/shared/erobs/src/bluesky_ros/load_robot.py
```

**Or create an alias in their profile:**

```python
# In their IPython profile startup:
def load_robot():
    exec(open('/full/path/to/load_robot.py').read())
```

Then they just type: `load_robot()`

---

## Setting Robot IP

```python
# Default is read from environment or 10.69.26.90
ROBOT_IP = '192.168.56.101'  # Change to their robot

# Then use normally
json_file = builder.move_to('home')
RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))
```

---

## Complete Example Session

```python
# 1. Start bsui (their normal way)
$ bsui

# 2. Load robot control
In [1]: %run ~/shared/erobs/src/bluesky_ros/load_robot.py
✅ Robot Control Loaded!

# 3. Change robot IP if needed
In [2]: ROBOT_IP = '192.168.56.101'

# 4. Use it!
In [3]: json_file = builder.pick_sequence('pickup_approach', 'pickup', 'post_pickup_camera_safety')

In [4]: RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))
```

---

## Make It Automatic (Optional)

**Create a function in their startup:**

Add to `~/.ipython/profile_collection/startup/99-robot.py`:

```python
import os

def load_robot(workspace='/path/to/erobs'):
    """Load robot control - usage: load_robot()"""
    exec(open(f'{workspace}/src/bluesky_ros/load_robot.py').read(), globals())

print("💡 Tip: Type 'load_robot()' to enable robot control")
```

Then users just type `load_robot()` when they need it.
