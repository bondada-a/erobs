# How to Use TaskBuilder - SIMPLE VERSION

## Step 1: See What Locations You Have

```bash
cd ~/work/github_ws/erobs
python3 src/bluesky_ros/task_builder.py
```

**Output:** You'll see your 19 locations like:
- `pickup_approach`
- `pickup`
- `place_approach`
- `vacuum_pickup_approach`
- etc.

---

## Step 2: Copy the Simple Example

```bash
cp simple_example.py my_experiment.py
```

---

## Step 3: Edit Your Experiment

Open `my_experiment.py` and change the `my_experiment()` function:

### Example: Just Move Somewhere

```python
def my_experiment():
    # Move to pickup position
    json_file = builder.move_to('pickup_approach')
    yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
```

### Example: Pick Something

```python
def my_experiment():
    # Pick using 3 location names
    json_file = builder.pick_sequence(
        approach='pickup_approach',
        grasp='pickup',
        retreat='post_pickup_camera_safety'
    )
    yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
```

### Example: Pick Then Place

```python
def my_experiment():
    # Pick
    json_file = builder.pick_sequence('pickup_approach', 'pickup', 'post_pickup_camera_safety')
    yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})

    # Place
    json_file = builder.place_sequence('place_approach', 'place', 'post_pickup_camera_safety')
    yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
```

### Example: Do It 5 Times

```python
def my_experiment():
    for i in range(5):
        print(f"Sample {i+1}/5...")

        # Pick
        json_file = builder.pick_sequence('pickup_approach', 'pickup', 'post_pickup_camera_safety')
        yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})

        # Place
        json_file = builder.place_sequence('place_approach', 'place', 'post_pickup_camera_safety')
        yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
```

---

## Step 4: Run It

```bash
# Terminal 1: Start robot (if not running)
ros2 launch mtc_pipeline mtc_bringup.launch.py robot_ip:=YOUR_IP

# Terminal 2: Run your experiment
python3 my_experiment.py
```

---

## That's It!

## Available Commands

```python
# Move to a location
json_file = builder.move_to('location_name')

# Pick (3 locations: approach, grasp, retreat)
json_file = builder.pick_sequence('approach', 'grasp', 'retreat')

# Place (3 locations: approach, place, retreat)
json_file = builder.place_sequence('approach', 'place', 'retreat')

# Change gripper
json_file = builder.tool_change('hande', 'epick')
```

Then always do:
```python
yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
```

---

## Your 19 Available Locations

```
pickup_approach
pickup
post_pickup_camera_safety
place_approach
place
vacuum_pickup_approach
vacuum_pickup
vacuum_post_pickup
vacuum_place_approach
vacuum_place
pipettor_pickup_approach
pipettor_pickup
pipettor_pre_pickup
pipettor_safety
pre_pickup_approach
pre_pickup_orientation
vision_approach
dock_approach
load_approach
```

---

## Common Mistakes

❌ **Wrong:**
```python
yield from bps.abs_set(mtc, builder.pick_sequence(...))  # Missing robot_ip!
```

✅ **Right:**
```python
json_file = builder.pick_sequence(...)
yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
```

---

## Need Help?

**See available locations:**
```bash
python3 src/bluesky_ros/task_builder.py
```

**Copy working example:**
```bash
cp simple_example.py my_test.py
```
