# TaskBuilder Command Reference (beambot)

Complete list of commands based on your actual workflows.

**Backend:** beambot (Python MTC implementation)
**Action server:** beambot_execution

> **Note:** Robot IP is now configured in beambot beamline config, not passed as argument.

---

## 📍 **Your 19 Available Locations**

```
pickup_approach, pickup, post_pickup_camera_safety
place_approach, place
pre_pickup_approach, pre_pickup_orientation
vacuum_pickup_approach, vacuum_pickup, vacuum_post_pickup
vacuum_place_approach, vacuum_place
pipettor_pickup_approach, pipettor_pre_pickup, pipettor_pickup, pipettor_safety
vision_approach
dock_approach, load_approach
```

---

## 🎯 **1. GRIPPER CONTROL (end_effector)**

### **HandE Gripper Open**
```python
tasks = [
    {"task_type": "end_effector",
     "end_effector_type": "hande",
     "end_effector_action": "hande_open"}
]
json_file = builder.build_task(tasks)
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

### **HandE Gripper Close**
```python
tasks = [
    {"task_type": "end_effector",
     "end_effector_type": "hande",
     "end_effector_action": "hande_closed"}
]
json_file = builder.build_task(tasks)
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

### **EPick Vacuum ON**
```python
tasks = [
    {"task_type": "end_effector",
     "end_effector_type": "epick",
     "end_effector_action": "vacuum_on"}
]
json_file = builder.build_task(tasks, gripper='epick')
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

### **EPick Vacuum OFF**
```python
tasks = [
    {"task_type": "end_effector",
     "end_effector_type": "epick",
     "end_effector_action": "vacuum_off"}
]
json_file = builder.build_task(tasks, gripper='epick')
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

---

## 🚀 **2. MOVEMENT (moveto)**

### **Move to Named Location (Joint Planning)**
```python
# Use the helper function (easiest)
json_file = builder.move_to('pickup_approach', planning_type='joint')
RE(bps.abs_set(mtc, {'json_file': json_file}))

# Or build manually
tasks = [
    {"task_type": "moveto",
     "target": "pickup_approach",
     "planning_type": "joint"}
]
json_file = builder.build_task(tasks)
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

### **Move to Named Location (Cartesian Planning)**
```python
json_file = builder.move_to('pickup', planning_type='cartesian')
RE(bps.abs_set(mtc, {'json_file': json_file}))

# Or manually
tasks = [
    {"task_type": "moveto",
     "target": "pickup",
     "planning_type": "cartesian"}
]
json_file = builder.build_task(tasks)
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

### **Relative Movement (Direction)**
```python
# Move down 5mm
tasks = [
    {"task_type": "moveto",
     "direction": "down",
     "distance": 0.005,
     "planning_type": "cartesian"}
]
json_file = builder.build_task(tasks)
RE(bps.abs_set(mtc, {'json_file': json_file}))

# Other directions: "up", "down", "forward", "backward", "left", "right"
```

---

## 🔧 **3. TOOL EXCHANGE**

### **Dock Current Gripper**
```python
tasks = [
    {"task_type": "tool_exchange",
     "operation": "dock",
     "gripper": "hande",
     "dock_number": 2,
     "approach_pose": "dock_approach"}
]
json_file = builder.build_task(tasks, gripper='hande')
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

### **Load New Gripper**
```python
tasks = [
    {"task_type": "tool_exchange",
     "operation": "load",
     "gripper": "epick",
     "dock_number": 3,
     "approach_pose": "load_approach"}
]
json_file = builder.build_task(tasks, gripper='hande')  # starting gripper
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

### **Complete Tool Change (Dock + Load) - EASIEST**
```python
# Use the helper function
json_file = builder.tool_change('hande', 'epick')
RE(bps.abs_set(mtc, {'json_file': json_file}))

# Or manually
tasks = [
    {"task_type": "tool_exchange", "operation": "dock",
     "gripper": "hande", "dock_number": 2, "approach_pose": "dock_approach"},
    {"task_type": "tool_exchange", "operation": "load",
     "gripper": "epick", "dock_number": 3, "approach_pose": "load_approach"}
]
json_file = builder.build_task(tasks, gripper='hande')
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

**Dock Numbers:**
- HandE: dock_number = 2
- EPick: dock_number = 3
- Pipettor: dock_number = 4

---

## 💉 **4. PIPETTOR OPERATIONS**

### **Pipettor Suck**
```python
tasks = [
    {"task_type": "pipettor",
     "operation": "SUCK",
     "volume_pct": 0.8}  # 80% volume
]
json_file = builder.build_task(tasks, gripper='pipettor')
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

### **Pipettor Eject Tip**
```python
tasks = [
    {"task_type": "pipettor",
     "operation": "EJECT_TIP",
     "volume_pct": 0.0}
]
json_file = builder.build_task(tasks, gripper='pipettor')
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

---

## 🎨 **5. COMPLETE WORKFLOWS (From triple_test.json)**

### **Complete Pick Sequence (HandE)**
```python
# Use helper function (easiest)
json_file = builder.pick_sequence(
    approach='pickup_approach',
    grasp='pickup',
    retreat='post_pickup_camera_safety',
    gripper='hande'
)
RE(bps.abs_set(mtc, {'json_file': json_file}))

# Or build manually (from lines 175-206 of triple_test.json)
tasks = [
    {"task_type": "end_effector", "end_effector_type": "hande",
     "end_effector_action": "hande_open"},
    {"task_type": "moveto", "target": "pre_pickup_orientation", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pickup_approach", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pickup", "planning_type": "cartesian"},
    {"task_type": "end_effector", "end_effector_type": "hande",
     "end_effector_action": "hande_closed"},
    {"task_type": "moveto", "direction": "down", "distance": 0.005, "planning_type": "cartesian"},
    {"task_type": "moveto", "target": "pickup_approach", "planning_type": "cartesian"},
    {"task_type": "moveto", "target": "post_pickup_camera_safety", "planning_type": "joint"}
]
json_file = builder.build_task(tasks, gripper='hande')
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

### **Complete Place Sequence (HandE)**
```python
# Use helper function
json_file = builder.place_sequence(
    approach='place_approach',
    place='place',
    retreat='post_pickup_camera_safety',
    gripper='hande'
)
RE(bps.abs_set(mtc, {'json_file': json_file}))

# Or manually (from lines 215-234 of triple_test.json)
tasks = [
    {"task_type": "moveto", "target": "pickup_approach", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pickup", "planning_type": "cartesian"},
    {"task_type": "end_effector", "end_effector_type": "hande",
     "end_effector_action": "hande_open"},
    {"task_type": "moveto", "target": "pickup_approach", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pre_pickup_orientation", "planning_type": "joint"}
]
json_file = builder.build_task(tasks, gripper='hande')
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

### **Vacuum Pick with EPick**
```python
# From lines 254-278 of triple_test.json
tasks = [
    {"task_type": "moveto", "target": "vacuum_pickup_approach", "planning_type": "joint"},
    {"task_type": "moveto", "target": "vacuum_pickup", "planning_type": "joint"},
    {"task_type": "end_effector", "end_effector_type": "epick",
     "end_effector_action": "vacuum_on"},
    {"task_type": "moveto", "target": "vacuum_pickup_approach", "planning_type": "joint"},
    {"task_type": "moveto", "target": "vacuum_post_pickup", "planning_type": "joint"}
]
json_file = builder.build_task(tasks, gripper='epick')
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

### **Vacuum Place with EPick**
```python
# From lines 279-297 of triple_test.json
tasks = [
    {"task_type": "moveto", "target": "vacuum_place_approach", "planning_type": "joint"},
    {"task_type": "moveto", "target": "vacuum_place", "planning_type": "cartesian"},
    {"task_type": "end_effector", "end_effector_type": "epick",
     "end_effector_action": "vacuum_off"},
    {"task_type": "moveto", "target": "vacuum_place_approach", "planning_type": "cartesian"}
]
json_file = builder.build_task(tasks, gripper='epick')
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

### **Complete Pipettor Workflow**
```python
# From lines 313-377 of triple_test.json
tasks = [
    {"task_type": "moveto", "target": "pipettor_safety", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pipettor_pickup_approach", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pipettor_pre_pickup", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pipettor_pickup", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pipettor_pre_pickup", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pipettor_pickup_approach", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pipettor_place_approach", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pipettor_place", "planning_type": "joint"},
    {"task_type": "pipettor", "operation": "SUCK", "volume_pct": 0.8},
    {"task_type": "moveto", "target": "pipettor_place_approach", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pipettor_pickup_approach", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pipettor_pre_pickup", "planning_type": "joint"},
    {"task_type": "pipettor", "operation": "EJECT_TIP", "volume_pct": 0.0}
]
json_file = builder.build_task(tasks, gripper='pipettor')
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

---

## 🔄 **6. COMPLETE TRIPLE TEST WORKFLOW**

Recreate the entire `triple_test.json` in one Python function:

```python
def triple_test_workflow():
    """Complete workflow: HandE pick → EPick vacuum → Pipettor"""

    # Part 1: HandE pick and place
    print("1. HandE pick and place...")
    json_file = builder.pick_sequence('pickup_approach', 'pickup', 'post_pickup_camera_safety', gripper='hande')
    yield from bps.abs_set(mtc, {'json_file': json_file})

    json_file = builder.place_sequence('pickup_approach', 'pickup', 'pre_pickup_orientation', gripper='hande')
    yield from bps.abs_set(mtc, {'json_file': json_file})

    # Part 2: Tool change HandE → EPick
    print("2. Changing to EPick...")
    json_file = builder.tool_change('hande', 'epick')
    yield from bps.abs_set(mtc, {'json_file': json_file})

    # Part 3: EPick vacuum pick and place
    print("3. EPick vacuum operations...")
    tasks = [
        {"task_type": "moveto", "target": "vacuum_pickup_approach", "planning_type": "joint"},
        {"task_type": "moveto", "target": "vacuum_pickup", "planning_type": "joint"},
        {"task_type": "end_effector", "end_effector_type": "epick", "end_effector_action": "vacuum_on"},
        {"task_type": "moveto", "target": "vacuum_pickup_approach", "planning_type": "joint"},
        {"task_type": "moveto", "target": "vacuum_post_pickup", "planning_type": "joint"},
        {"task_type": "moveto", "target": "vacuum_place_approach", "planning_type": "joint"},
        {"task_type": "moveto", "target": "vacuum_place", "planning_type": "cartesian"},
        {"task_type": "end_effector", "end_effector_type": "epick", "end_effector_action": "vacuum_off"},
        {"task_type": "moveto", "target": "vacuum_place_approach", "planning_type": "cartesian"}
    ]
    json_file = builder.build_task(tasks, gripper='epick')
    yield from bps.abs_set(mtc, {'json_file': json_file})

    # Part 4: Tool change EPick → Pipettor
    print("4. Changing to Pipettor...")
    json_file = builder.tool_change('epick', 'pipettor')
    yield from bps.abs_set(mtc, {'json_file': json_file})

    # Part 5: Pipettor operations
    print("5. Pipettor operations...")
    tasks = [
        {"task_type": "moveto", "target": "pipettor_safety", "planning_type": "joint"},
        {"task_type": "moveto", "target": "pipettor_pickup_approach", "planning_type": "joint"},
        {"task_type": "moveto", "target": "pipettor_pre_pickup", "planning_type": "joint"},
        {"task_type": "moveto", "target": "pipettor_pickup", "planning_type": "joint"},
        {"task_type": "moveto", "target": "pipettor_pre_pickup", "planning_type": "joint"},
        {"task_type": "moveto", "target": "pipettor_pickup_approach", "planning_type": "joint"},
        {"task_type": "moveto", "target": "pipettor_place_approach", "planning_type": "joint"},
        {"task_type": "moveto", "target": "pipettor_place", "planning_type": "joint"},
        {"task_type": "pipettor", "operation": "SUCK", "volume_pct": 0.8},
        {"task_type": "moveto", "target": "pipettor_place_approach", "planning_type": "joint"},
        {"task_type": "moveto", "target": "pipettor_pickup_approach", "planning_type": "joint"},
        {"task_type": "moveto", "target": "pipettor_pre_pickup", "planning_type": "joint"},
        {"task_type": "pipettor", "operation": "EJECT_TIP", "volume_pct": 0.0}
    ]
    json_file = builder.build_task(tasks, gripper='pipettor')
    yield from bps.abs_set(mtc, {'json_file': json_file})

    print("✅ Complete workflow finished!")

# Run it:
RE(triple_test_workflow())
```

---

## 📋 **Quick Command Summary**

```python
# Helper functions (EASIEST)
builder.move_to('location_name')
builder.pick_sequence('approach', 'grasp', 'retreat', gripper='hande')
builder.place_sequence('approach', 'place', 'retreat', gripper='hande')
builder.tool_change('old_gripper', 'new_gripper')

# Manual task building (MOST FLEXIBLE)
tasks = [...]  # List of task dictionaries
json_file = builder.build_task(tasks, gripper='starting_gripper')

# Execute
RE(bps.abs_set(mtc, {'json_file': json_file}))
```

---

## 🎯 **Task Type Reference**

| Task Type | Required Fields | Example |
|-----------|----------------|---------|
| `moveto` (named) | `target`, `planning_type` | `{"task_type": "moveto", "target": "pickup", "planning_type": "joint"}` |
| `moveto` (relative) | `direction`, `distance`, `planning_type` | `{"task_type": "moveto", "direction": "down", "distance": 0.005, "planning_type": "cartesian"}` |
| `end_effector` | `end_effector_type`, `end_effector_action` | `{"task_type": "end_effector", "end_effector_type": "hande", "end_effector_action": "hande_open"}` |
| `tool_exchange` | `operation`, `gripper`, `dock_number`, `approach_pose` | `{"task_type": "tool_exchange", "operation": "dock", "gripper": "hande", "dock_number": 2, "approach_pose": "dock_approach"}` |
| `pipettor` | `operation`, `volume_pct` | `{"task_type": "pipettor", "operation": "SUCK", "volume_pct": 0.8}` |
