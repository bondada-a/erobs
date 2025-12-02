# TaskBuilder - Quick Start Guide

**What it does:** Build MTC tasks using your existing named locations instead of creating separate JSON files.

**Time to implement:** ✅ DONE! (15 minutes)
**Files changed:** 0 (pure addition)
**Lines of code:** ~200 lines

---

## 🎯 The Problem It Solves

**Before:**
```python
# Need to create pick_sample.json (50+ lines)
# Need to create place_sample.json (50+ lines)
# Need to create change_tool.json (30+ lines)
# ... 15 JSON files for different operations ...

yield from bps.abs_set(mtc, {
    'json_file': 'task_sequences/pick_sample.json',
    'robot_ip': '10.69.26.90'
})
```

**After:**
```python
# Use your existing 19 locations from triple_test_no_tasks.json
from bluesky_ros.task_builder import TaskBuilder

builder = TaskBuilder()  # Loads your locations automatically

# Build tasks on the fly with location names!
json_file = builder.pick_sequence('pickup_approach', 'pickup', 'post_pickup_camera_safety')

yield from bps.abs_set(mtc, {
    'json_file': json_file,
    'robot_ip': '10.69.26.90'
})
```

---

## 📚 Your Existing Locations (19 total)

From `task_sequences/triple_test_no_tasks.json`:

**Pick/Place:**
- `pickup_approach`, `pickup`, `post_pickup_camera_safety`
- `place_approach`, `place`
- `pre_pickup_approach`, `pre_pickup_orientation`

**Vacuum (EPick):**
- `vacuum_pickup_approach`, `vacuum_pickup`, `vacuum_post_pickup`
- `vacuum_place_approach`, `vacuum_place`

**Pipettor:**
- `pipettor_safety`, `pipettor_pickup_approach`, `pipettor_pre_pickup`, `pipettor_pickup`

**Tool Change:**
- `dock_approach`, `load_approach`

**Vision:**
- `vision_approach`

---

## 🚀 Quick Examples

### 1. Simple Move
```python
from bluesky_ros.task_builder import TaskBuilder

builder = TaskBuilder()

# Move to any named location
json_file = builder.move_to('pickup_approach')
yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
```

### 2. Pick Sequence
```python
# Pick using 3 location names
json_file = builder.pick_sequence(
    approach='pickup_approach',
    grasp='pickup',
    retreat='post_pickup_camera_safety',
    gripper='hande'
)
yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
```

### 3. Place Sequence
```python
json_file = builder.place_sequence(
    approach='place_approach',
    place='place',
    retreat='post_pickup_camera_safety'
)
yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
```

### 4. Tool Change
```python
# Automatic dock/load sequence
json_file = builder.tool_change('hande', 'epick')
yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
```

### 5. Custom Task Sequence
```python
# Build any custom sequence
tasks = [
    {"task_type": "moveto", "target": "vision_approach", "planning_type": "joint"},
    {"task_type": "moveto", "target": "pickup_approach", "planning_type": "joint"},
    {"task_type": "end_effector", "end_effector_type": "hande", "end_effector_action": "hande_closed"}
]

json_file = builder.build_task(tasks, gripper='hande')
yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
```

### 6. Multi-Sample Workflow (THE POWER!)
```python
# Process 10 samples with a loop!
for i in range(10):
    # Pick
    json_file = builder.pick_sequence('vacuum_pickup_approach', 'vacuum_pickup', 'vacuum_post_pickup', gripper='epick')
    yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})

    # Do measurement
    yield from xray_scan()

    # Return
    json_file = builder.place_sequence('vacuum_place_approach', 'vacuum_place', 'vacuum_post_pickup', gripper='epick')
    yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
```

---

## 🧪 Test It Right Now

```bash
# See all your locations
python3 src/bluesky_ros/task_builder.py

# Run interactive examples
python3 example_task_builder.py
```

---

## 📊 Comparison

| Feature | Old Way (JSON files) | New Way (TaskBuilder) |
|---------|---------------------|----------------------|
| **Pick sample** | Create 50-line JSON file | `builder.pick_sequence('approach', 'grasp', 'retreat')` |
| **10 samples** | 10 JSON files (500+ lines) | 3-line Python loop |
| **Change location** | Edit JSON file, save, test | Change location name in code |
| **Reuse poses** | Copy-paste between files | All tasks share same locations |
| **Add new sample** | Create new JSON file | Add one line to loop |

---

## 💡 Benefits

✅ **No more JSON file explosion** - Build tasks on the fly
✅ **Location reuse** - All tasks use same location registry
✅ **Easy loops** - Process multiple samples trivially
✅ **Quick iteration** - Change location name, not entire JSON
✅ **Backward compatible** - Old JSON files still work
✅ **Zero C++ changes** - Pure Python wrapper

---

## 🔧 Under the Hood

TaskBuilder:
1. Loads poses from `triple_test_no_tasks.json`
2. Provides helper methods (`pick_sequence`, `move_to`, etc.)
3. Builds complete task JSON internally
4. Saves to temp file
5. Returns path for use with `simple_mtc_bluesky.py`

**Nothing changes in mtc_pipeline C++ code!** It still receives the same JSON format.

---

## 📝 Next Steps

### Immediate (Use today):
1. Run examples: `python3 example_task_builder.py`
2. Write your own Bluesky plans using TaskBuilder
3. Enjoy shorter, cleaner code!

### Future (Phase 2):
- Add more convenience methods
- Support for vision-based picking
- Pose teaching interface
- Full ERobsDevice Ophyd class

---

## 🐛 Troubleshooting

**Q: "Location 'foo' not found"**
A: Check available locations with `builder.list_locations()`

**Q: "File not found: triple_test_no_tasks.json"**
A: Make sure you're running from workspace root (`~/work/github_ws/erobs`)

**Q: "Can I use my old JSON files?"**
A: Yes! Old approach still works. TaskBuilder is additive.

**Q: "Can I add new locations?"**
A: Yes! Edit `task_sequences/triple_test_no_tasks.json` and add to the "poses" section.

---

## 📚 API Reference

```python
class TaskBuilder:
    def __init__(locations_file='task_sequences/triple_test_no_tasks.json')
        """Load locations from JSON"""

    def list_locations()
        """Print all available location names"""

    def move_to(location, planning_type='joint', gripper=None)
        """Simple move to named location"""

    def pick_sequence(approach, grasp, retreat, gripper='hande')
        """Build pick: approach → grasp → close → retreat"""

    def place_sequence(approach, place, retreat, gripper='hande')
        """Build place: approach → place → open → retreat"""

    def tool_change(old_gripper, new_gripper)
        """Build dock old + load new sequence"""

    def build_task(tasks, gripper=None)
        """Build custom task from task list"""
```

---

**Total implementation time: 15 minutes**
**Immediate value: ✅ Use location names instead of files**
**Lines changed in C++: 0**
**Files created: 2 (task_builder.py, example_task_builder.py)**

🎉 **You now have a cleaner, more maintainable way to build robot tasks!**
