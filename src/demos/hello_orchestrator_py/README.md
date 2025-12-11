# hello_orchestrator_py

🎓 **Educational Demo**: Learn MTC (MoveIt Task Constructor) orchestrator patterns used in EROBS production systems.

## Purpose

**Onboarding tool** for new engineers joining the EROBS team. Demonstrates the foundational architecture used throughout EROBS robotic systems:
- **hello_orchestrator_py** (this demo) - Learn orchestrator dispatch and MTC basics
- **mtc_py** (production) - Full orchestrator with 7 action types (gripper, vision, tool exchange, etc.)
- **pdf_beamtime** (production) - FSM with Bluesky integration for beamline experiments

## What It Demonstrates

```
JSON Task → Orchestrator → Specialized Action Servers → MTC Execution
```

- ✅ **Orchestrator dispatch pattern** - Route tasks to specialized servers
- ✅ **MTC stage construction** - Build motion plans using MoveIt Task Constructor
- ✅ **Custom stages** - Create non-motion actions (print server example)
- ✅ **Action server architecture** - Standard pattern for all EROBS systems
- ✅ **Sequential task execution** - Execute multi-step workflows

## Prerequisites

### MoveIt with ExecuteTaskSolutionCapability

This demo uses **MoveIt Task Constructor (MTC)** which requires the `ExecuteTaskSolutionCapability` to execute planned trajectories.

⚠️ **Standard ur_moveit_config does NOT include this capability by default.**

### Two Options

**Option A: EROBS Custom Configs (Recommended)**
```bash
# Already has ExecuteTaskSolutionCapability configured
ros2 launch ur_standalone_moveit_config robot_bringup.launch.py \
  use_fake_hardware:=true
```
✅ Pre-configured with ExecuteTaskSolutionCapability
✅ Planning group: `ur_arm`
✅ Integrated hardware (Zivid camera, grippers)

**Option B: Standard UR Config (DIY)**

If using standard `ur_moveit_config`:
1. Add ExecuteTaskSolutionCapability to move_group launch
2. Change demo to use `ur_manipulator` planning group (see [Customization](#customization))

See [MoveIt Capabilities Documentation](https://moveit.picknik.ai/main/doc/concepts/move_group.html) for adding capabilities.

---

## Quick Start

### 1. Build

```bash
cd ~/work/github_ws/experimental

colcon build --packages-select \
  ur_standalone_moveit_config \
  hello_orchestrator_py

source install/setup.bash
```

### 2. Launch MoveIt (Terminal 1)

```bash
source install/setup.bash

ros2 launch ur_standalone_moveit_config robot_bringup.launch.py \
  use_fake_hardware:=true
```

**Wait for**: `"You can start planning now!"`

**Verify ExecuteTaskSolutionCapability**:
```bash
ros2 action list | grep execute_task_solution
# Should output: /execute_task_solution
```

### 3. Launch Demo Servers (Terminal 2)

```bash
source install/setup.bash

ros2 launch hello_orchestrator_py demo.launch.py
```

**You should see**:
```
[INFO] [print_server_py]: print_server_py started on 'print_message_py'
[INFO] [move_server_py]: move_server_py started on 'move_to_named_py'
[INFO] [orchestrator_server_py]: Orchestrator action server ready
```

### 4. Send Test Task (Terminal 3)

```bash
source install/setup.bash

ros2 run hello_orchestrator_py task_client.py \
  src/demos/hello_orchestrator_py/config/demo_task.json
```

**Expected output**:
```
[INFO] Sending goal with 4 tasks
[INFO] Goal accepted
[INFO] Feedback: Step 1/4 - print
[INFO] Feedback: Step 2/4 - move
[INFO] Feedback: Step 3/4 - print
[INFO] Feedback: Step 4/4 - print
[INFO] Goal succeeded!
```

**In RViz**: Robot moves to `moveit_home` position (all joints at [0°, -90°, 0°, 0°, 0°, 0°])

---

## Architecture Deep Dive

### Orchestrator Dispatch Pattern

```python
# demo_task.json
{
  "tasks": [
    {"type": "print", "message": "Hello"},
    {"type": "move", "target": "moveit_home"}
  ]
}
```

**Execution flow**:
```
1. task_client.py sends JSON → orchestrator_server.py
2. Orchestrator reads task list
3. Task 1: type="print" → dispatch to print_server
   - print_server executes custom logic
   - Returns success
4. Task 2: type="move" → dispatch to move_server
   - move_server creates MTC task
   - Plans using OMPL
   - Executes using execute_task_solution
   - Returns success
5. Orchestrator reports completion
```

### Action Servers

| Server | Purpose | Stage Type |
|--------|---------|------------|
| **print_server** | Demonstrates custom non-motion actions | Custom Python logic |
| **move_server** | Demonstrates MTC motion planning | MTC MoveTo stage |
| **orchestrator_server** | Dispatches tasks to specialized servers | Coordination |

### MTC Stage Construction

From `move_server.py`:
```python
# Create MTC task
task = self.create_task_template("MoveTo Task")

# Add MoveTo stage
planner = self.make_pipeline_planner()  # OMPL
move_stage = stages.MoveTo("move_to_home", planner)
move_stage.group = "ur_arm"  # Planning group
move_stage.setGoal("moveit_home")  # Named state from SRDF
task.add(move_stage)

# Plan and execute
self.load_plan_execute(task)
```

---

## Configuration

### Named States (SRDF)

⚠️ **Important**: The demo uses **SRDF named states** for motion targets. If using custom MoveIt configs, verify available named states match your task.

**Current available states** (in `ur_standalone_moveit_config/config/ur.srdf`):
- `moveit_home` - All joints at [0°, -90°, 0°, 0°, 0°, 0°]

#### Option 1: Use Existing States

Update `config/demo_task.json` to use available named states:
```json
{
  "tasks": [
    {"type": "move", "target": "moveit_home"}
  ]
}
```

#### Option 2: Add Custom Named States

Edit `ur_standalone_moveit_config/config/ur.srdf`:
```xml
<group_state name="custom_pose" group="ur_arm">
  <joint name="shoulder_pan_joint" value="1.57"/>
  <joint name="shoulder_lift_joint" value="-1.57"/>
  <joint name="elbow_joint" value="1.57"/>
  <joint name="wrist_1_joint" value="-1.57"/>
  <joint name="wrist_2_joint" value="-1.57"/>
  <joint name="wrist_3_joint" value="0"/>
</group_state>
```

Then use in task:
```json
{"type": "move", "target": "custom_pose"}
```

### Planning Group

**EROBS custom configs use**: `ur_arm`
**Standard UR configs use**: `ur_manipulator`

If using standard configs, update:
- `hello_orchestrator_py/stages/base_stages.py`: Change `self.arm_group = "ur_arm"` → `"ur_manipulator"`
- `launch/demo.launch.py`: Change `'ur_arm'` → `'ur_manipulator'` in kinematics config

---

## Learning Path

### Level 1: Understand This Demo (You Are Here)

**Study these files** (in order):

1. **`config/demo_task.json`** - See task format
   - JSON structure: `{"tasks": [...]}`
   - Task types: `"print"`, `"move"`
   - Parameters per task type

2. **`scripts/orchestrator_server.py`** - See dispatch logic
   - How orchestrator receives JSON goal
   - How it routes tasks to specialized servers
   - How it tracks progress and reports feedback

3. **`hello_orchestrator_py/stages/move_stages.py`** - See MTC MoveTo
   - How to create MTC task template
   - How to construct MoveTo stage
   - How to plan and execute

4. **`hello_orchestrator_py/stages/print_stages.py`** - See custom stage
   - How to create non-motion actions
   - Simple stage pattern for custom logic

**Key takeaways**:
- ✅ Orchestrator dispatches to specialized servers
- ✅ Each server handles one task type
- ✅ MTC stages build motion plans
- ✅ Execute via `execute_task_solution` action

### Level 2: Production Orchestrator (mtc_py)

After mastering this demo, move to **`mtc_py/`**:

**What's added**:
- 7 action types (vs 2 in demo):
  - `move_to` - Joint/Cartesian motion
  - `pick_place` - Grasp and place operations
  - `end_effector` - Gripper control
  - `vision` - Camera-based positioning
  - `tool_exchange` - Automatic gripper swapping
  - `pipettor` - Liquid handling
  - `vision_pick_place` - Combined vision + grasp
- Gripper integration (Hand-E, ePick, pipettor)
- Vision integration (Zivid 3D camera, ArUco markers)
- Tool exchange (automatic MoveIt restart on gripper swap)

**Same patterns**:
- ✅ Orchestrator dispatch (identical structure)
- ✅ MTC stage construction (same methods)
- ✅ Action server architecture (same base class)

### Level 3: Beamline FSM (pdf_beamtime)

Production FSM with Bluesky integration:

**What's added**:
- 11-state finite state machine
- Pause/resume/abort capabilities
- Bluesky RunEngine integration
- Grasp success/failure handling
- Planning scene obstacle management

---

## Customization Exercises

### Exercise 1: Add a Delay Action

Create a new action type that waits for N seconds:

1. **Define action**: `action/Delay.action`
   ```
   float32 seconds
   ---
   bool success
   ---
   float32 time_remaining
   ```

2. **Create server**: `scripts/delay_server.py`
   ```python
   class DelayServer(BaseActionServer):
       def execute_goal(self, goal):
           time.sleep(goal.seconds)
           return True
   ```

3. **Update orchestrator**: Add delay handler in `orchestrator_server.py`

4. **Test**: Add `{"type": "delay", "seconds": 3.0}` to demo_task.json

### Exercise 2: Add More Named Poses

Edit SRDF to add useful poses:
- `scan_position` - Position for camera scanning
- `pickup_approach` - Pre-grasp approach
- `safe_retract` - Safe position above workspace

### Exercise 3: Cartesian Motion (Advanced)

Modify move_server to support Cartesian paths:
- Study `mtc_py/mtc_py_lib/stages/base_stages.py` → `create_relative_move_stage()`
- Add Cartesian task type to demo
- Test linear motion vs joint space motion

---

## Troubleshooting

### "Planning group 'ur_arm' not found"

**Cause**: Using standard ur_moveit_config (uses `ur_manipulator`)

**Fix**:
- Use EROBS custom config: `ur_standalone_moveit_config`
- OR change demo to use `ur_manipulator` (see [Planning Group](#planning-group))

### "Failed to connect to execute_task_solution"

**Cause**: MoveIt doesn't have ExecuteTaskSolutionCapability loaded

**Fix**:
```bash
# Verify capability:
ros2 action list | grep execute_task_solution

# If missing, relaunch with custom config:
ros2 launch ur_standalone_moveit_config robot_bringup.launch.py use_fake_hardware:=true
```

### "Unknown joint pose: home" or "Unknown joint pose: moveit_home"

**Cause**: Named state doesn't exist in SRDF

**Fix**:
1. Check available states in SRDF:
   ```bash
   grep "group_state name" src/custom-ur-descriptions/ur5e_moveit_configs/ur_standalone_moveit_config/config/ur.srdf
   ```

2. Update `demo_task.json` to use existing state, OR

3. Add custom state to SRDF (see [Named States](#named-states-srdf))

### "Module not found: hello_orchestrator_py"

**Cause**: Package not built or sourced

**Fix**:
```bash
colcon build --packages-select hello_orchestrator_py
source install/setup.bash
```

### "Planning succeeded, but robot doesn't move"

**Cause**: Execution might be commented out

**Fix**: Check `hello_orchestrator_py/stages/base_stages.py` line ~198:
```python
# Should see:
self.logger.info("Executing...")
result = task.execute(task.solutions[0])

# NOT:
# self.logger.warn("SKELETON DEMO: Execution skipped...")
```

If commented, uncomment and rebuild.

### Warning: "Publisher already registered"

**Status**: Known warning, doesn't affect functionality

**Cause**: Multiple rclcpp.Node instances created (MTC requirement)

**Impact**: None - warning only, can be ignored

---

## Comparison: hello_orchestrator_py vs mtc_py

| Aspect | hello_orchestrator_py | mtc_py |
|--------|----------------------|--------|
| **Purpose** | Educational demo | Production system |
| **Action types** | 2 (print, move) | 7 (move, pick_place, gripper, vision, etc.) |
| **Hardware** | None required | Grippers, camera, tool exchanger |
| **Complexity** | ~800 lines | ~2500+ lines |
| **Batching** | Sequential (one task at a time) | Planned (future optimization) |
| **MTC stages** | MoveTo only | MoveTo, Pick, Place, Cartesian, Custom |
| **Execution** | execute_task_solution | execute_task_solution |
| **Orchestrator** | JSON dispatch | JSON dispatch |
| **Learning curve** | ⭐ Easy | ⭐⭐⭐ Advanced |

**Key insight**: Same architecture, different scale. Master this demo → understand mtc_py structure.

---

## File Structure

```
hello_orchestrator_py/
├── action/                          # ROS2 action definitions
│   ├── PrintMessage.action         # Print task interface
│   ├── MoveToNamed.action          # Motion task interface
│   └── OrchestratorTask.action     # Orchestrator interface
│
├── hello_orchestrator_py/      # Python modules
│   └── stages/
│       ├── base_stages.py          # MTC utilities (create_task_template, planners)
│       ├── print_stages.py         # Custom print stage
│       └── move_stages.py          # MTC MoveTo stage
│
├── scripts/                         # Executable servers
│   ├── print_server.py             # Print action server
│   ├── move_server.py              # Motion action server
│   ├── orchestrator_server.py      # Orchestrator action server
│   └── task_client.py              # Test client
│
├── launch/
│   └── demo.launch.py              # Launch all servers
│
├── config/
│   └── demo_task.json              # Example task definition
│
├── CMakeLists.txt                  # Build configuration
├── package.xml                     # ROS2 package manifest
└── README.md                       # This file
```

---

## Further Reading

### EROBS-Specific
- `../../CLAUDE.md` - Workspace overview and architecture
- `../../mtc_py/README.md` - Production orchestrator documentation
- `../../custom-ur-descriptions/` - Custom MoveIt configurations

### External Resources
- [MoveIt Task Constructor Tutorial](https://moveit.picknik.ai/humble/doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial.html)
- [ROS2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [UR ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [MoveIt2 Concepts](https://moveit.picknik.ai/humble/doc/concepts/concepts.html)

---

## Summary

**hello_orchestrator_py teaches**:
- ✅ How to structure an MTC-based orchestrator
- ✅ How to dispatch tasks to specialized servers
- ✅ How to construct and execute MTC stages
- ✅ How to create custom non-motion actions
- ✅ The foundation for understanding mtc_py and pdf_beamtime

**What it deliberately omits** (appropriately for education):
- Gripper integration (hardware-specific)
- Vision integration (complex setup)
- Tool exchange (advanced topic)
- Batching optimization (future enhancement)
- FSM state management (production feature)

**Next steps**: Master this demo, then explore mtc_py for production capabilities.

---

## Questions?

- **EROBS Team**: Ask your mentor or check Slack #erobs-dev
- **Issues**: Open issue in repository or discuss in team meetings

---

**Version**: 1.0.0
**Last Updated**: 2024-12-10
**Maintainer**: EROBS Team
