# Hello Orchestrator

Minimal demonstration of the **orchestrator dispatch pattern** used in `beambot`.

## What This Demonstrates

This demo proves the core architectural pattern:
- **Multiple specialized action servers** (PrintServer, MoveServer)
- **Orchestrator with action clients** to each server
- **Task dispatcher** that routes based on task type
- **JSON-based task specification** for multi-step sequences

## Architecture

```
User → Orchestrator → Dispatch → Specialized Servers
       (routes)       (if/else)   (execute primitive)
```

### Components

**1. PrintServer** (`print_server`)
- Prints messages to console
- Shows simplest possible action server

**2. MoveServer** (`move_server`)
- Moves robot to named poses using MoveIt
- Shows realistic robot operation

**3. OrchestratorServer** (`orchestrator_server`)
- Accepts JSON task definitions
- Maintains action clients to PrintServer and MoveServer
- Dispatches each task to the appropriate server

**4. TaskClient** (`task_client.py`)
- Python client that sends multi-step tasks
- Shows how to use the orchestrator

## Requirements

- **UR ROS 2 packages** (robot driver + MoveIt config)
  ```bash
  sudo apt install ros-humble-ur-robot-driver ros-humble-ur-moveit-config
  ```
- Uses standard **"ur_arm"** planning group
- Uses standard **"home"** pose from UR SRDF

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select hello_orchestrator
source install/setup.bash
```

## Usage

### Terminal 1: Launch UR MoveIt with Fake Hardware

**Recommended (Combined):**
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true \
  use_fake_hardware:=true
```

**Alternative (Separate - more control):**
```bash
# Terminal 1a - Robot driver
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e robot_ip:=0.0.0.0 use_fake_hardware:=true \
  launch_rviz:=false initial_joint_controller:=joint_trajectory_controller

# Terminal 1b - MoveIt (after 1a starts)
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e launch_rviz:=true
```

**Note:** Replace `ur5e` with your UR model (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30)

### Terminal 2: Launch Demo Servers
```bash
source install/setup.bash
ros2 launch hello_orchestrator demo.launch.py
```

### Terminal 3: Run Client

**Option 1: Run with default demo task**
```bash
source install/setup.bash
ros2 run hello_orchestrator task_client.py
```

**Option 2: Run with custom task file**
```bash
source install/setup.bash
ros2 run hello_orchestrator task_client.py /path/to/your/task.json
```

## Task Definition (JSON format)

Tasks are defined in **JSON files** located in the `config/` directory.

### Default Task File

The default task is loaded from:
```
install/hello_orchestrator/share/hello_orchestrator/config/demo_task.json
```

### Task JSON Format

```json
{
  "tasks": [
    {"type": "print", "message": "Starting..."},
    {"type": "move", "target": "home"},
    {"type": "print", "message": "At home!"},
    {"type": "print", "message": "Complete!"}
  ]
}
```

### Creating Custom Tasks

**1. Copy the example:**
```bash
cp install/hello_orchestrator/share/hello_orchestrator/config/demo_task.json my_task.json
```

**2. Edit your task:**
```bash
nano my_task.json
```

**3. Run with your custom task:**
```bash
ros2 run hello_orchestrator task_client.py my_task.json
```

**Available Task Types:**
- `"print"` - Print a message (requires: `message`)
- `"move"` - Move to named pose (requires: `target`)

**Standard UR Named Poses:**
- `"home"` - Default home position (all joints at 0)
- `"up"` - Vertical configuration
- `"ready"` - Ready position (varies by UR model)

You can also use custom poses if defined in your SRDF.

## What You'll See

**Orchestrator dispatches to servers:**
```
📋 ORCHESTRATOR: Executing 4 tasks
  → Step 1/4: print
📝 MESSAGE: Demo starting...
  → Step 2/4: move
🤖 MOVING to: home
  → Step 3/4: print
📝 MESSAGE: Reached home position
  → Step 4/4: print
📝 MESSAGE: Demo complete!
✅ ORCHESTRATOR: All tasks completed successfully
```

**Note:** When using fake hardware (`use_fake_hardware:=true`), motion executes instantly. With a real robot, you'll need to accept the move on the teach pendant.

## Key Code: Orchestrator Dispatch

```cpp
// Core dispatch logic in orchestrator_server.cpp
for (auto& task : tasks) {
    std::string type = task["type"];

    if (type == "print") {
        success = execute_print_task(task);
    }
    else if (type == "move") {
        success = execute_move_task(task);
    }
}
```

**execute_print_task()** calls `print_client_->async_send_goal()`
**execute_move_task()** calls `move_client_->async_send_goal()`

## Pattern Mapping to beambot

| hello_orchestrator | beambot |
|-------------------|--------------|
| PrintServer | EndEffectorServer |
| MoveServer | MoveToServer |
| OrchestratorServer | MTCOrchestratorServer |
| 2 servers | 7 specialized servers |
| Simple dispatch | Dispatch + gripper switching |

## Extending the Demo

### Add a New Server

1. Define action in `hello_orchestrator_interfaces/action/`
2. Implement server in `src/new_server.cpp`
3. Add client in `orchestrator_server.cpp`:
   ```cpp
   new_client_ = rclcpp_action::create_client<NewAction>(this, "new_action");
   ```
4. Add dispatch case:
   ```cpp
   else if (type == "new") {
       success = execute_new_task(task);
   }
   ```

### Modify the Task

Edit `task_client.py`:
```python
task = {
    "tasks": [
        {"type": "print", "message": "Custom task"},
        {"type": "move", "target": "custom_pose"},
        # Add more steps...
    ]
}
```

## Understanding the Pattern

**Why this architecture?**
- ✅ Each server has single responsibility
- ✅ Orchestrator coordinates without implementation details
- ✅ Easy to add new operation types
- ✅ Tasks are declarative (what, not how)
- ✅ Servers can be developed independently

**Scaling to beambot:**
- More specialized servers (vision, gripper, tool exchange)
- MTC for complex motion planning
- Dynamic configuration switching
- Multi-gripper support

But the core pattern is the same: **orchestrator dispatches to specialized servers**.
