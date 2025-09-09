# MTC Pipeline - Current Modular Architecture

## Complete System Architecture Flow

```
┌─────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                    EXTERNAL CLIENTS                                            │
│                                                                                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐            │
│  │   GUI Clients   │  │   CLI Tools     │  │ Python Scripts  │  │  Other Nodes    │            │
│  │                 │  │                 │  │                 │  │                 │            │
│  │ • RViz2         │  │ • ros2 action   │  │ • Bluesky       │  │ • Custom Apps   │            │
│  │ • Web UIs       │  │   send_goal     │  │ • Ophyd         │  │ • Monitoring    │            │
│  │ • Dashboards    │  │ • Custom CLIs   │  │ • PDF Scripts   │  │ • Integration   │            │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  └─────────────────┘            │
│           │                     │                     │                     │                   │
│           └─────────────────────┼─────────────────────┼─────────────────────┘                   │
│                                 │                     │                                         │
│                                 ▼                     ▼                                         │
└─────────────────────────────────────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────────┐
│                              MAIN ORCHESTRATOR                                                 │
│                                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────────────────────────┐│
│  │                    mtc_orchestrator_action_server                                          ││
│  │                                                                                             ││
│  │  ┌─────────────────────────────────────────────────────────────────────────────────────┐   ││
│  │  │                    MTCExecution Action Server                                        │   ││
│  │  │                                                                                     │   ││
│  │  │  • Receives high-level task scripts (JSON)                                          │   ││
│  │  │  • Parses task steps (moveto, pick_place, tool_exchange, end_effector)             │   ││
│  │  │  • Orchestrates execution flow                                                      │   ││
│  │  │  • Manages gripper switching                                                         │   ││
│  │  │  • Provides feedback and abort capabilities                                         │   ││
│  │  └─────────────────────────────────────────────────────────────────────────────────────┘   ││
│  │                                     │                                                      ││
│  │                                     ▼                                                      ││
│  │  ┌─────────────────────────────────────────────────────────────────────────────────────┐   ││
│  │  │                        execute_step() Method                                        │   ││
│  │  │                                                                                     │   ││
│  │  │  if (action == "moveto") {                                                          │   ││
│  │  │      return call_moveto_action(step, poses);                                        │   ││
│  │  │  }                                                                                  │   ││
│  │  │  if (action == "pick_and_place") {                                                 │   ││
│  │  │      return call_pickplace_action(step, poses);                                     │   ││
│  │  │  }                                                                                  │   ││
│  │  │  if (action == "tool_exchange") {                                                  │   ││
│  │  │      return call_toolexchange_action(step, poses);                                  │   ││
│  │  │  }                                                                                  │   ││
│  │  │  if (action == "end_effector") {                                                   │   ││
│  │  │      return call_endeffector_action(step, poses);                                   │   ││
│  │  │  }                                                                                  │   ││
│  │  └─────────────────────────────────────────────────────────────────────────────────────┘   ││
│  │                                     │                                                      ││
│  │                                     ▼                                                      ││
│  │  ┌─────────────────────────────────────────────────────────────────────────────────────┐   ││
│  │  │                        Action Clients                                               │   ││
│  │  │                                                                                     │   ││
│  │  │  • moveto_action_client_                                                            │   ││
│  │  │  • pickplace_action_client_                                                         │   ││
│  │  │  • toolexchange_action_client_                                                      │   ││
│  │  │  • endeffector_action_client_                                                       │   ││
│  │  │                                                                                     │   ││
│  │  │  Each client sends ROS2 action goals to corresponding action servers               │   ││
│  │  └─────────────────────────────────────────────────────────────────────────────────────┘   ││
│  └─────────────────────────────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────────┐
│                            MODULAR ACTION SERVERS                                              │
│                                                                                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐            │
│  │ moveto_action   │  │ pickplace_action│  │ toolexchange_   │  │ endeffector_    │            │
│  │ _server         │  │ _server         │  │ action_server   │  │ action_server   │            │
│  │                 │  │                 │  │                 │  │                 │            │
│  │ • ROS2 Action   │  │ • ROS2 Action   │  │ • ROS2 Action   │  │ • ROS2 Action   │            │
│  │   Server        │  │   Server        │  │   Server        │  │   Server        │            │
│  │ • MoveToStages  │  │ • PickPlaceStages│  │ • ToolExchange  │  │ • EndEffector   │            │
│  │   Integration   │  │   Integration   │  │   Stages        │  │   Stages        │            │
│  │ • Full Feedback │  │ • Full Feedback │  │   Integration   │  │   Integration   │            │
│  │ • Abort Support │  │ • Abort Support │  │ • Full Feedback │  │ • Full Feedback │            │
│  │                 │  │                 │  │ • Abort Support │  │ • Abort Support │            │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  └─────────────────┘            │
└─────────────────────────────────────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────────┐
│                              MOVEIT TASK CONSTRUCTOR                                           │
│                                                                                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐            │
│  │   MoveToStages  │  │ PickPlaceStages │  │ ToolExchange    │  │ EndEffector     │            │
│  │                 │  │                 │  │ Stages          │  │ Stages          │            │
│  │ • Joint/Cartesian│  │ • Pick Sequence │  │ • Load/Dock     │  │ • Gripper Control│            │
│  │   Planning       │  │ • Place Sequence│  │   Operations    │  │ • Vacuum Control│            │
│  │ • Named States   │  │ • Approach/     │  │ • Relative Moves│  │ • Custom Actions│            │
│  │ • Relative Moves │  │   Retreat       │  │ • Dock Shifting │  │ • Force/Position│            │
│  │ • Constraints    │  │ • Wrist         │  │ • Tool Attach/  │  │   Control       │            │
│  │                 │  │   Constraints   │  │   Detach        │  │                 │            │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  └─────────────────┘            │
└─────────────────────────────────────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                MOVEIT FRAMEWORK                                                │
│                                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────────────────────────┐│
│  │                          move_group Node                                                   ││
│  │                                                                                             ││
│  │  • Robot Model Loading                                                                     ││
│  │  • Planning Scene Management                                                               ││
│  │  • OMPL Planning                                                                           ││
│  │  • Trajectory Execution                                                                    ││
│  │  • Collision Detection                                                                     ││
│  │  • Kinematics Solver                                                                       ││
│  └─────────────────────────────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────────┐
│                              ROBOT HARDWARE                                                   │
│                                                                                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐            │
│  │   UR5e Robot    │  │ HandE Gripper   │  │ Epick Gripper   │  │ Tool Docking    │            │
│  │                 │  │                 │  │                 │  │ Station         │            │
│  │ • 6-DOF Arm     │  │ • Finger Control│  │ • Vacuum Control│  │ • 5 Dock        │            │
│  │ • Joint Control │  │ • Force Control │  │ • Pressure      │  │   Positions     │            │
│  │ • TCP Control   │  │ • Position      │  │   Control       │  │ • Tool Exchange │            │
│  │ • Safety        │  │   Control       │  │ • Suction       │  │ • Alignment     │            │
│  │   Monitoring    │  │ • Feedback      │  │   Feedback      │  │ • Locking       │            │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  └─────────────────┘            │
└─────────────────────────────────────────────────────────────────────────────────────────────────┘
```

## Communication Flow

### 1. External Client → Main Orchestrator
```bash
# Example: GUI sends MTCExecution action
ros2 action send_goal /mtc_execution mtc_pipeline/action/MTCExecution '{
  "task_script": "moveto: home, pick_place: sample1, tool_exchange: dock",
  "poses": {"home": [0,0,0,0,0,0], "sample1": [1,2,3,4,5,6]}
}'
```

### 2. Main Orchestrator → Modular Action Servers
```cpp
// Orchestrator calls action clients
if (action == "moveto") {
    return call_moveto_action(step, poses);  // ROS2 Action Call
}
if (action == "pick_and_place") {
    return call_pickplace_action(step, poses);  // ROS2 Action Call
}
```

### 3. Action Servers → Stage Classes
```cpp
// Each action server wraps existing stage classes
bool success = moveto_stages_->run(step, poses, shared_from_this());
bool success = pick_place_stages_->run(step, poses, shared_from_this());
```

### 4. Stage Classes → MoveIt Task Constructor
```cpp
// Stage classes create MTC tasks
moveit::task_constructor::Task task;
task.add(std::make_unique<mtc::stages::MoveTo>(...));
task.add(std::make_unique<mtc::stages::MoveRelative>(...));
```

### 5. MoveIt → Robot Hardware
```cpp
// MTC executes on robot via move_group
auto result = task.execute(*task.solutions().front());
```

## Key Benefits

### ✅ **Modular Architecture**
- Each action server is independent and can be developed/tested separately
- Easy to add new action types without touching existing code
- Can run action servers on different machines for scalability

### ✅ **Full ROS2 Action Protocol**
- Complete feedback, abort, and monitoring capabilities at every level
- Standard ROS2 interfaces for easy integration
- Robust error handling and recovery

### ✅ **Reusable Components**
- Existing stage classes work without modification
- Action servers are thin wrappers around proven stage logic
- Easy to maintain and extend

### ✅ **Parameter Sharing**
- All nodes automatically share robot_description, robot_description_semantic
- OMPL planning parameters synchronized across all components
- No parameter synchronization issues

### ✅ **Launch Flexibility**
- `action_servers_only.launch.py` - Test action servers independently
- `modular_action_servers.launch.py` - Full system with MoveIt
- Easy to customize which components to launch

## Current Status

- ✅ **Modular Action Servers**: All 4 action servers implemented and running
- ✅ **Launch Files**: Created for both testing and full system
- ✅ **CMakeLists.txt**: Complete build configuration
- ✅ **Testing**: All action servers successfully compile and run
- 🔄 **Next Step**: Update orchestrator to use external action clients instead of embedded servers
