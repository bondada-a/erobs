# MTC Pipeline

Modular motion planning framework for UR robotic systems using MoveIt Task Constructor (MTC). Provides specialized action servers for complex manipulation tasks with support for multiple end-effectors, vision-guided motion, and automated tool changing.

## Features

- **7 Specialized Action Servers**: MoveTo, PickPlace, EndEffector, ToolExchange, Vision, Pipettor, VisionPickPlace
- **Task Orchestration**: Multi-step task sequencing with automatic gripper switching
- **Vision Integration**: ArUco marker detection with Zivid 3D camera
- **Multi-Gripper Support**: Robotiq Hand-E, ePick vacuum, pipettor end-effectors
- **Dynamic MoveIt Management**: Automatic gripper configuration switching during tasks
- **Collision-Aware Planning**: YAML-based scene obstacle configuration

## Architecture

### Core Components

```
mtc_pipeline/
├── action_servers/          # ROS 2 action server nodes
│   ├── mtc_orchestrator_action_server.cpp
│   ├── move_to_action_server.cpp
│   ├── pick_place_action_server.cpp
│   ├── end_effector_action_server.cpp
│   ├── tool_exchange_action_server.cpp
│   ├── vision_action_server.cpp
│   ├── pipettor_action_server.cpp
│   └── vision_pick_place_action_server.cpp
├── stages/                  # MTC task implementations
│   ├── base_stages.cpp
│   ├── move_to_stages.cpp
│   ├── pick_place_stages.cpp
│   ├── end_effector_stages.cpp
│   ├── tool_exchange_stages.cpp
│   ├── vision_stages.cpp
│   ├── pipettor_stages.cpp
│   ├── vision_pick_place_stages.cpp
│   └── pipettor_operation_stage.cpp
├── core/                    # Infrastructure components
│   ├── moveit_lifecycle_manager.cpp
│   └── ur_tool_interface.cpp
├── utils/                   # Helper utilities
│   ├── gripper_config_registry.cpp
│   └── obstacle_loader.cpp
└── config/
    ├── grippers.yaml        # Gripper configurations
    ├── beamline_scene.yaml  # Collision scene objects
    └── vision_objects.json  # Vision object definitions
```

### Design Pattern

Each action server follows a two-layer architecture:

1. **Action Server Layer**: Handles ROS 2 action lifecycle, threading, and error reporting
2. **Stages Layer**: Builds and executes MTC task pipelines

```cpp
BaseActionServer<ActionType, StagesType>
    └─> StagesType : public BaseStages
            └─> bool run(const ActionType::Goal& goal)
```

### Orchestrator

The MTC Orchestrator coordinates multi-step tasks:

- **Gripper Registry**: Loads gripper configurations from YAML
- **Lifecycle Manager**: Spawns/kills MoveIt processes for different grippers
- **Tool Interface**: Manages UR tool voltage via URScript socket
- **Task Dispatcher**: Routes steps to appropriate action servers

## Installation

### Dependencies

```bash
# ROS 2 packages
sudo apt install ros-humble-moveit-task-constructor-core \
                 ros-humble-moveit-ros-planning-interface

# Workspace dependencies (if using vision/pipettor)
# - zivid_interfaces (for vision tasks)
# - pipette_driver (for pipetting operations)
```

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select mtc_pipeline
source install/setup.bash
```

## Configuration

### Gripper Configuration (`config/grippers.yaml`)

```yaml
grippers:
  hande:
    moveit_package: "ur_zivid_hande_moveit_config"
    tool_voltage: 24

  epick:
    moveit_package: "ur_zivid_epick_moveit_config"
    tool_voltage: 24

  pipettor:
    moveit_package: "ur_zivid_pipettor_moveit_config"
    tool_voltage: 12

  none:
    moveit_package: "ur_standalone_moveit_config"
    tool_voltage: 0
```

### Scene Obstacles (`config/beamline_scene.yaml`)

```yaml
obstacles:
  - name: "table"
    type: "box"
    dimensions: [2.0, 2.0, 0.05]
    position: [0.0, 0.0, -0.025]
    orientation: [0, 0, 0]  # RPY in radians
```

### Vision Objects (`config/vision_objects.json`)

```json
{
  "objects": [
    {
      "tag_id": 0,
      "name": "sample_holder",
      "shape": "cylinder",
      "dimensions": [0.05, 0.02],
      "tag_offset": [0.0, 0.0, 0.01]
    }
  ]
}
```

## Usage

### Launching the System

```bash
# Launch all action servers
ros2 launch mtc_pipeline mtc_bringup.launch.py

# With custom robot IP
ros2 launch mtc_pipeline mtc_bringup.launch.py robot_ip:=192.168.1.100
```

### Orchestrator Example

Execute multi-step tasks with automatic gripper switching:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mtc_pipeline.action import MTCExecution
import json

# Initialize ROS 2
rclpy.init()
node = Node('mtc_client')

# Define task sequence
task = {
    "start_gripper": "hande",
    "tasks": [
        {
            "task_type": "moveto",
            "target": "home",
            "planning_type": "joint"
        },
        {
            "task_type": "pick_and_place",
            "gripper": "hande",
            "pick_approach": "pick_approach",
            "pick_target": "pick_pose",
            "place_approach": "place_approach",
            "place_target": "place_pose"
        },
        {
            "task_type": "tool_exchange",
            "operation": "dock",
            "dock_number": 1,
            "approach_pose": "dock_approach"
        }
    ],
    "poses": {
        "home": [0, -90, 90, -90, -90, 0],
        "pick_approach": [30, -100, 95, -85, -90, 0],
        "pick_pose": [30, -110, 100, -80, -90, 0],
        "place_approach": [-30, -100, 95, -85, -90, 0],
        "place_pose": [-30, -110, 100, -80, -90, 0],
        "dock_approach": [45, -80, 70, -80, -90, 0]
    }
}

# Send goal
goal = MTCExecution.Goal()
goal.robot_ip = "192.168.1.2"
goal.full_json = json.dumps(task)

client = ActionClient(node, MTCExecution, 'mtc_execution')
client.wait_for_server()
future = client.send_goal_async(goal)
```

### Direct Action Server Usage

```python
from mtc_pipeline.action import MoveToAction

# Move to named pose (joint space)
goal = MoveToAction.Goal()
goal.target = "home"
goal.planning_type = "joint"
goal.poses_json = json.dumps({
    "home": [0, -90, 90, -90, -90, 0]
})

# Relative Cartesian movement
goal = MoveToAction.Goal()
goal.direction = "forward"  # forward, backward, up, down, left, right
goal.distance = 0.1
goal.planning_type = "cartesian"
```

## Action Servers Reference

### MoveTo Action

**Purpose**: Joint or Cartesian motion to named poses or relative movements

**Parameters**:
- `target` (string): Named pose key from poses_json
- `planning_type` (string): "joint" or "cartesian"
- `direction` (string): "forward", "backward", "up", "down", "left", "right"
- `distance` (float): Distance in meters for relative moves
- `poses_json` (string): JSON map of pose names to joint angles (degrees)

### PickPlace Action

**Purpose**: Pick and place sequence with approach/retreat motions

**Parameters**:
- `gripper` (string): Gripper type ("hande", "epick", "pipettor")
- `pick_approach` (string): Named pose for pick approach
- `pick_target` (string): Named pose for pick grasp
- `place_approach` (string): Named pose for place approach
- `place_target` (string): Named pose for place release
- `poses_json` (string): JSON map of poses

**Sequence**: approach → close gripper → retreat → approach → open gripper → retreat

### EndEffector Action

**Purpose**: Direct gripper open/close control

**Parameters**:
- `end_effector_type` (string): Gripper type
- `end_effector_action` (string): "open" or "closed"

### ToolExchange Action

**Purpose**: Automated tool changing at docking stations

**Parameters**:
- `operation` (string): "load" or "dock"
- `gripper` (string): Target gripper type (for load)
- `current_attached_gripper` (string): Currently attached gripper
- `dock_number` (int): Docking station identifier
- `approach_pose` (string): Named approach pose
- `poses_json` (string): JSON map of poses

### Vision Action

**Purpose**: Vision-guided motion to ArUco markers

**Parameters**:
- `tag_id` (int): ArUco marker ID
- `timeout` (float): Detection timeout in seconds (default: 10.0)

**Features**:
- Automatic gripper detection for z-offset
- TF frame broadcasting
- Collision object addition from vision_objects.json

### Pipettor Action

**Purpose**: Pipetting operations with volume control

**Parameters**:
- `operation` (string): "SUCK", "EXPEL", or "SET_LED"
- `volume_pct` (float): Volume percentage (0.0-100.0)
- `led_color` (ColorRGBA): LED color for SET_LED operation

### VisionPickPlace Action

**Purpose**: Combined vision detection and pick-place

**Parameters**:
- `tag_id` (int): ArUco marker ID for pick location
- `grasp_offset` (JSON object): XYZ offset from tag to grasp point
- `place_target` (string): Named pose for place location
- `place_approach` (string): Named pose for place approach
- `poses_json` (string): JSON map of poses

