# mtc_pipeline

This package provides modular action servers for task-based motion planning using
MoveIt Task Constructor (MTC) on UR robotic systems with various end-effectors:

- Seven specialized action servers for different manipulation primitives
- Task orchestrator for complex multi-step procedures
- Vision integration with ArUco marker detection
- Support for Robotiq Hand-E, ePick vacuum, and pipettor end-effectors
- Collision-aware motion planning with customizable scene objects

## Available Action Servers

### move_to_action_server

Joint-space or Cartesian motion to named poses or relative movements.
Supports both predefined poses and directional movements (forward/backward/up/down).

### pick_place_action_server

Pick and place operations with configurable approach/retreat motions.
Supports multiple gripper types with automatic configuration.

### end_effector_action_server

Direct gripper control for open/close operations.
Maps end-effector types to SRDF-defined states.

### tool_exchange_action_server

Automated tool changing between docking stations.
Operations: load tool, dock tool, with collision-aware movements.

### vision_action_server

Vision-guided motion to ArUco markers using Zivid camera.
Auto-detects gripper type and adjusts approach offsets.

### pipettor_action_server

Pipetting operations with volume control and LED feedback.
Integrates with pipette_driver for hardware control.

### vision_pick_place_action_server

Combined vision detection and pick-place execution.
Supports both vision-based and predefined place positions.

## Package Structure

### Action Definitions (`action/`)

- **MoveToAction.action**: Motion to poses or relative movements
- **PickPlaceAction.action**: Pick and place with approach/retreat
- **EndEffectorAction.action**: Gripper open/close commands
- **ToolExchangeAction.action**: Tool docking/loading operations
- **VisionMoveToAction.action**: Vision-guided motion to ArUco tags
- **PipettorAction.action**: Pipetting with volume control
- **VisionPickPlaceAction.action**: Vision-based pick and place
- **MTCExecution.action**: Orchestrator for multi-step tasks

### Stage Implementations (`src/` and `include/mtc_pipeline/`)

- **base_stages**: Base class with common MTC utilities
- **move_to_stages**: Joint/Cartesian motion implementation
- **pick_place_stages**: Pick-place sequence generation
- **end_effector_stages**: Gripper control stages
- **tool_exchange_stages**: Tool changing logic
- **vision_stages**: ArUco detection and TF handling
- **pipettor_stages**: Pipetting operation stages
- **vision_pick_place_stages**: Vision-based manipulation

### Configuration (`config/`)

- **vision_objects.json**: Collision object definitions for detected markers

### Launch Files (`launch/`)

- **mtc_bringup.launch.py**: Launches all action servers and orchestrator

## Dependencies

This package requires the following packages:

- **moveit_task_constructor**: Core MTC framework
- **ur_robot_driver**: UR robot control interface
- **zivid_interfaces**: Zivid camera service definitions (optional)
- **pipette_driver**: Pipetting hardware interface (optional)

External dependencies from workspace:
- **robotiq_hande_description**: Hand-E gripper descriptions
- **epick_config**: ePick vacuum gripper configurations

```xml
<depend>moveit_task_constructor_core</depend>
<depend>moveit_ros_planning_interface</depend>
<depend>rclcpp_action</depend>
<depend>zivid_interfaces</depend>
<depend>pipette_driver</depend>
```

## Action Server Architecture

### BaseActionServer Template

All action servers inherit from `BaseActionServer<ActionType, StagesType>`:
- Handles action server lifecycle and goal execution
- Delegates motion planning to specialized Stages classes
- Provides unified error handling and result reporting

### Stages Pattern

Each action type has a corresponding Stages class:
- Inherits from `BaseStages` for common utilities
- Implements `run()` method to build and execute MTC task
- Returns success/failure with error messages

### Orchestrator Pattern

The MTCExecution action server:
- Accepts JSON task definitions with multiple steps
- Routes each step to appropriate action server
- Manages task-level success/failure propagation

## Usage

### Building the Package

```bash
colcon build --packages-select mtc_pipeline
source install/setup.bash
```

### Launching Action Servers

```bash
# Launch all servers with default configuration
ros2 launch mtc_pipeline mtc_bringup.launch.py

# Launch with custom parameters
ros2 launch mtc_pipeline mtc_bringup.launch.py \
    robot_ip:=192.168.1.100 \
    kinematics_config:=$(ros2 pkg prefix ur5e_robot_description)/share/ur5e_robot_description/config/ur5e_calibration.yaml
```

### Executing Tasks via Orchestrator

Send a multi-step task as JSON to the orchestrator:

```python
from mtc_pipeline.action import MTCExecution
import json

# Define task with multiple steps
task = {
    "tasks": [
        {
            "type": "move_to",
            "target": "home"
        },
        {
            "type": "pick_place",
            "pick_approach": "pick_approach",
            "pick_target": "pick_position",
            "place_approach": "place_approach",
            "place_target": "place_position",
            "gripper": "hande"
        }
    ],
    "poses": {
        "home": [0, -90, 90, -90, -90, 0],
        "pick_approach": [30, -100, 95, -85, -90, 0],
        "pick_position": [30, -110, 100, -80, -90, 0],
        "place_approach": [-30, -100, 95, -85, -90, 0],
        "place_position": [-30, -110, 100, -80, -90, 0]
    }
}

goal = MTCExecution.Goal()
goal.task_json = json.dumps(task)
```

### Direct Action Server Usage

```python
from mtc_pipeline.action import MoveToAction

# Move to named pose
goal = MoveToAction.Goal()
goal.target = "home"
goal.planning_type = "joint"
goal.poses_json = json.dumps({"home": [0, -90, 90, -90, -90, 0]})

# Relative movement
goal = MoveToAction.Goal()
goal.direction = "forward"
goal.distance = 0.1
goal.planning_type = "cartesian"
```

## Configuration Notes

### Pose Definitions

Poses are defined as 6 joint angles in degrees:
- Format: `[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]`
- Example: `[0, -90, 90, -90, -90, 0]` represents home position

### Planner Configuration

Default planner parameters (defined in BaseStages):
- Pipeline planner: OMPL with 20% velocity/acceleration scaling
- Cartesian planner: 1mm step size, 60% path validity requirement
- Joint interpolation: 20% velocity/acceleration scaling

### Vision Configuration

ArUco marker detection settings:
- Default dictionary: `aruco4x4_50`
- Detection timeout: 10 seconds
- Automatic gripper detection for z-offset calculation

### Gripper Support

Supported gripper types with automatic SRDF state mapping:
- `"hande"`: Robotiq Hand-E gripper states
- `"epick"`: ePick vacuum gripper states
- `"pipettor"`: Pipetting end-effector states

## Integration with MoveIt

This package works with the following MoveIt configuration packages:
- `ur_standalone_moveit_config`
- `ur_zivid_hande_moveit_config`
- `ur_zivid_epick_moveit_config`
- `ur_zivid_pipettor_moveit_config`

## TODO

- Dynamic parameter loading from YAML configuration files
- Implement collision object management for vision-detected objects
- Add gripper payload configuration for grasp planning
- Complete VisionPickPlace integration with orchestrator
- Add service interfaces for synchronous execution