# MoveTo Module Usage Guide

The `moveto` module provides flexible robot movement capabilities within the MTC pipeline. It allows you to move the robot to specific poses using either joint space or Cartesian planning.

## Overview

The `moveto` module is designed to be a simple, flexible way to move the robot arm to desired positions. It supports three types of targets and two planning methods:

### Target Types
1. **Pose from JSON** - Use predefined poses from the JSON file
2. **Direct Joint Angles** - Specify joint angles directly
3. **Named States** - Use SRDF named states (like "home", "ready", etc.)
4. **Relative Movement** - Move a specified distance in a particular direction

### Planning Types
1. **Joint Space Planning** - Uses joint trajectory planning
2. **Cartesian Planning** - Uses Cartesian path planning

## JSON Configuration

### Basic Structure
```json
{
  "action": "moveto",
  "target_type": "pose|joints|named_state|relative",
  "target": "pose_name|joint_angles|named_state_name",
  "direction": "forward|right|up|backward|left|down|x|y|z|-x|-y|-z",
  "distance": 0.3,
  "planning_type": "joint|cartesian",
  "arm_group": "ur_arm"
}
```

### Parameters

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `action` | string | Yes | Must be "moveto" |
| `target_type` | string | Yes | "pose", "joints", "named_state", or "relative" |
| `target` | string/array | Yes* | Pose name, joint angles, or named state (*not needed for relative) |
| `direction` | string | Yes* | Direction for relative movement (*only for relative type) |
| `distance` | number | Yes* | Distance in meters for relative movement (*only for relative type) |
| `planning_type` | string | No | "joint" (default) or "cartesian" |
| `arm_group` | string | No | Robot arm group name (default: "ur_arm") |

## Usage Examples

### 1. Move to Predefined Pose (Joint Planning)
```json
{
  "action": "moveto",
  "target_type": "pose",
  "target": "pickup_approach",
  "planning_type": "joint",
  "arm_group": "ur_arm"
}
```

This will move the robot to the "pickup_approach" pose defined in the JSON poses section using joint space planning.

### 2. Move to Predefined Pose (Cartesian Planning)
```json
{
  "action": "moveto",
  "target_type": "pose",
  "target": "custom_pose_1",
  "planning_type": "cartesian",
  "arm_group": "ur_arm"
}
```

This will move the robot to the "custom_pose_1" pose using Cartesian path planning.

### 3. Move to Direct Joint Angles
```json
{
  "action": "moveto",
  "target_type": "joints",
  "target": [30.0, -60.0, 30.0, -60.0, 30.0, 0.0],
  "planning_type": "joint",
  "arm_group": "ur_arm"
}
```

This will move the robot directly to the specified joint angles (in degrees) using joint space planning.

### 4. Move to Named State
```json
{
  "action": "moveto",
  "target_type": "named_state",
  "target": "home",
  "planning_type": "joint",
  "arm_group": "ur_arm"
}
```

This will move the robot to the "home" position defined in the SRDF file using joint space planning.

### 5. Relative Movement
```json
{
  "action": "moveto",
  "target_type": "relative",
  "direction": "forward",
  "distance": 0.3,
  "planning_type": "cartesian",
  "arm_group": "ur_arm"
}
```

This will move the robot 0.3 meters forward from its current position using Cartesian planning.

## Complete Example JSON File

```json
{
  "start_gripper": "none",    
  "poses": {
    "pickup_approach": [14.76, -143.77, -62.81, -153.21, 69.87, 0.4],
    "pickup": [12.08, -135.05, -79.9, -144.83, 67.25, 0.41],
    "place_approach": [-28.2, -114.77, -60.94, -183.92, 26.37, -1.08],
    "place": [-43.22, -112.27, -64.52, -182.26, 11.36, -1.62],
    "custom_pose_1": [0.0, -90.0, 0.0, -90.0, 0.0, 0.0],
    "custom_pose_2": [45.0, -45.0, 45.0, -45.0, 45.0, 0.0]
  },
  "sequence": [
    {
      "action": "moveto",
      "target_type": "pose",
      "target": "pickup_approach",
      "planning_type": "joint"
    },
    {
      "action": "moveto",
      "target_type": "pose",
      "target": "custom_pose_1",
      "planning_type": "cartesian"
    },
    {
      "action": "moveto",
      "target_type": "joints",
      "target": [30.0, -60.0, 30.0, -60.0, 30.0, 0.0],
      "planning_type": "joint"
    },
    {
      "action": "moveto",
      "target_type": "named_state",
      "target": "home"
    },
    {
      "action": "moveto",
      "target_type": "relative",
      "direction": "forward",
      "distance": 0.3,
      "planning_type": "cartesian"
    },
    {
      "action": "moveto",
      "target_type": "relative",
      "direction": "right",
      "distance": 0.1,
      "planning_type": "cartesian"
    }
  ]
}

## Relative Movement Directions

When using `target_type: "relative"`, you can specify the direction using these keywords:

### Direction Keywords
| Keyword | Alternative | Description |
|---------|-------------|-------------|
| `forward` | `x` | Move in the positive X direction (forward) |
| `backward` | `-x` | Move in the negative X direction (backward) |
| `right` | `y` | Move in the positive Y direction (right) |
| `left` | `-y` | Move in the negative Y direction (left) |
| `up` | `z` | Move in the positive Z direction (up) |
| `down` | `-z` | Move in the negative Z direction (down) |

### Examples
```json
// Move 0.3 meters forward
{
  "action": "moveto",
  "target_type": "relative",
  "direction": "forward",
  "distance": 0.3,
  "planning_type": "cartesian"
}

// Move 0.1 meters to the right
{
  "action": "moveto",
  "target_type": "relative",
  "direction": "right",
  "distance": 0.1,
  "planning_type": "cartesian"
}

// Move 0.05 meters up
{
  "action": "moveto",
  "target_type": "relative",
  "direction": "up",
  "distance": 0.05,
  "planning_type": "cartesian"
}
```

## Planning Types Comparison

### Joint Space Planning (`planning_type: "joint"`)
- **Pros**: Faster planning, more predictable paths, better for large movements
- **Cons**: May not follow straight lines in Cartesian space
- **Best for**: Large movements, when exact Cartesian path is not critical

### Cartesian Planning (`planning_type: "cartesian"`)
- **Pros**: Follows straight lines in Cartesian space, better for precise positioning
- **Cons**: Slower planning, may fail if path is blocked
- **Best for**: Small movements, when straight-line motion is required

## Integration with Other Modules

The `moveto` module can be used alongside existing modules:

```json
{
  "sequence": [
    {
      "action": "moveto",
      "target_type": "pose",
      "target": "pickup_approach",
      "planning_type": "joint"
    },
    {
      "action": "tool_exchange",
      "operation": "load",
      "gripper": "epick",
      "dock_number": 3,
      "poses": ["load_approach"]
    },
    {
      "action": "moveto",
      "target_type": "pose",
      "target": "custom_pose_1",
      "planning_type": "cartesian"
    },
    {
      "action": "pick_and_place",
      "pick_poses": ["pickup_approach", "pickup"],
      "place_poses": ["place_approach", "place"],
      "gripper": "hande"
    }
  ]
}
```

## Error Handling

The module includes comprehensive error handling:
- Validates target types and parameters
- Checks for valid joint angles
- Ensures poses exist in JSON file
- Verifies named states exist in SRDF
- Provides detailed error messages for debugging

## Performance Considerations

- **Joint planning** is typically faster and more reliable
- **Cartesian planning** may take longer but provides more precise paths
- Consider using joint planning for large movements and Cartesian planning for fine positioning
- The module includes velocity and acceleration scaling (0.2) for safety

## Troubleshooting

### Common Issues

1. **"Pose not found"**: Ensure the pose name exists in the JSON poses section
2. **"Invalid joint angles"**: Check that joint angles are in degrees and match the robot's joint limits
3. **"Named state not found"**: Verify the named state exists in your SRDF file
4. **"Planning failed"**: Try switching between joint and Cartesian planning, or check for obstacles

### Debug Information

The module provides detailed logging:
- Current robot state before movement
- Planning success/failure messages
- Execution status
- Robot stability confirmation

## Running the Module

To use the moveto module, run the orchestrator with your JSON file:

```bash
ros2 launch mtc_pipeline orchestrator_launch.launch.py robot_ip:=192.168.56.101 use_fake_hardware:=true poses_file:=your_moveto_file.json
```
