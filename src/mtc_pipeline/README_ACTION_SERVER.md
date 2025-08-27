# MTC Orchestrator Action Server

This document describes how to use the new MTC Orchestrator Action Server, which provides the same functionality as the original `mtc_orchestrator` but through a ROS2 action interface.

## Overview

The MTC Action Server accepts JSON task configurations and executes them using the same underlying MoveIt Task Constructor pipeline. The main advantages of the action server approach are:

- **Asynchronous execution**: Tasks run in the background while providing feedback
- **Cancellation support**: Tasks can be cancelled mid-execution
- **Progress monitoring**: Real-time feedback on task progress
- **Better integration**: Can be easily integrated with other ROS2 systems

## Action Interface

### Goal
- `task_script_json` (string): JSON string containing the task sequence and poses
- `robot_ip` (string): Robot IP address (optional, defaults to "192.168.1.101")
- `start_gripper` (string): Initial gripper configuration (optional, defaults to "none")

### Result
- `success` (bool): Whether the task completed successfully
- `error_message` (string): Error message if task failed
- `completed_steps` (int32): Number of steps completed
- `total_steps` (int32): Total number of steps in the task

### Feedback
- `current_step` (int32): Current step being executed
- `current_action` (string): Type of action being performed
- `progress_percentage` (float32): Overall progress (0-100%)
- `status_message` (string): Human-readable status message
- `current_gripper` (string): Current gripper configuration

## Usage

### 1. Launch the Action Server

```bash
# Launch the action server
ros2 launch mtc_pipeline mtc_action_server.launch.py robot_ip:=192.168.1.101
```

### 2. Send Goals Using the Example Client

```bash
# Build the package first
colcon build --packages-select mtc_pipeline

# Source the workspace
source install/setup.bash

# Run the example client
ros2 run mtc_pipeline mtc_action_client_example /path/to/your/script.json 192.168.1.101 none
```

### 3. Send Goals Programmatically

```python
import rclpy
from rclpy.action import ActionClient
from mtc_pipeline.action import MTCExecution
import json

# Initialize ROS2
rclpy.init()
node = rclpy.create_node('mtc_client')

# Create action client
client = ActionClient(node, MTCExecution, 'mtc_execution')

# Wait for server
client.wait_for_server()

# Read JSON file
with open('/path/to/script.json', 'r') as f:
    config = json.load(f)

# Create goal
goal = MTCExecution.Goal()
goal.task_script_json = json.dumps(config)
goal.robot_ip = "192.168.1.101"
goal.start_gripper = "none"

# Send goal
future = client.send_goal_async(goal)

# Wait for result
rclpy.spin_until_future_complete(node, future)
result = future.result()

if result.success:
    print("Task completed successfully!")
else:
    print(f"Task failed: {result.error_message}")
```

### 4. Monitor Progress

The action server provides real-time feedback during execution:

```bash
# Monitor feedback
ros2 topic echo /mtc_execution/_action/feedback
```

## JSON Configuration Format

The JSON configuration format remains the same as the original orchestrator:

```json
{
  "poses": {
    "pickup_approach": [0.0, -90.0, 0.0, -90.0, 0.0, 0.0],
    "pickup": [0.0, -90.0, 0.0, -90.0, 0.0, 0.0],
    "place_approach": [90.0, -90.0, 0.0, -90.0, 0.0, 0.0],
    "place": [90.0, -90.0, 0.0, -90.0, 0.0, 0.0]
  },
  "sequence": [
    {
      "action": "pick_and_place",
      "pick_poses": ["pickup_approach", "pickup"],
      "place_poses": ["place_approach", "place"],
      "gripper": "hande"
    }
  ]
}
```

## Supported Actions

The action server supports the same actions as the original orchestrator:

- `pick_and_place`: Pick and place operations
- `tool_exchange`: Tool/gripper exchange operations
- `moveto`: Simple movement to poses
- `end_effector`: End effector operations (vacuum, etc.)

## Error Handling

The action server provides detailed error information:

- **Planning failures**: When MoveIt cannot plan a path
- **Execution failures**: When robot execution fails
- **Configuration errors**: When JSON is invalid or missing required fields
- **System errors**: When MoveIt or robot services are unavailable

## Cancellation

Tasks can be cancelled at any time:

```python
# Cancel the current goal
future = client.cancel_goal_async(goal_handle)
rclpy.spin_until_future_complete(node, future)
```

## Building

```bash
# Build the package
colcon build --packages-select mtc_pipeline

# Source the workspace
source install/setup.bash
```

## Troubleshooting

### Common Issues

1. **Action server not found**: Make sure the action server is running
2. **JSON parsing errors**: Verify your JSON format is correct
3. **MoveIt not ready**: Check that MoveIt services are available
4. **Robot connection issues**: Verify robot IP and network connectivity

### Debug Information

Enable debug logging:

```bash
ros2 run mtc_pipeline mtc_orchestrator_action_server --ros-args --log-level debug
```

## Differences from Original Orchestrator

| Feature | Original | Action Server |
|---------|----------|---------------|
| Execution | Synchronous | Asynchronous |
| Feedback | None | Real-time |
| Cancellation | None | Supported |
| Integration | Standalone | ROS2 integrated |
| Error handling | Exit on error | Detailed error reporting |
| Process management | Same | Same |

The action server maintains all the core functionality of the original orchestrator while providing better integration capabilities and user experience.
