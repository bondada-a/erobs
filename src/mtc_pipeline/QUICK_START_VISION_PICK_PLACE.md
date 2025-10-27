# Quick Start: Vision Pick and Place with Predefined Place Position

## Overview
This guide shows how to pick an object marked with AprilTag ID 3 and place it at a predefined position using the vision pick and place functionality.

## Starting the System

### Step 1: Launch the complete system with all action servers

```bash
# Terminal 1: Launch the MoveIt configuration and robot drivers
ros2 launch ur_zivid_pipettor_moveit_config ur_zivid_pipettor_planning_execution.launch.py robot_ip:=192.168.56.101 use_fake_hardware:=false

# Terminal 2: Launch all action servers including vision_pick_place
ros2 launch mtc_pipeline modular_action_servers.launch.py robot_ip:=192.168.56.101
```

The `modular_action_servers.launch.py` will start:
- All MTC action servers (pick_place, vision, etc.)
- **vision_pick_place_action_server** (NEW)
- AprilTag detector
- MTC orchestrator

### Step 2: Ensure camera is running

```bash
# Terminal 3: Check if Zivid camera service is available
ros2 service list | grep capture_2d

# You should see:
/capture_2d
```

## Executing Vision Pick and Place

### Option 1: Using the Python client script (Recommended)

```bash
# Source the workspace
source /home/aditya/work/github_ws/erobs/install/setup.bash

# Pick from AprilTag 3, place at default position (x=0.4, y=0.3, z=0.15)
ros2 run mtc_pipeline vision_pick_predefined_place.py 3

# Pick from AprilTag 3, place at custom position (x=0.4, y=0.2, z=0.1)
ros2 run mtc_pipeline vision_pick_predefined_place.py 3 0.4,0.2,0.1

# Pick from AprilTag 5, custom place, use epick gripper
ros2 run mtc_pipeline vision_pick_predefined_place.py 5 0.3,0.3,0.15 epick
```

### Option 2: Using ROS 2 action command line

```bash
# Send action goal from command line
ros2 action send_goal /vision_pick_place_action mtc_pipeline/action/VisionPickPlaceAction "
{
  pick_tag_id: 3,
  place_tag_id: -1,
  gripper: 'hande',
  grasp_offset_json: '{\"x\": 0.0, \"y\": 0.0, \"z\": 0.05, \"rpy\": [0, 3.14159, 0]}',
  place_poses_json: '{\"place_position\": [0.4, 0.2, 0.1]}',
  approach_offset: 0.1,
  retreat_offset: 0.15
}"
```

### Option 3: Using a custom Python script

Create a file `test_vision_pick.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mtc_pipeline.action import VisionPickPlaceAction
import json


def main():
    rclpy.init()

    node = Node('test_vision_pick')
    client = ActionClient(node, VisionPickPlaceAction, 'vision_pick_place_action')

    # Wait for server
    node.get_logger().info('Waiting for action server...')
    client.wait_for_server()

    # Create goal
    goal = VisionPickPlaceAction.Goal()
    goal.pick_tag_id = 3  # Pick from AprilTag 3
    goal.place_tag_id = -1  # Use predefined position
    goal.gripper = 'hande'

    # Grasp offset: 5cm above tag, flipped down
    goal.grasp_offset_json = json.dumps({
        "x": 0.0, "y": 0.0, "z": 0.05,
        "rpy": [0, 3.14159, 0]
    })

    # Custom place position
    goal.place_poses_json = json.dumps({
        "place_position": [0.4, 0.25, 0.12]  # x, y, z in meters
    })

    goal.approach_offset = 0.1
    goal.retreat_offset = 0.15

    # Send goal
    node.get_logger().info('Sending goal...')
    future = client.send_goal_async(goal)

    # Wait for result
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()

    if goal_handle.accepted:
        node.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result().result

        if result.success:
            node.get_logger().info('Success!')
        else:
            node.get_logger().error(f'Failed: {result.error_message}')
    else:
        node.get_logger().error('Goal rejected!')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Parameters Explained

### Required Parameters
- `pick_tag_id`: AprilTag ID to pick from (e.g., 3)
- `place_tag_id`: Set to **-1** for predefined position, or tag ID for vision-based place
- `gripper`: Either "hande" or "epick"

### Grasp Offset Configuration
The `grasp_offset_json` defines how to grasp relative to the detected tag:
- `x`, `y`, `z`: Translation offset from tag center (meters)
- `rpy`: Rotation as [roll, pitch, yaw] (radians)

Example: `{"x": 0, "y": 0, "z": 0.05, "rpy": [0, 3.14159, 0]}`
- 5cm above the tag
- Rotated 180° around pitch (gripper pointing down)

### Place Position Configuration
When `place_tag_id = -1`, use `place_poses_json` to specify place position:
- `place_position`: [x, y, z] coordinates in base_link frame (meters)

Example: `{"place_position": [0.4, 0.2, 0.1]}`
- 40cm forward (X)
- 20cm left (Y)
- 10cm height (Z)

### Default Place Position
If no `place_poses_json` is provided, the default position is:
- X: 0.4m (40cm forward)
- Y: 0.3m (30cm left)
- Z: 0.15m (15cm height)

## Monitoring Execution

Watch the action server output:
```bash
# In the terminal where modular_action_servers.launch.py is running
[vision_pick_place_action_server]: Detecting pick tag 3...
[vision_pick_place_action_server]: Tag 3 detected at [0.352, 0.124, 0.051]
[vision_pick_place_action_server]: Using predefined place poses
[vision_pick_place_action_server]: Default place position: [0.400, 0.300, 0.150]
```

## Troubleshooting

### Issue: "Action server not available"
- Ensure `modular_action_servers.launch.py` is running
- Check that all nodes started successfully

### Issue: "Failed to detect tag"
- Verify AprilTag 3 is visible to the camera
- Check lighting conditions
- Ensure tag is clean and flat
- Test detection: `ros2 topic echo /detections`

### Issue: "Planning failed"
- Check if target positions are reachable
- Verify no collisions in the scene
- Try adjusting approach/retreat offsets

### Issue: "Gripper not responding"
- Ensure correct gripper type is specified
- Check gripper connections
- Verify gripper driver is running

## Complete Workflow Example

```bash
# Terminal 1: Start robot and MoveIt
ros2 launch ur_zivid_pipettor_moveit_config ur_zivid_pipettor_planning_execution.launch.py robot_ip:=192.168.56.101

# Terminal 2: Start all action servers
ros2 launch mtc_pipeline modular_action_servers.launch.py robot_ip:=192.168.56.101

# Terminal 3: Execute pick and place
source /home/aditya/work/github_ws/erobs/install/setup.bash

# Pick tag 3, place at x=0.35, y=0.25, z=0.12
ros2 run mtc_pipeline vision_pick_predefined_place.py 3 0.35,0.25,0.12
```

## Safety Notes

1. **Always test with reduced speed first**
2. **Ensure emergency stop is accessible**
3. **Verify workspace is clear of obstacles**
4. **Start with small movements**
5. **Monitor force/torque if available**

## Next Steps

- Adjust grasp offset for your specific objects
- Fine-tune place positions for your workspace
- Add multiple place locations
- Integrate with higher-level task planning