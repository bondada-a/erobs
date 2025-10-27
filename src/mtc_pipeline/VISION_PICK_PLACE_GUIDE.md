# Vision-Based Pick and Place Guide

## Overview

The Vision Pick Place functionality enables the robot to perform pick and place operations using AprilTag detection for object localization. Instead of using predefined joint positions, the system detects AprilTags and computes grasp poses dynamically.

## Architecture

### Components

1. **VisionPickPlaceAction.action**: Action interface for vision-based pick and place
2. **VisionPickPlaceStages**: MTC stages implementation for vision-guided manipulation
3. **vision_pick_place_action_server**: ROS 2 action server node

### Key Features

- **AprilTag Detection**: Uses Zivid camera for tag detection
- **Dynamic Pose Computation**: Calculates grasp poses from tag poses with configurable offsets
- **Cartesian Path Planning**: Uses straight-line paths for approach and retreat
- **Wrist Constraint**: Maintains orientation to prevent object tilting
- **Detection Caching**: Reuses recent detections for efficiency

## Usage

### Starting the Action Server

```bash
ros2 run mtc_pipeline vision_pick_place_action_server
```

### Action Interface

#### Goal Parameters

- `pick_tag_id` (int32): AprilTag ID to detect for picking
- `place_tag_id` (int32): AprilTag ID for placing (-1 for predefined poses)
- `gripper` (string): Gripper type ("hande" or "epick")
- `grasp_offset_json` (string): JSON string defining grasp offset from tag
- `place_poses_json` (string): Fallback poses if place_tag_id = -1
- `approach_offset` (float64): Vertical approach distance (default 0.1m)
- `retreat_offset` (float64): Vertical retreat distance (default 0.15m)

#### Grasp Offset Format

The grasp offset is specified as a JSON string:

```json
{
  "x": 0.0,      // Translation in X (meters)
  "y": 0.0,      // Translation in Y (meters)
  "z": 0.05,     // Translation in Z (meters)
  "rpy": [0, 3.14159, 0]  // Rotation (roll, pitch, yaw in radians)
}
```

### Example Task Configuration

```json
{
  "type": "vision_pick_place",
  "pick_tag_id": 10,
  "place_tag_id": 20,
  "gripper": "hande",
  "grasp_offset_json": "{\"x\": 0.0, \"y\": 0.0, \"z\": 0.05, \"rpy\": [0, 3.14159, 0]}",
  "approach_offset": 0.1,
  "retreat_offset": 0.15,
  "return_home": true
}
```

### Python Client Example

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mtc_pipeline.action import VisionPickPlaceAction
import json

class VisionPickPlaceClient(Node):
    def __init__(self):
        super().__init__('vision_pick_place_client')
        self._action_client = ActionClient(
            self,
            VisionPickPlaceAction,
            'vision_pick_place_action'
        )

    def send_goal(self, pick_tag, place_tag):
        goal_msg = VisionPickPlaceAction.Goal()
        goal_msg.pick_tag_id = pick_tag
        goal_msg.place_tag_id = place_tag
        goal_msg.gripper = "hande"

        # Configure grasp offset (5cm above tag, rotated 180° around pitch)
        grasp_offset = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.05,
            "rpy": [0, 3.14159, 0]
        }
        goal_msg.grasp_offset_json = json.dumps(grasp_offset)

        goal_msg.approach_offset = 0.1  # 10cm above grasp
        goal_msg.retreat_offset = 0.15  # 15cm above grasp

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    client = VisionPickPlaceClient()

    # Pick from tag 10, place at tag 20
    future = client.send_goal(pick_tag=10, place_tag=20)
    rclpy.spin_until_future_complete(client, future)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Execution Flow

1. **Detection Phase**
   - Detect pick AprilTag
   - Transform tag pose to base_link frame
   - Apply grasp offset to compute grasp pose
   - Calculate approach and retreat poses

2. **Pick Sequence**
   - Open gripper
   - Move to approach pose (straight line)
   - Move to grasp pose (straight line)
   - Close gripper
   - Move to retreat pose (straight line)

3. **Place Phase**
   - Detect place AprilTag (if place_tag_id >= 0)
   - Compute place poses with offsets
   - Move to place approach
   - Move to place position
   - Open gripper
   - Retreat

4. **Return Home** (optional)
   - Move to home position

## Launch Integration

Add to your launch file:

```python
Node(
    package='mtc_pipeline',
    executable='vision_pick_place_action_server',
    name='vision_pick_place_action_server',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}]
)
```

## Prerequisites

1. **AprilTag Detection**: Ensure apriltag_ros is running
2. **Camera Service**: `/capture_2d` service must be available
3. **TF Tree**: Proper transforms from camera to base_link
4. **MoveIt**: MoveIt motion planning must be configured

## Troubleshooting

### Detection Failures
- Check camera exposure and lighting
- Verify AprilTag is clean and flat
- Ensure tag size is configured correctly
- Check `/detections` topic for raw detections

### Motion Planning Failures
- Verify robot workspace limits
- Check for collisions in planning scene
- Adjust approach/retreat offsets if needed
- Review wrist constraints if orientation issues occur

### Grasp Issues
- Fine-tune grasp_offset for your specific objects
- Consider different gripper configurations
- Adjust approach speeds if needed

## Advanced Configuration

### Custom Place Poses

For scenarios where the place location isn't marked with an AprilTag:

```python
goal_msg.place_tag_id = -1  # Disable vision for place
goal_msg.place_poses_json = json.dumps({
    "place_approach": [0, -90, 90, -90, -90, 0],
    "place_target": [0, -90, 100, -90, -90, 0]
})
```

### Multiple Offset Configurations

Store different grasp offsets for various object types:

```python
offsets = {
    "cube": {"x": 0, "y": 0, "z": 0.04, "rpy": [0, 3.14, 0]},
    "cylinder": {"x": 0, "y": 0, "z": 0.06, "rpy": [0, 3.14, 0]},
    "plate": {"x": 0, "y": 0, "z": 0.02, "rpy": [0, 3.14, 0]}
}
```

## Performance Optimization

- **Detection Caching**: Recent detections are cached for 30 seconds
- **Parallel Detection**: Pick and place tags can be detected in parallel
- **Cartesian Planning**: Straight-line paths minimize computation
- **Wrist Constraints**: Applied selectively to critical moves

## Safety Considerations

- Always test with reduced speed first
- Implement collision checking in MoveIt
- Add force/torque monitoring for contact detection
- Use protective stops for unexpected collisions
- Validate tag detections before execution