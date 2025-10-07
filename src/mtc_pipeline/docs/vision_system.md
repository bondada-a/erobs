# Vision System with AprilTag Detection

## Overview
The vision system enables the robot to detect AprilTag markers using the Zivid camera and move relative to detected objects. This is useful for vision-based pick and place operations where exact object positions are not known in advance.

## Components

### 1. Vision Action Server (`vision_action_server`)
- Standalone ROS2 action server for vision-based movements
- Subscribes to AprilTag detections
- Executes MoveIt motion plans to approach detected tags

### 2. VisionMoveToAction
Action definition with the following parameters:
- `tag_id`: AprilTag ID to detect (integer)
- `approach_distance`: Distance from tag to stop (meters, default: 0.1)
- `timeout`: Detection timeout (seconds, default: 5.0)
- `approach_direction`: Direction to approach from ("x", "-x", "y", "-y", "z", "-z")
- `use_preset_height`: Override Z with preset value (boolean)
- `preset_height`: Fixed height for approach (meters, default: 0.15)

### 3. AprilTag Configuration (`config/apriltag_config.yaml`)
- Tag family: tag36h11 (default)
- Tag sizes: Configurable per tag ID
- Detection parameters: refinement, threading, etc.

## Usage

### 1. Launch the Vision System
```bash
ros2 launch mtc_pipeline vision_system.launch.py
```

Optional parameters:
- `launch_apriltag:=true/false` - Launch AprilTag detector
- `camera_namespace:=/zivid_camera` - Camera namespace
- `tag_config_file:=<path>` - Custom config file

### 2. Launch the Orchestrator
```bash
ros2 launch mtc_pipeline mtc_orchestrator.launch.py
```

### 3. Send Vision Tasks
Use the test JSON file:
```bash
ros2 action send_goal /mtc_execution_action mtc_pipeline/action/MTCExecution \
  "{robot_ip: '192.168.56.101', start_gripper: 'epick', poses_json: '{}', steps_json: '$(cat vision_test.json)'}"
```

## JSON Task Format
```json
{
  "task_type": "vision_moveto",
  "tag_id": 0,                  // Tag to detect
  "approach_distance": 0.1,     // Stop 10cm from tag
  "timeout": 10.0,              // Wait up to 10 seconds
  "approach_direction": "z",    // Approach from above
  "use_preset_height": true,    // Use fixed height
  "preset_height": 0.15         // 15cm above table
}
```

## Approach Directions
- `"x"` or `"forward"`: Move in negative X (forward)
- `"-x"` or `"backward"`: Move in positive X (backward)
- `"y"` or `"right"`: Move in negative Y (right)
- `"-y"` or `"left"`: Move in positive Y (left)
- `"z"` or `"up"`: Move in positive Z (up/above)
- `"-z"` or `"down"`: Move in negative Z (down/below)

## Tag Frame Convention
AprilTag poses are published via TF with frame IDs in format: `<family>:<id>`
Example: `tag36h11:0` for tag ID 0 of family tag36h11

## Troubleshooting

### No Tag Detected
- Check camera is publishing images to `/zivid_camera/color/image_raw`
- Verify tag is visible and well-lit
- Check tag size in config matches physical tag
- Verify AprilTag detector is running: `ros2 topic echo /apriltag/detections`

### Transform Errors
- Ensure robot description is loaded
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify camera is calibrated and transform is published

### Motion Planning Failures
- Check robot is at a valid starting configuration
- Verify approach pose is reachable
- Try different approach directions
- Reduce approach distance

## Example Workflow
1. Place AprilTag markers on objects to manipulate
2. Configure tag IDs and sizes in apriltag_config.yaml
3. Launch vision system and orchestrator
4. Send vision tasks to detect and approach objects
5. Chain with gripper actions for pick and place

## Integration with Pick & Place
Vision tasks can be combined with other tasks:
```json
{
  "steps": [
    {"task_type": "vision_moveto", "tag_id": 0, ...},
    {"task_type": "endeffector", "command": "close"},
    {"task_type": "moveto", "location": "home"},
    {"task_type": "endeffector", "command": "open"}
  ]
}
```

This enables fully autonomous vision-based manipulation without hardcoded positions.