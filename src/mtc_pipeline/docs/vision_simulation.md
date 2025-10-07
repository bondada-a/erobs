# Vision System Simulation Mode

## Overview
Test the vision-based movement system without physical camera or AprilTags using the simulation mode. This uses a mock AprilTag detector that publishes simulated tag detections and transforms.

## Quick Start

### 1. Launch MoveIt and Robot (if not already running)
```bash
# Terminal 1: Launch your robot system
ros2 launch erobs_moveit_interface_sim demo.launch.py
```

### 2. Launch Vision System in Simulation Mode
```bash
# Terminal 2: Launch simulated vision system
ros2 launch mtc_pipeline vision_system_sim.launch.py
```

### 3. Launch the Orchestrator
```bash
# Terminal 3: Launch the orchestrator
ros2 launch mtc_pipeline mtc_orchestrator.launch.py
```

### 4. Run Tests

#### Option A: Simple tag approach test
```bash
# Terminal 4: Approach tag 0 from above
python3 src/mtc_pipeline/scripts/test_vision_sim.py --tag 0 --distance 0.1 --direction z
```

#### Option B: Pick and place demo
```bash
# Terminal 4: Run full pick and place demo
python3 src/mtc_pipeline/scripts/test_vision_sim.py --demo
```

#### Option C: Manual JSON test
```bash
# Terminal 4: Send custom test
ros2 action send_goal /mtc_execution_action mtc_pipeline/action/MTCExecution \
  "{robot_ip: '192.168.56.101', start_gripper: 'epick', poses_json: '{}', steps_json: '$(cat vision_test.json)'}"
```

## Simulated Tags

The mock detector simulates 3 tags by default:
- **Tag 0**: Center of workspace (0.5m forward, 0.02m above table)
- **Tag 1**: Left side (0.4m forward, 0.2m left)
- **Tag 2**: Right side (0.4m forward, 0.2m right)

## Launch Parameters

### vision_system_sim.launch.py
```bash
ros2 launch mtc_pipeline vision_system_sim.launch.py \
  tag_ids:="[0,1,2,3,4]" \          # Tag IDs to simulate
  moving_tags:=true \                # Enable moving tags
  movement_radius:=0.05 \            # Movement radius in meters
  movement_speed:=0.3                # Movement speed in rad/s
```

### Moving Tags Demo
To simulate dynamic scenes with moving objects:
```bash
ros2 launch mtc_pipeline vision_system_sim.launch.py moving_tags:=true
```

Tags will move in circles, each with a different phase offset.

## Test Script Options

### test_vision_sim.py
```bash
# Approach specific tag
python3 test_vision_sim.py --tag 1 --distance 0.15 --direction -z

# Run pick and place demo
python3 test_vision_sim.py --demo

# Options:
#   --tag ID         Tag ID to approach (0, 1, 2, ...)
#   --distance M     Approach distance in meters
#   --direction DIR  Approach from: x, -x, y, -y, z, -z
#   --demo          Run full pick/place demo sequence
```

## Monitoring Simulation

### Check Published Topics
```bash
# View simulated detections
ros2 topic echo /apriltag/detections

# Check TF transforms
ros2 run tf2_tools view_frames

# Monitor specific tag transform
ros2 run tf2_ros tf2_echo base_link tag36h11:0
```

### Visualize in RViz
```bash
# Launch RViz with robot model
ros2 run rviz2 rviz2

# Add displays:
# - RobotModel
# - TF (to see tag frames)
# - MarkerArray (if visualization markers are published)
```

## Troubleshooting

### "Action server not available"
- Ensure orchestrator is running: `ros2 launch mtc_pipeline mtc_orchestrator.launch.py`
- Check vision action server: `ros2 node list | grep vision`

### "Transform not available"
- Verify mock detector is running: `ros2 node list | grep mock`
- Check TF tree: `ros2 run tf2_tools view_frames`

### "Goal rejected"
- Check robot is in valid start state
- Verify MoveIt is running: `ros2 node list | grep move_group`

### Motion planning fails
- Tags might be out of robot reach
- Try different approach directions
- Reduce approach distance

## Example Sequences

### Pick from tag 0, place at tag 1:
```json
{
  "steps": [
    {"task_type": "moveto", "location": "home"},
    {"task_type": "vision_moveto", "tag_id": 0, "approach_distance": 0.1},
    {"task_type": "endeffector", "command": "close"},
    {"task_type": "moveto", "location": "home"},
    {"task_type": "vision_moveto", "tag_id": 1, "approach_distance": 0.1},
    {"task_type": "endeffector", "command": "open"}
  ]
}
```

### Scan multiple tags:
```json
{
  "steps": [
    {"task_type": "vision_moveto", "tag_id": 0, "approach_distance": 0.2},
    {"task_type": "vision_moveto", "tag_id": 1, "approach_distance": 0.2},
    {"task_type": "vision_moveto", "tag_id": 2, "approach_distance": 0.2}
  ]
}
```

## Benefits of Simulation Mode
- Test vision logic without hardware
- Rapid development and debugging
- Consistent test environment
- Dynamic scenes with moving tags
- No camera calibration needed
- Works with any robot configuration