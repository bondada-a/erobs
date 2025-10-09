# Step 4: TCP to Tag Movement Testing

## Goal
Move robot TCP (`robotiq_hande_tcp`) to detected AprilTag position.

## Prerequisites
- ✓ Camera publishing images (Step 1)
- ✓ Camera and tag positions verified (Step 2)
- ✓ TCP frame defined for HandE gripper (Step 3)

## System Architecture

```
Zivid Camera → /capture_2d service
                    ↓
              2D Image Published
                    ↓
              AprilTag Detector → /detections topic
                    ↓
              TF2 Transforms (tag36h11:0, tag36h11:1)
                    ↓
            Vision Action Server
                    ↓
              MTC Motion Planning
                    ↓
            Robot Executes Movement
```

## Testing Steps

### Terminal 1: Robot Drivers
```bash
cd ~/work/github_ws/erobs
source install/setup.bash
ros2 launch ur_standalone_moveit_config robot_bringup.launch.py
```

**Expected**: Robot driver, MoveIt, RViz launch successfully

---

### Terminal 2: Zivid Camera
```bash
cd ~/work/github_ws/erobs
source install/setup.bash

# Launch Zivid camera with 2D settings
ros2 run zivid_camera zivid_camera --ros-args \
  -p settings_2d_file_path:=/home/aditya/work/github_ws/erobs/src/zivid-ros/cam_settings_2d_auto.yml
```

**Expected**:
- Camera connects
- `/capture_2d` service available

**Check**:
```bash
ros2 service list | grep capture_2d
```

---

### Terminal 3: Vision System (Action Server + AprilTag)
```bash
cd ~/work/github_ws/erobs
source install/setup.bash
ros2 launch mtc_pipeline vision_system.launch.py
```

**Expected**:
- Vision action server starts
- AprilTag detector starts
- Subscribes to `/color/image_color`

**Check**:
```bash
# Check action server
ros2 action list | grep vision_move_to_action

# Check AprilTag node
ros2 node list | grep apriltag

# Check topics
ros2 topic list | grep -E "(detections|color)"
```

---

### Terminal 4: Test Movement
```bash
cd ~/work/github_ws/erobs
source install/setup.bash

# Test movement to tag 0
python3 src/mtc_pipeline/scripts/test_vision.py 0

# Or test tag 1
python3 src/mtc_pipeline/scripts/test_vision.py 1
```

---

## Debugging Checks

### 1. Verify Camera Capture Works
```bash
# Trigger manual capture
ros2 service call /capture_2d std_srvs/srv/Trigger

# Check image is published
ros2 topic echo /color/image_color --once
```

### 2. Check AprilTag Detections
```bash
# After triggering capture, check detections
ros2 topic echo /detections
```

### 3. Check TF Transforms
```bash
# List all TF frames
ros2 run tf2_ros tf2_echo base_link tag36h11:0

# Or check all frames
ros2 run tf2_tools view_frames
```

### 4. Monitor Vision Action Server
```bash
# Check action server status
ros2 action list -t

# Send test goal manually
ros2 action send_goal /vision_move_to_action mtc_pipeline/action/VisionMoveToAction "{tag_id: 0, timeout: 10.0}"
```

---

## Expected Behavior

1. **test_vision.py executes**:
   - Waits for vision action server
   - Sends goal with tag_id

2. **Vision action server**:
   - Calls `/capture_2d` service
   - Waits for detection on `/detections`
   - Gets TF transform for tag
   - Creates MTC task: "Move robotiq_hande_tcp to tag pose"
   - Plans and executes motion

3. **Robot moves**:
   - TCP moves to tag position
   - Success message returned

---

## Common Issues

| Issue | Check | Fix |
|-------|-------|-----|
| No detections | AprilTag node running? | Verify camera image topic mapping |
| No TF transform | Tag ID in config? | Add tag ID to `apriltag_config.yaml` |
| Planning fails | MoveIt running? | Check robot_bringup launched |
| Can't reach pose | Tag position reachable? | Check tag placement |
| Capture timeout | Zivid service? | Verify camera node running |

---

## Configuration Files

- **AprilTag**: `src/mtc_pipeline/config/apriltag_config.yaml`
  - Tags: [0, 1]
  - Size: 0.008m (8mm)

- **Camera 2D Settings**: `src/zivid-ros/cam_settings_2d_auto.yml`
  - Auto exposure/white balance

- **Vision Code**: `src/mtc_pipeline/src/vision_stages.cpp:151-169`
  - IK Frame: `robotiq_hande_tcp`
  - Planning group: `ur_arm`
