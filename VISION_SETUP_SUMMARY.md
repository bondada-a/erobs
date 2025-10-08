# Zivid Vision System Setup - Summary

## ✅ Changes Made

### 1. **Added Optical Frame to URDF**
**File**: `src/ur5e_robot_description/urdf/zivid_camera_mount.xacro`

**What was added:**
- `zivid_optical_frame` link (measurement/camera frame)
- `zivid_optical_joint` connecting camera body to optical center
- Support for multiple Zivid models (ZIVID_2_M70, ZIVID_2_PLUS_M60, etc.)
- Configurable optical center values based on Zivid specifications

**Key Details:**
- Camera model: **Zivid 2 Plus MR60**
- Optical center: xyz="0.049 0.03202 0.0295"
- Orientation: rpy="-π/2 0 -(π/2 + 2.5°)"

### 2. **TF Tree Now Complete**
```
base_link → ... → flange → zivid_camera → zivid_optical_frame → tag36h11:X
```

## 📋 Verification Results

- ✅ URDF generates correctly
- ✅ Optical frame present in robot description
- ✅ TF chain properly connected
- ✅ Build successful (all packages compiled)

## 🚀 How to Use the Vision System

### Option 1: With Real Zivid Camera

```bash
# Terminal 1: Launch robot + vision system
ros2 launch mtc_pipeline vision_system.launch.py

# Terminal 2: Test vision move-to
python3 src/mtc_pipeline/scripts/test_vision.py 0  # Detect and move to tag ID 0
```


### Verify TF Tree

```bash
# Check if optical frame exists
ros2 run tf2_tools view_frames

# Verify transform from base to optical frame
ros2 run tf2_ros tf2_echo base_link zivid_optical_frame

# Monitor AprilTag detections
ros2 topic echo /apriltag/detections
```

## 🔧 Configuration Files

### AprilTag Settings
**File**: `src/mtc_pipeline/config/apriltag_config.yaml`

Key parameters:
- `camera_frame: "zivid_optical_frame"` ✅ (now matches URDF)
- `tag_edge_size: 0.05` (50mm tags - adjust for your tags!)
- `tag_family: "tag36h11"`

### Vision System Launch
**File**: `src/mtc_pipeline/launch/vision_system.launch.py`

Camera topic remappings:
```python
('image_rect', '/zivid_camera/color/image_raw'),
('camera_info', '/zivid_camera/color/camera_info'),
```

**⚠️ Important**: Verify these match your actual Zivid camera topics!

## 📝 What You Need to Change

### 1. **Camera Topics** (if using real Zivid)
Check your Zivid camera's actual topics:
```bash
ros2 topic list | grep zivid
```

Then update `vision_system.launch.py` remappings if needed.

### 2. **Tag Sizes**
Measure your physical AprilTags and update `apriltag_config.yaml`:
```yaml
tag_sizes:
  0: 0.05   # Tag 0: 50mm (update this!)
  1: 0.05   # Tag 1: 50mm
```

### 3. **Zivid Model** (optional)
If using a different Zivid model, update the call in `ur_with_zivid_hande.xacro`:
```xml
<xacro:zivid_camera_mount
  parent_link="$(arg tf_prefix)flange"
  xyz="0.025 0 -0.105"
  rpy="-1.5708 0 -1.5708"
  model="ZIVID_2_PLUS_M60"/>  <!-- Add this if needed -->
```

Supported models:
- `ZIVID_2_M70` (default)
- `ZIVID_2_L100`
- `ZIVID_2_PLUS_M60`
- `ZIVID_2_PLUS_L110`
- `ZIVID_2_PLUS_M130`

### 4. **Hand-Eye Calibration** (recommended)
The optical frame transform is based on factory specifications. For precision applications:

1. Perform hand-eye calibration
2. Update optical center values in `zivid_camera_mount.xacro`
3. Rebuild: `colcon build --packages-select ur5e_robot_description`

## 🐛 Troubleshooting

### Issue: Tag not detected
```bash
# Check camera feed
ros2 topic hz /zivid_camera/color/image_raw

# Check AprilTag node is running
ros2 node list | grep apriltag

# Enable debug mode in apriltag_config.yaml
debug_mode: true
```

### Issue: TF lookup fails
```bash
# List all frames
ros2 run tf2_tools view_frames
# Check frames.pdf for missing transforms

# Check TF connectivity
ros2 run tf2_ros tf2_echo base_link zivid_optical_frame
```

### Issue: Move-to fails
- Verify MoveIt is running and planning group is correct
- Check collision detection isn't blocking the path
- Ensure target pose is reachable

## 📚 Code Structure

### Vision System Files
```
mtc_pipeline/
├── src/
│   ├── vision_stages.cpp           # Main vision logic
│   └── vision_action_server.cpp    # ROS2 action server
├── launch/
│   └── vision_system.launch.py     # System launcher
├── config/
│   └── apriltag_config.yaml        # Tag detection config
├── scripts/
│   └── test_vision.py              # Test client
└── action/
    └── VisionMoveToAction.action   # Action definition
```

### Key Functions
- **`detect_tag()`**: Waits for AprilTag detection with timeout
- **`move_to_pose()`**: Creates MTC task to move to detected pose
- **`run()`**: Main entry point - detect + move

## 🎯 Next Steps for Production Use

1. **Add offset/approach poses** to avoid collisions
2. **Implement error recovery** if detection fails
3. **Add pose validation** (reachability check)
4. **Tune detection parameters** for your environment
5. **Perform hand-eye calibration** for accuracy
6. **Add gripper approach vector** based on tag orientation

## 📊 Expected Performance

- **Detection timeout**: 5.0 seconds (configurable)
- **Tag update rate**: 10 Hz
- **TF lookup timeout**: 100ms
- **Planning time**: ~2-5 seconds (depends on MoveIt config)

---

**Last Updated**: $(date)
**Status**: ✅ Ready for testing
