# Testing Zivid Built-in ArUco Detection

## What Was Changed

✅ **Updated:** `launch/modular_action_servers.launch.py`
- Added `settings_file_path` parameter pointing to your `zivid_3d_settings.yml`
- This enables Zivid's `/capture_and_detect_markers` service

## ⚠️ CRITICAL: You Need ArUco Markers!

**Zivid's built-in detection ONLY works with ArUco markers, NOT AprilTag!**

### What You Currently Have
- ✅ **AprilTag markers** (tag36h11 family) - working with apriltag_ros
- Works perfectly for 2D image-based detection

### What You Need to Test Zivid
- ❌ **ArUco markers** (e.g., aruco4x4_50 dictionary)
- Different marker format - NOT compatible with AprilTag

## Step-by-Step Testing Guide

### Option 1: Print an ArUco Marker (Recommended for Testing)

1. **Generate ArUco marker:**
   - Visit: https://chev.me/arucogen/
   - Settings:
     - Dictionary: `DICT_4X4_50`
     - Marker ID: `3` (same as your AprilTag for comparison)
     - Marker Size: `100mm` (or same size as your current tag)
   - Download and print

2. **Place the ArUco marker** where your robot can see it

3. **Restart the system** with the new launch file:
   ```bash
   # Terminal 1: Kill existing launch if running (Ctrl+C)
   # Then restart with updated settings
   ros2 launch mtc_pipeline modular_action_servers.launch.py robot_ip:=192.168.1.101
   ```

4. **Run the comparison test:**
   ```bash
   # Terminal 2
   source install/setup.bash
   ros2 run mtc_pipeline test_zivid_marker_detection.py 3
   ```

### Option 2: Command Line Testing

**Test Zivid detection directly:**
```bash
# Call the service
ros2 service call /capture_and_detect_markers \
  zivid_interfaces/srv/CaptureAndDetectMarkers \
  "{marker_ids: [3], marker_dictionary: 'aruco4x4_50'}"
```

**Expected output (if ArUco marker present):**
```yaml
success: True
message: "Markers detected successfully"
detection_result:
  detected_markers:
    - id: 3
      pose:
        position: {x: 0.352, y: 0.124, z: 0.051}
        orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
      corners_in_camera_coordinates: [...]  # 4 corners in 3D!
```

## Expected Results

### AprilTag (Current - Should Work)
```
✓ AprilTag 3 detected!
  Family: tag36h11
  Centre: [1115.77, 854.66] px
  Decision margin: 69.32
```

### ArUco (Zivid - Only if you have ArUco marker)
```
✓ ArUco marker 3 detected!
  Pose (camera frame):
    Position: [0.352, 0.124, 0.051] m
    Orientation: [0.0, 1.0, 0.0, 0.0]
  4 Corners in 3D:
    Corner 0: [0.347, 0.119, 0.051] m
    Corner 1: [0.357, 0.119, 0.051] m
    ...
```

**Notice:** ArUco provides **3D corner positions** - this is the key advantage!

## Comparison: What You Get

| Feature | AprilTag (apriltag_ros) | ArUco (Zivid) |
|---------|------------------------|---------------|
| **Marker type** | AprilTag ✅ (you have) | ArUco ❌ (need new) |
| **Detection** | 2D image | 3D point cloud ✅ |
| **Position data** | 2D pixels | 3D meters ✅ |
| **Corner data** | 2D only | 2D + 3D ✅ |
| **Accuracy** | ~1-2mm | ~0.5-1mm ✅ |
| **Setup** | Simple ✅ | Need config |
| **Currently working** | Yes ✅ | Need ArUco marker |

## If You Don't Have ArUco Markers Yet

**You can still verify the setup is correct:**

```bash
# Should return error about no markers detected (not settings error)
ros2 service call /capture_and_detect_markers \
  zivid_interfaces/srv/CaptureAndDetectMarkers \
  "{marker_ids: [3], marker_dictionary: 'aruco4x4_50'}"
```

**Before fix:**
```
success: False
message: "Both 'settings_file_path' and 'settings_yaml' parameters are empty!"
```

**After fix (with ArUco marker visible):**
```
success: True
message: "Markers detected successfully"
```

**After fix (NO ArUco marker, but settings OK):**
```
success: True
message: "Detection completed"
detection_result:
  detected_markers: []  # Empty, but no error!
```

## Decision Time: Which Approach?

### Stick with AprilTag (apriltag_ros) if:
- ✅ Current accuracy (1-2mm) is sufficient for your grasping tasks
- ✅ Don't want to replace all physical markers
- ✅ Value simplicity and standard ROS 2 integration
- ✅ Might switch cameras later (Realsense, etc.)
- ✅ It's already working perfectly

### Switch to ArUco (Zivid built-in) if:
- ✅ Need sub-millimeter accuracy (~0.5mm)
- ✅ Want to leverage full 3D point cloud data
- ✅ Need 3D corner positions for validation
- ✅ Willing to replace markers and recalibrate
- ✅ Committed to Zivid long-term
- ✅ Have time for integration (~4-8 hours)

## My Recommendation

**Start with AprilTag test, then decide:**

1. Run: `ros2 run mtc_pipeline test_zivid_marker_detection.py 3`
2. See AprilTag working (it will!)
3. **Then ask:** "Do I really need better than 1-2mm accuracy?"

For most robotic grasping, **1-2mm is plenty**. Save yourself the marker replacement hassle unless you have a specific need for sub-mm precision.

## Next Steps After Testing

### If Staying with AprilTag:
```bash
# Test the fixed pick/place (simplified poses)
ros2 run mtc_pipeline vision_pick_predefined_place.py 3
```

### If Switching to ArUco:
1. Print ArUco markers for all your objects
2. Replace physical markers
3. Test detection with this script
4. If accuracy is noticeably better, integrate fully
5. Update all configurations and documentation

## Troubleshooting

### "Settings error" on service call
- ✅ **FIXED!** You added the settings file to launch
- Restart launch file if you see this

### "No markers detected"
- Check: Do you have ArUco marker (not AprilTag)?
- Check: Is marker visible to camera?
- Check: Is marker dictionary correct? (aruco4x4_50)

### ArUco marker won't detect
- Verify marker is printed clearly (no blur, folds)
- Check lighting conditions
- Ensure marker is flat and fully visible
- Try getting closer (0.5-1m range)

## Resources

**Generate ArUco markers:**
- https://chev.me/arucogen/
- OpenCV: `cv2.aruco.drawMarker()`

**Supported dictionaries:**
- aruco4x4_50, aruco5x5_100, aruco6x6_250, etc.
- Use same size as your current AprilTag (~100mm typical)

**Remember:** ArUco ≠ AprilTag - they are different marker families!