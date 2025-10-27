# Vision Detection - Gripper Configuration

## Overview

The vision detection system **automatically detects** which gripper is attached and uses the appropriate TCP frame and Z-offset. Manual configuration is also supported.

## Auto-Detection (Default Behavior)

**The system automatically detects which gripper is loaded** by checking the TF tree:

1. When `vision_action_server` starts, it waits 2 seconds for TF to populate
2. Checks if `epick_tip` frame exists → Uses EPick configuration
3. Checks if `robotiq_hande_end` frame exists → Uses Hand-E configuration
4. Falls back to Hand-E if neither detected

**You'll see in the logs:**
```
[vision_action_server] Auto-detecting gripper TCP frame...
[vision_action_server]   Detected: Robotiq Hand-E gripper
[vision_action_server] VisionStages initialized with Zivid ArUco detection (dictionary: aruco4x4_50, ik_frame: robotiq_hande_end, z_offset: -0.020m, cache: 30s)
```

**No configuration needed** - just launch with the appropriate URDF and it works!

## Supported Grippers

### 1. Robotiq Hand-E (Default)

**Configuration:**
```yaml
ik_frame: 'robotiq_hande_end'
z_offset: -0.02  # 2cm lower
```

**Description:**
- TCP at finger tips
- 2cm Z-offset to account for gripper geometry
- Works well for grasping objects on flat surfaces

### 2. Robotiq EPick

**Configuration:**
```yaml
ik_frame: 'epick_tip'
z_offset: -0.08  # 8cm lower (adjust based on testing)
```

**Description:**
- TCP at suction cup tip
- Larger Z-offset needed due to longer gripper body (~71mm)
- Recommended for flat objects that can be vacuum-picked

## Manual Configuration (Optional)

**Auto-detection works in most cases**, but you can manually override if needed.

### Method 1: Edit Launch File

Edit `src/mtc_pipeline/launch/modular_action_servers.launch.py`:

**Auto-detect (default):**
```python
vision_action_server = Node(
    package='mtc_pipeline',
    executable='vision_action_server',
    name='vision_action_server',
    output='screen',
    parameters=action_server_parameters + [
        {'publish_marker_frames': True},
        {'ik_frame': ''},  # Empty string = auto-detect
    ]
)
```

**Force Hand-E:**
```python
vision_action_server = Node(
    package='mtc_pipeline',
    executable='vision_action_server',
    name='vision_action_server',
    output='screen',
    parameters=action_server_parameters + [
        {'publish_marker_frames': True},
        {'ik_frame': 'robotiq_hande_end'},
        {'z_offset': -0.02}  # Optional override
    ]
)
```

**Force EPick:**
```python
vision_action_server = Node(
    package='mtc_pipeline',
    executable='vision_action_server',
    name='vision_action_server',
    output='screen',
    parameters=action_server_parameters + [
        {'publish_marker_frames': True},
        {'ik_frame': 'epick_tip'},
        {'z_offset': -0.08}  # Start with -8cm, tune as needed
    ]
)
```

### Method 2: Runtime Parameter Update

Change parameters while the system is running:

```bash
# Switch to EPick
ros2 param set /vision_action_server ik_frame epick_tip
ros2 param set /vision_action_server z_offset -0.08

# Switch back to Hand-E
ros2 param set /vision_action_server ik_frame robotiq_hande_end
ros2 param set /vision_action_server z_offset -0.02
```

**Note:** Runtime changes require restarting the action to take effect.

## Z-Offset Tuning

The Z-offset determines how close the TCP gets to the detected marker surface.

### Finding the Right Offset:

1. **Start conservative** (smaller absolute value, e.g., -0.05m)
2. **Test with vision_moveto:**
   ```bash
   ros2 action send_goal /vision_move_to_action mtc_pipeline/action/VisionMoveToAction \
     '{tag_id: 2, timeout: 10.0, poses_json: "{}"}'
   ```
3. **Observe in RViz:**
   - Enable TF display
   - Check distance between TCP and marker frame
4. **Adjust:**
   - **TCP too high** → More negative offset (e.g., -0.02 → -0.04)
   - **TCP too low** → Less negative offset (e.g., -0.08 → -0.06)
5. **Rebuild and restart** after editing launch file

### Typical Values:

| Gripper | Z-Offset | Notes |
|---------|----------|-------|
| **Hand-E** | -0.02m | Fingers at ~2cm above tag |
| **EPick** | -0.08m | Suction cup at ~8cm above tag (tune based on setup) |
| **Custom** | Variable | Measure your TCP-to-contact distance |

## Testing Procedure

### 1. With Hand-E (Current Setup)

Already working! Current configuration:
- ik_frame: `robotiq_hande_end`
- z_offset: `-0.02`

### 2. With EPick

**Steps:**
1. Edit launch file to EPick parameters
2. Rebuild:
   ```bash
   colcon build --packages-select mtc_pipeline
   ```
3. Restart action servers:
   ```bash
   source install/setup.bash
   ros2 launch mtc_pipeline modular_action_servers.launch.py robot_ip:=192.168.1.101
   ```
4. Test detection:
   ```bash
   ros2 action send_goal /vision_move_to_action mtc_pipeline/action/VisionMoveToAction \
     '{tag_id: 2, timeout: 10.0, poses_json: "{}"}'
   ```
5. Check in RViz:
   - Is `epick_tip` frame aligned with `aruco_2`?
   - Is suction cup at correct height above marker?
6. Adjust `z_offset` if needed and repeat

## Advanced: Custom Grippers

For custom end-effectors:

1. **Identify TCP frame** in URDF:
   ```bash
   grep -r "tcp\|tip\|end" your_gripper.urdf
   ```

2. **Measure Z-distance** from TCP to contact point

3. **Configure in launch file:**
   ```python
   {'ik_frame': 'your_tcp_frame'},
   {'z_offset': -0.0X}  # Your measured distance
   ```

## Troubleshooting

### "GOAL_STATE_INVALID" Error

**Cause:** IK frame doesn't exist in URDF

**Fix:**
```bash
# Check available frames
ros2 run tf2_ros tf2_echo base_link your_frame_name

# Verify frame exists
ros2 run tf2_tools view_frames
```

### Gripper Collides with Object

**Cause:** Z-offset too large (too negative)

**Fix:** Reduce absolute value (e.g., -0.08 → -0.06)

### Gripper Too Far from Object

**Cause:** Z-offset too small

**Fix:** Increase absolute value (e.g., -0.02 → -0.04)

### Wrong Orientation

**Issue:** All grippers use 180° Z-rotation after detection

**Why:** ArUco markers have a standard orientation, rotation aligns gripper correctly

**If needed:** Modify rotation in `vision_stages.cpp:252` (e.g., change `M_PI` to `M_PI/2` for 90°)

## Summary

- ✅ **Auto-detection enabled by default** - no configuration needed!
- ✅ System detects Hand-E or EPick automatically from TF tree
- ✅ Correct TCP frame and Z-offset selected automatically:
  - **Hand-E:** `robotiq_hande_end`, `-0.02m`
  - **EPick:** `epick_tip`, `-0.08m`
- ✅ Manual override available if needed
- ✅ Check logs to see which gripper was detected
- ✅ Works with any URDF - just launch and go!

**Testing:**
1. Launch your robot with Hand-E or EPick URDF
2. Launch action servers: `ros2 launch mtc_pipeline modular_action_servers.launch.py`
3. Check logs: Should say "Detected: Robotiq Hand-E gripper" or "Detected: Robotiq EPick gripper"
4. Test: `ros2 action send_goal /vision_move_to_action ...`

No configuration needed - the system adapts automatically!
