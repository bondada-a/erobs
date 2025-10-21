# Vision Detection Cache Feature

## Overview

The vision module now caches AprilTag detections to avoid unnecessary camera captures when:
1. A tag was recently detected (within 30 seconds)
2. The robot has not moved since the detection

This significantly improves performance when using the GUI to preview tags before executing vision_moveto tasks.

## How It Works

### Detection Caching
- Subscribes to `/detections` topic continuously
- When a tag is detected, caches:
  - Tag pose in base_link frame
  - Detection timestamp
  - Robot joint positions at detection time

### Cache Validation
Before triggering a new capture, checks if cached detection is valid:
1. **Tag exists in cache:** Tag ID was previously detected
2. **Recent detection:** Less than 30 seconds old
3. **Robot stationary:** Joint positions haven't changed by more than 0.01 radians (~0.57 degrees)

### Workflow

**With GUI preview:**
```
1. User opens GUI → sees camera view with tag overlays
2. User clicks "Capture Image" → tag detected and cached
3. User runs vision_moveto task → uses cached detection (no re-capture!)
```

**Without cache (old behavior):**
```
1. User runs vision_moveto → captures image
2. Task completes
3. User runs another vision_moveto → captures again (even if same tag)
```

## Configuration

Default parameters (in `vision_stages.hpp`):
```cpp
double cache_timeout_sec_ = 30.0;           // Cache valid for 30 seconds
double joint_movement_threshold_ = 0.01;    // 0.01 radians (~0.57 degrees)
```

## Benefits

✅ **Faster execution:** Skip camera capture when detection is cached
✅ **Better workflow:** Preview tags in GUI, then execute tasks
✅ **Reliability:** Only uses cache if robot hasn't moved
✅ **Transparency:** Logs when cache is used vs. new capture

## Example Logs

**Using cached detection:**
```
[vision_action_server] Using cached detection for tag 5 (age: 2.3s, robot stationary)
```

**Cache invalid (robot moved):**
```
[vision_action_server] No valid cached detection for tag 5, capturing...
[vision_action_server] Capture attempt 1: Triggering camera...
```

**Cache expired:**
```
[vision_action_server] No valid cached detection for tag 5, capturing...
```

## Technical Details

### Subscriptions
- `/detections` (apriltag_msgs/AprilTagDetectionArray) - For caching detections
- `/joint_states` (sensor_msgs/JointState) - For tracking robot movement

### Thread Safety
- Callbacks run in executor thread
- Cache access is simple read/write (single-threaded executor)
- No mutex needed due to ROS 2 executor model

### Memory Usage
- Stores one CachedDetection per tag ID
- Max: 9 tags × (PoseStamped + timestamp + 6 joint values) ≈ 1-2 KB

### TF Lookups
- Cache stores poses in `base_link` frame
- Detection callback performs TF lookup at cache time
- Vision task uses cached `base_link` pose directly

## Disabling Cache

To disable caching (force every task to capture):

Set `cache_timeout_sec_` to 0 in `vision_stages.hpp`:
```cpp
double cache_timeout_sec_ = 0.0;  // Disable caching
```

Then rebuild:
```bash
colcon build --packages-select mtc_pipeline
```

## Future Enhancements

Potential improvements:
- Make timeout configurable via ROS parameter
- Add cache statistics (hit rate, age distribution)
- Support pose-based invalidation (not just joint-based)
- Add manual cache clear service
