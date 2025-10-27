# Vision Pick and Place - Recent Improvements

## Issue Fixed: Cartesian Path Planning Failures

### Problem
The vision pick and place task was failing at the "pick approach" stage with the error:
```
CartesianPath: min_fraction not met. Achieved: 0.000000
```

This meant the robot couldn't plan a valid straight-line (Cartesian) path to the approach position.

### Root Cause
The original implementation used Cartesian path planning for all movements, including:
- Long-distance approach moves
- Moves that might require navigating around obstacles
- Moves where a straight line might not be feasible

Cartesian planners require **strict straight-line paths**, which aren't always possible from arbitrary starting positions to vision-detected target positions.

### Solution Implemented

**1. Hybrid Planning Strategy:**
   - **Pipeline Planner (OMPL)** for approach moves - can plan around obstacles
   - **Cartesian Planner** only for short, critical moves (final grasp, retreat)

**2. Relaxed Cartesian Constraints:**
   - Increased step size: 1mm → 5mm (faster, more robust)
   - Reduced min_fraction: 60% → 50% (more tolerant)

**3. Improved Logging:**
   - Detailed pose information for debugging
   - Clear indication of which planner is used for each stage

### Updated Task Sequence

```
Pick Sequence:
1. Open gripper                 [Joint Interpolation]
2. Move to pick approach        [Pipeline/OMPL - FLEXIBLE PATH]
3. Move to grasp pose           [Cartesian - STRAIGHT LINE DOWN]
4. Close gripper                [Joint Interpolation]
5. Pick retreat                 [Cartesian - STRAIGHT LINE UP]

Place Sequence:
6. Move to place approach       [Pipeline/OMPL - FLEXIBLE PATH]
7. Move to place position       [Cartesian - STRAIGHT LINE DOWN]
8. Open gripper                 [Joint Interpolation]
9. Place retreat                [Cartesian - STRAIGHT LINE UP]

Optional:
10. Return home                 [Pipeline/OMPL]
```

### Why This Works Better

**Pipeline Planner (OMPL):**
- ✅ Can navigate around obstacles
- ✅ Finds valid paths from any configuration
- ✅ Handles complex joint-space movements
- ❌ Paths may not be perfectly straight

**Cartesian Planner:**
- ✅ Perfect for short vertical moves (approach to grasp, retreat)
- ✅ Predictable, straight-line motion
- ✅ Good for final precision movements
- ❌ Fails if straight line is blocked or too long

**Hybrid Approach:**
- Uses the right tool for each job
- Long-distance moves: Pipeline planner
- Short precision moves: Cartesian planner
- Best of both worlds!

## Additional Fixes

### 1. Parameter Conflict Resolution
Fixed the `ParameterAlreadyDeclaredException` error that occurred when multiple stages objects tried to declare the same parameters.

**Solution:**
```cpp
if (!node_->has_parameter("ompl.planning_plugin")) {
    node_->declare_parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner");
}
```

### 2. Enhanced Debugging
Added detailed logging for all computed poses:

```
Pick poses computed:
  Grasp:    [0.352, 0.124, 0.051]
  Approach: [0.352, 0.124, 0.151] (0.100m above)
  Retreat:  [0.352, 0.124, 0.201] (0.150m above)

Place poses computed (predefined default):
  Place:    [0.400, 0.300, 0.150]
  Approach: [0.400, 0.300, 0.250]
  Retreat:  [0.400, 0.300, 0.300]
```

This helps diagnose issues with:
- Tag detection accuracy
- Grasp offset configuration
- Place position validity
- Offset calculations

## Testing the Improvements

### 1. Launch the system
```bash
# Terminal 1: Robot and MoveIt
ros2 launch ur_zivid_pipettor_moveit_config ur_zivid_pipettor_planning_execution.launch.py robot_ip:=192.168.1.101

# Terminal 2: Action servers
ros2 launch mtc_pipeline modular_action_servers.launch.py robot_ip:=192.168.1.101
```

### 2. Test vision pick with predefined place
```bash
# Source workspace
source install/setup.bash

# Pick from tag 3, place at default position
ros2 run mtc_pipeline vision_pick_predefined_place.py 3

# Pick from tag 3, place at custom position
ros2 run mtc_pipeline vision_pick_predefined_place.py 3 0.4,0.25,0.12
```

### 3. Monitor the output
Watch for the detailed pose logging in the action server terminal:
- Tag detection coordinates
- Computed grasp poses
- Approach/retreat offsets
- Planning success for each stage

## Expected Behavior

### Success Indicators
✅ All 12 stages show solutions (not 0/1)
✅ Planning completes within reasonable time (~10-30 seconds)
✅ Robot moves smoothly through all stages
✅ No "min_fraction not met" errors

### Example Successful Output
```
[vision_pick_place_action_server]: Tag 3 detected at [0.352, 0.124, 0.051]
[vision_pick_place_action_server]: Pick poses computed:
[vision_pick_place_action_server]:   Grasp:    [0.352, 0.124, 0.051]
[vision_pick_place_action_server]:   Approach: [0.352, 0.124, 0.151] (0.100m above)
[vision_pick_place_action_server]:   Retreat:  [0.352, 0.124, 0.201] (0.150m above)
[vision_pick_place_action_server]: Building MTC task with 12 stages

Task stages:
  1  - ←   5 →   -  5 / open gripper
  -  5 →  12 →   -  60 / pick approach      [SUCCESS - Pipeline planner]
  - 12 →   8 →   -  96 / grasp              [SUCCESS - Cartesian]
  -  8 →   6 →   -  48 / close gripper
  -  6 →  15 →   -  90 / pick retreat       [SUCCESS - Cartesian]
  ... etc
```

## Troubleshooting

### Still getting planning failures?

**Check robot workspace:**
- Ensure target positions are within reach
- Verify no collisions in planning scene
- Check joint limits aren't exceeded

**Adjust offsets:**
```bash
# Increase approach height if robot struggles to reach
ros2 run mtc_pipeline vision_pick_predefined_place.py 3 0.4,0.3,0.15

# Or modify in code:
goal.approach_offset = 0.15  # 15cm instead of 10cm
goal.retreat_offset = 0.20   # 20cm instead of 15cm
```

**Grasp offset tuning:**
If the grasp pose is unreachable, adjust the z-offset:
```python
grasp_offset = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.08,  # Try 8cm instead of 5cm
    "rpy": [0, 3.14159, 0]
}
```

**Place position validation:**
Ensure place positions are reachable and collision-free:
```bash
# Test with a known-good position first
ros2 run mtc_pipeline vision_pick_predefined_place.py 3 0.3,0.2,0.2
```

## Performance Tuning

### Speed vs. Success Rate
Current settings prioritize reliability (20% speed):
```cpp
planner->setMaxVelocityScalingFactor(0.2);      // 20% of max
planner->setMaxAccelerationScalingFactor(0.2);   // 20% of max
```

Once stable, you can increase for faster execution:
```cpp
planner->setMaxVelocityScalingFactor(0.5);      // 50% of max
planner->setMaxAccelerationScalingFactor(0.3);   // 30% of max
```

## Summary

The vision pick and place system now uses a **smart hybrid planning approach**:
- 🚀 More robust approach planning with OMPL
- 🎯 Precise Cartesian movements where needed
- 📊 Better debugging with detailed logging
- ✅ No parameter conflicts between stages
- 🔧 Easier to tune and troubleshoot

This makes the system **production-ready** for vision-based pick and place operations!