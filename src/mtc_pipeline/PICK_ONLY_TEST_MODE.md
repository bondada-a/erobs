# Vision Pick and Place - PICK-ONLY Test Mode

## Current Configuration

The vision pick and place system is currently configured in **PICK-ONLY mode** for testing. The place sequence has been temporarily commented out.

## Active Stages (5 Total)

### Pick Sequence

| Stage | Type | What It Does | Planner Used |
|-------|------|--------------|--------------|
| 1. **Open gripper** | Gripper Action | Opens the gripper to prepare for picking | Joint Interpolation |
| 2. **Pick approach** | Movement | Moves to a position ABOVE the detected object (e.g., 10cm above) | Pipeline/OMPL |
| 3. **Grasp** | Movement | Moves DOWN from approach position to the actual object | Cartesian |
| 4. **Close gripper** | Gripper Action | Closes the gripper to grab the object | Joint Interpolation |
| 5. **Pick retreat** | Movement | Moves UP with the grasped object to a safe height | Cartesian |

## Understanding "Grasp" vs "Close Gripper"

**Common Confusion:** "Why do we have both 'grasp' and 'close gripper'?"

**Answer:** They do completely different things!

- **"Grasp" stage** = **MOVEMENT** from approach position down to object
  - Type: Cartesian path (straight line down)
  - Purpose: Position the gripper fingers around the object
  - Does NOT touch the gripper

- **"Close gripper" stage** = **GRIPPER ACTION** to actually grab
  - Type: Joint space movement of gripper fingers
  - Purpose: Close the gripper to physically grasp the object
  - Does NOT move the arm

### Visual Sequence

```
Initial Position: Robot anywhere in workspace
                  |
                  v
         1. OPEN GRIPPER
         (prepare gripper)
                  |
                  v
        2. PICK APPROACH
    (move above object at safe height)
         Approach pose: [x, y, z+0.1m]
                  |
                  v
           3. "GRASP"
    (move DOWN to object position)
         Grasp pose: [x, y, z]
                  |
                  v
       4. CLOSE GRIPPER
      (grab the object)
                  |
                  v
        5. PICK RETREAT
    (move UP with object)
         Retreat pose: [x, y, z+0.15m]
                  |
                  v
         END (holding object)
```

## Commented Out Stages (for later testing)

The following stages are temporarily disabled:

- ~~6. Place approach~~ - Move to position above place location
- ~~7. Place~~ - Move down to place position
- ~~8. Release gripper~~ - Open gripper to release object
- ~~9. Place retreat~~ - Move up after releasing
- ~~10. Return home~~ - Return to home position

## Testing the Pick-Only Sequence

### Launch and Test

```bash
# Terminal 1: Launch action servers
ros2 launch mtc_pipeline modular_action_servers.launch.py robot_ip:=192.168.1.101

# Terminal 2: Execute pick-only test
source install/setup.bash
ros2 run mtc_pipeline vision_pick_predefined_place.py 3
```

### Expected Output

```
[vision_pick_place_action_server]: Tag 3 detected at [0.352, 0.124, 0.051]
[vision_pick_place_action_server]: Pick poses computed:
[vision_pick_place_action_server]:   Grasp:    [0.352, 0.124, 0.051]
[vision_pick_place_action_server]:   Approach: [0.352, 0.124, 0.151] (0.100m above)
[vision_pick_place_action_server]:   Retreat:  [0.352, 0.124, 0.201] (0.150m above)
[vision_pick_place_action_server]: Building MTC task with PICK-ONLY sequence (5 stages)
[vision_pick_place_action_server]: PLACE SEQUENCE TEMPORARILY DISABLED FOR TESTING

Task stages:
  1  - ←   1 →   -  1 / current state
  -  1 →   5 →   -  5 / open gripper           ✓
  -  5 →  12 →   - 60 / pick approach          ✓ (Pipeline planner)
  - 12 →   8 →   - 96 / grasp                  ✓ (Cartesian - straight down)
  -  8 →   6 →   - 48 / close gripper          ✓
  -  6 →  15 →   - 90 / pick retreat           ✓ (Cartesian - straight up)
```

### Success Criteria

✅ All 5 stages show solutions (not 0/0/0)
✅ Robot moves smoothly through sequence
✅ Object is successfully grasped
✅ Robot ends at retreat position holding object

## Re-enabling Place Sequence

Once pick testing is successful, uncomment the place stages in:
`src/mtc_pipeline/src/vision_pick_place_stages.cpp`

Find this section (around line 268):
```cpp
// TODO: Uncomment after pick testing is complete
/*
// 6. Move to place approach...
```

Remove the `/*` at the start and `*/` at the end to re-enable the place sequence.

Then rebuild:
```bash
colcon build --packages-select mtc_pipeline
```

## Troubleshooting Pick-Only Mode

### "Pick approach" fails (0 solutions)
- **Cause:** Start position too far or obstacles in path
- **Fix:** Move robot closer to object manually first, or clear obstacles

### "Grasp" fails (0 solutions)
- **Cause:** Straight line from approach to grasp is blocked or too long
- **Fix:** Increase approach_offset to give more clearance
  ```bash
  # Try with 15cm approach instead of 10cm
  # Modify the script or use custom offsets
  ```

### "Pick retreat" fails (0 solutions)
- **Cause:** Cannot move straight up from grasp position
- **Fix:**
  - Check for obstacles above object
  - Verify retreat height is reasonable
  - May need to increase retreat_offset

### Object not grasped properly
- **Issue:** Gripper closes but doesn't hold object
- **Causes:**
  1. Grasp offset is incorrect (gripper not centered on object)
  2. Object too small/large for gripper
  3. Object slippery or gripper needs more force
- **Fixes:**
  1. Adjust grasp_offset_json in the goal
  2. Ensure object size matches gripper capacity
  3. Check gripper force settings

### Robot holds object but trembles/shakes
- **Cause:** Path constraints or velocity scaling too aggressive
- **Fix:** Already set to 20% speed for stability
  - If still shaking, check wrist constraints
  - May need to adjust object weight in planning scene

## Performance Notes

**Current Speed:** 20% of maximum (conservative for testing)
- Safe for initial testing
- Good for debugging grasp positions
- Can increase later after validation

**Planner Selection:**
- **Pick approach:** Pipeline (OMPL) - handles obstacles, flexible paths
- **Grasp & Retreat:** Cartesian - straight lines for precision

This hybrid approach gives best results for vision-based picking!