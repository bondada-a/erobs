# Vision-Based Movement - TODO

## Current Limitation

`createRelativeMoveStage()` always moves the **flange**, not the actual **tool TCP**.

This works fine for:
- Tool exchange operations (flange needs to align with holders)
- Manual relative moves

This breaks for:
- Vision-based movements (AprilTag/ArUco detection)
- "Go to detected object" tasks

## The Problem

```
AprilTag detected. Command: "Move vacuum TCP forward 0.05m"

Current: Flange moves 0.05m → TCP overshoots by ~0.15m (tool offset)
Needed:  TCP moves 0.05m → MoveIt calculates flange motion automatically
```

## Solution (Implement when adding camera)

**1. Add parameter to `createRelativeMoveStage()` in base_stages.cpp:296:**
```cpp
const std::string& ik_frame_link = ""  // New optional parameter
```

**2. Set ik_frame property if provided:**
```cpp
if (!ik_frame_link.empty()) {
  geometry_msgs::msg::PoseStamped ik_pose;
  ik_pose.header.frame_id = ik_frame_link;
  ik_pose.pose.orientation.w = 1.0;
  stage->properties().set("ik_frame", ik_pose);
}
```

**3. Tool TCP links (from URDFs):**
- Hand-E: `robotiq_hande_end`
- EPick: `epick_tcp`
- Camera: `optical_frame`

**4. Usage in vision code:**
```cpp
createRelativeMoveStage("approach", "forward", 0.05, planner, "", "epick_tcp");
```

No offset calculations needed - MoveIt handles everything from URDF.

## Related Files
- `src/mtc_pipeline/src/base_stages.cpp:296` - Implementation location
- `src/mtc_pipeline/src/moveto_stages.cpp:55` - General relative moves (needs fix)
- `src/mtc_pipeline/src/tool_exchange_stages.cpp:45` - Tool exchange (keep as-is)
