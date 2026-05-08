# System Design Analysis
*Date: 2026-03-06 | Branch: humble-experimental*

## Current Architecture

```
                    ┌─────────────────────────────────────┐
                    │         INTERACTION LAYER            │
                    │                                      │
                    │  Claude Code ←→ MCP Servers          │
                    │    ├── ros-mcp-server (generic ROS2) │
                    │    └── erobs MCP (Zivid/detection)   │
                    │                                      │
                    │  GUI Client (mtc_gui)                │
                    │  CLI Client (beambot_client.py)      │
                    │  Bluesky (mtc_ophyd_device.py)       │
                    └────────────┬────────────────────────┘
                                 │ JSON task scripts
                                 ▼
                    ┌─────────────────────────────────────┐
                    │      ORCHESTRATION LAYER             │
                    │                                      │
                    │  MTCOrchestratorServer               │
                    │    ├── Task parsing & validation     │
                    │    ├── Batch planning                │
                    │    ├── MoveIt lifecycle management   │
                    │    ├── Pause/Resume                  │
                    │    └── Controller recovery           │
                    └────────────┬────────────────────────┘
                                 │ Action goals
                                 ▼
                    ┌─────────────────────────────────────┐
                    │      ACTION SERVER LAYER             │
                    │                                      │
                    │  MoveTo, EndEffector, PickPlace      │
                    │  ToolExchange, Vision, Pipettor      │
                    │  VisionPickPlace                     │
                    └────────────┬────────────────────────┘
                                 │ MTC stages
                                 ▼
                    ┌─────────────────────────────────────┐
                    │      PLANNING & EXECUTION            │
                    │                                      │
                    │  MoveIt Task Constructor              │
                    │    ├── OMPL (joint planning)         │
                    │    ├── CartesianPath (linear moves)  │
                    │    └── JointInterpolation (grippers) │
                    └────────────┬────────────────────────┘
                                 │ Trajectories
                                 ▼
                    ┌─────────────────────────────────────┐
                    │      HARDWARE LAYER                  │
                    │                                      │
                    │  UR5e Driver (ros2_control)          │
                    │  Zivid Camera (zivid_camera)         │
                    │  HandE / ePick / Pipettor drivers    │
                    └─────────────────────────────────────┘
```

---

## Strengths

### 1. Clean Separation of Concerns
The 3-tier architecture (Orchestrator → Action Servers → Stages) cleanly separates coordination, execution, and planning. Each layer has a well-defined responsibility.

### 2. Beamline-Agnostic Design
The beamline config system (`default_beamline.yaml`) makes it straightforward to adapt to different beamlines. Grippers, robot IP, camera settings are all configurable without code changes.

### 3. MCP Integration
The dual MCP server approach (generic ros-mcp-server + custom erobs server) is architecturally sound. It provides:
- Full ROS2 topic/service/action access via ros-mcp-server
- Optimized vision workflow via erobs server (handles Zivid QoS quirks)

### 4. Robust Task Format
The JSON task format is flexible, supports 8 task types, and enables both simple and complex sequences. Batching optimization reduces overhead.

### 5. Vision Pipeline
The camera abstraction (factory pattern), multi-method detection (ArUco, circle, contour, HSV), and multi-position scanning with cache are well-designed.

---

## Bottlenecks

### 1. MoveIt Restart on Gripper Change (~45s)
**Impact**: HIGH - Every tool exchange requires killing and relaunching MoveIt with new URDF/SRDF.

**Root Cause**: Each gripper has different URDF (different links, joints, collision geometries) requiring separate MoveIt configs. MoveIt2 doesn't support hot-swapping robot descriptions.

**Mitigation Ideas**:
- Pre-warm MoveIt configs (keep multiple move_group instances ready)
- Use a unified URDF with all grippers and enable/disable collision objects
- Use MoveIt's planning scene to attach/detach gripper models dynamically

### 2. MCP Server Cold Start
**Impact**: MEDIUM - ROS2 bridge initializes lazily on first tool call.

**Root Cause**: Correct design choice (MCP server starts fast), but first capture takes extra seconds for ROS2 init + subscription discovery.

**Mitigation**: Could pre-warm the bridge on server start with a background initialization thread.

### 3. Point Cloud Transfer Time (3-4s)
**Impact**: MEDIUM - Zivid point cloud is ~40MB, takes 3-4s to transmit via ROS2.

**Root Cause**: Zivid 2+ produces 2448x2048 points with RGBA at 16 bytes/point. ROS2 DDS isn't optimized for large single messages.

**Mitigation Ideas**:
- Only request point cloud when 3D positions are needed (already done with mode parameter)
- Use shared memory transport (iceoryx) for intra-container communication
- Subsample the point cloud before transfer

### 4. Sequential Task Execution
**Impact**: LOW-MEDIUM - Tasks execute strictly sequentially. Cannot overlap vision processing with robot motion.

**Root Cause**: MTC task execution is blocking. The orchestrator waits for each task to complete before starting the next.

**Mitigation Ideas**:
- Pipeline: Start vision processing during approach motion (non-trivial with current MTC architecture)
- Pre-fetch: Issue vision scan during previous task's retreat phase

### 5. spin_until_future_complete in Action Servers
**Impact**: LOW - Some vision stages use `rclpy.spin_until_future_complete()` which can conflict with the MultiThreadedExecutor.

**Root Cause**: `zivid.py` detection functions use `spin_until_future_complete` for service calls. The MCP server correctly uses a different pattern (background executor + threading events).

**Mitigation**: Migrate `zivid.py` to use the same async pattern as `erobs_mcp_server.py`.

---

## How Interaction Can Be Improved

### Current MCP Limitations

1. **No task execution tool**: Claude can see the camera and read TF, but can't command robot motion through the erobs MCP server. Must use ros-mcp-server's `call_action` which requires manual JSON construction.

2. **No state awareness**: Claude can't easily check what the robot is doing (executing? paused? what gripper is attached?).

3. **No experiment context**: Claude doesn't know about beamline-specific context (sample locations, experiment protocols).

### Proposed MCP Tool Additions

```
erobs MCP Server (current):
  ├── ping              ✅ exists
  ├── capture_image     ✅ exists
  ├── detect_objects    ✅ exists
  └── get_tf_transform  ✅ exists

erobs MCP Server (proposed additions):
  ├── execute_task       ← Send JSON task to orchestrator
  ├── get_robot_state    ← Joint positions, gripper, status
  ├── get_system_health  ← Driver/camera/controller status
  ├── pause_execution    ← Pause current task
  ├── resume_execution   ← Resume paused task
  ├── cancel_execution   ← Cancel current task
  ├── list_poses         ← Available named poses
  └── teach_pose         ← Save current position as named pose
```

### Higher-Level Abstraction

Beyond individual MCP tools, Claude would benefit from a "beamline scientist" abstraction:

```python
# Instead of Claude constructing raw JSON:
{
  "start_gripper": "epick",
  "tasks": [
    {"task_type": "vision_scan", "scan_positions": ["scan_1", "scan_2"]},
    {"task_type": "vision_pick_place", "detection_type": "marker", "tag_id": 5, ...}
  ],
  "poses": {...}
}

# Claude should be able to say:
execute_task(action="pick_sample", sample_id="5", destination="hotplate")
```

This higher-level API would map scientist-level intent to the correct task sequences, handling:
- Which gripper is needed
- Tool exchange if necessary
- Vision scan before pick
- Correct approach/retreat poses
- Error recovery

---

## Deployment Architecture Analysis

### Current: Single Machine
```
Developer Machine (GPU)
  ├── Docker: erobs-common-img (ROS2, MoveIt, Zivid SDK)
  ├── Docker: bsui (optional Bluesky)
  └── Physical: UR5e + Zivid + Grippers
```

### Target: Beamline Integration
```
Beamline Network          │  Robot Network
                          │
Bluesky RunEngine ────────┼──► EROBS Container
  (existing infra)   API  │      ├── Orchestrator
                    bridge │      ├── Action Servers
                          │      ├── MoveIt
                          │      └── Zivid SDK
                          │
Claude Code ──────── MCP ─┼──► erobs_mcp_server (in container)
  (operator tool)         │
```

**Key Insight**: The MCP approach may actually simplify the beamline integration story. Instead of building a complex Bluesky bridge (the DSSI approach), the MCP server provides a natural interface for AI-driven experiment control. Claude Code can:
1. Observe samples via vision
2. Decide what to do via reasoning
3. Execute via MCP tools
4. Monitor results

This is closer to the "autonomous beamline scientist" vision than the traditional Bluesky integration path.

---

## Recommendations

### Short-term (1-2 weeks)
1. Add `execute_task` and `get_robot_state` MCP tools
2. Extract shared detection module
3. Update documentation

### Medium-term (1-2 months)
4. Add higher-level experiment tools to MCP server
5. Investigate unified URDF to avoid MoveIt restart
6. Add execution logging

### Long-term (3+ months)
7. Pipeline vision processing with robot motion
8. Implement adaptive experiment planning (Claude suggests next sample)
9. Multi-beamline deployment with shared framework
