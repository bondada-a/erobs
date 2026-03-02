# MCP Architecture Design Notes

**Date**: 2026-03-02
**Context**: After successfully setting up ros-mcp-server and demonstrating natural language → robot control, we analyzed the architectural options and planned next steps.

---

## 1. MCP vs VLA — Analysis for EROBS

### MCP Approach (current, working)
```
Natural Language → LLM (Claude) → MCP Tool Calls → rosbridge → ROS2 Action Servers → MoveIt → Robot
```
- LLM operates at **task level** — selects actions, constructs goal JSON, dispatches
- All motion planning/control handled by existing MoveIt/MTC pipeline
- Zero training data — works with context docs
- Interpretable, safe (collision checking), uses existing infrastructure

### VLA Approach (Vision-Language-Action models)
```
Natural Language + Camera Image → VLA Model → Raw Joint Actions → Robot
```
- Single neural network, **motor level** — outputs joint velocities at 5-10Hz
- End-to-end: perception + planning + control in one model
- Needs hundreds-thousands of demonstrations on THIS robot/setup
- Black box, no explicit collision checking

### Key Finding: MCP + Better Context Covers Most VLA Advantages

The blue ball demo proved Claude can already handle **novel tasks** by writing code on the fly:
- Wrote HSV detection on Zivid imagery
- Read organized point cloud for 3D position
- Used TF for camera→base frame transform
- Computed flange-frame moves from base-frame deltas
- Iterative visual servoing (capture→reason→move loop)

None of this was a predefined action server — the LLM improvised the entire pipeline.

| Original "MCP weakness" | Does richer context fix it? |
|---|---|
| Only predefined tasks | **Yes** — LLM writes novel perception + motion code |
| No spatial intuition | **Mostly** — with point cloud + TF + planning scene, symbolic geometry reasoning works |
| Can't improvise grasps | **Partially** — see sensor augmentation below |
| Limited to known objects | **Yes** — HSV, contour detection, YOLO can all be code-generated |

### Remaining Gap: Reactive Closed-Loop Control

The one genuine VLA advantage: continuous Hz-rate control for the last few cm of grasping. MCP's capture-reason-move loop has ~2-5s latency per iteration.

**But**: for beamline work, samples are static. Discrete verification checkpoints are sufficient.

---

## 2. Sensor Augmentation — Closing the Reactive Gap

Adding sensors as ROS topics lets Claude insert verification checkpoints:

```
Current:  capture → detect → move → grasp (open loop)
With sensors: capture → detect → move → grasp → CHECK → retry/adjust
```

| Sensor | ROS Topic | Usage |
|---|---|---|
| Force/Torque (wrist) | `/ft_sensor/wrench` | After close → check force > threshold → confirms grasp |
| Contact sensor (fingertip) | `/gripper/contact` | During approach → stop on contact instead of hardcoded z |
| Visual verification | `/color/image_color` | Re-capture after grasp → object gone = success |
| Gripper position feedback | `/joint_states` (gripper) | After close → fully closed = missed, partial = holding |

**Note**: Hand-E gripper position is already available in `/joint_states`. This can be used immediately for grasp verification without new hardware.

Pattern is always: **subscribe → check condition → branch**. Discrete event checking, not continuous control.

---

## 3. Latency Analysis

### Per-action cycle breakdown
```
LLM reasoning + tool call:    ~3-10s  (model dependent, the bottleneck)
MCP server processing:        ~50ms
rosbridge WebSocket:           ~10ms
ROS2 action + MoveIt planning: ~1-3s
Robot execution:               ~2-10s
                       Total:  ~6-23s per step
```

### Model choice for different task types
| Model | Latency | Use for |
|---|---|---|
| Haiku | ~1-2s | Routine dispatch (move, pick/place with known poses) |
| Sonnet | ~2-5s | Moderate reasoning (construct complex JSON, parse sensor data) |
| Opus | ~5-15s | Novel tasks (write perception code, compute transforms) |

### Is latency a problem for EROBS?
No — synchrotron measurements take 30s-5min per sample. LLM decision overhead of 3-10s is within the noise. The orchestrator's batching system further mitigates by planning multiple steps together.

---

## 4. Hybrid Architecture Concept

```
┌─────────────────────────────────────────────────────┐
│  Natural Language / Bluesky Adaptive                │
│  "Load sample A3 onto the hotplate"                 │
└──────────────────────┬──────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────┐
│  LLM (Claude via MCP)           ← TASK PLANNER      │
│  Decomposes into: tool_exchange → move → pick →     │
│  transport → place → move_home                      │
│  Also: novel perception, error recovery             │
└──────────────────────┬──────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────┐
│  Orchestrator (existing)        ← TASK DISPATCH      │
│  Sequences tasks, manages grippers, batches stages  │
└───────┬──────────────┬─────────────────┬────────────┘
        ▼              ▼                 ▼
┌──────────────┐ ┌───────────┐ ┌──────────────────┐
│ MoveIt/MTC   │ │ Sensor    │ │ Tool Exchange /  │
│ Transport,   │ │ Feedback  │ │ Gripper Control  │
│ collision-   │ │ Grasp     │ │ (existing)       │
│ safe paths   │ │ verify    │ │                  │
│ ← MOTION     │ │ ← CHECK   │ │ ← MECHANISM      │
└──────────────┘ └───────────┘ └──────────────────┘
```

Each layer does what it's best at:
- **LLM**: Intent understanding, task sequencing, error recovery, novel perception
- **MoveIt**: Collision-free transport — deterministic, safe
- **Sensors**: Verification checkpoints — grasp confirmation, contact detection
- **Existing infra**: Tool exchange, gripper drivers, MoveIt lifecycle — unchanged

---

## 5. Custom MCP Server Plan — erobs-mcp-server

### Decision: Separate server, not fork

Keep upstream ros-mcp-server for generic ROS tools. Write a separate `erobs-mcp-server` for EROBS-specific tools. Both run simultaneously via `.mcp.json`:

```json
{
  "mcpServers": {
    "ros": {"command": "uvx", "args": ["ros-mcp", "--transport=stdio"]},
    "erobs": {"command": "python3", "args": ["src/beambot/mcp/erobs_mcp_server.py"]}
  }
}
```

**Benefits**: No merge conflicts with upstream, custom tools version-controlled with beambot, clean separation of concerns.

### Proposed custom tools

| Tool | Replaces | Purpose |
|---|---|---|
| `capture_zivid_3d()` | 20-line QoS workaround script | Handles single-shot timing permanently |
| `capture_zivid_2d()` | Similar workaround | 2D image capture |
| `detect_object(method, params)` | HSV/contour/YOLO scripts | Standardized detection, consistent output |
| `get_object_pose_3d(pixel_x, pixel_y)` | Point cloud indexing + TF | Organized pointcloud → base frame position |
| `get_planning_scene()` | No current equivalent | Collision objects, attached objects for reasoning |
| `beambot_execute(task_json)` | Manual JSON string construction | Validates fields, handles serialization |
| `get_gripper_state()` | Subscribe + parse joint_states | Quick check: holding something or empty? |
| `get_tf_transform(source, target)` | TF lookup scripts | Any frame-to-frame transform |

### Perception context topics (subscribe on demand)

These aren't custom tools but should be documented for Claude to use via generic `subscribe_once`:

| Topic | Content | When to use |
|---|---|---|
| `/joint_states` | Current 6-DOF + gripper | Check configuration, grasp verification |
| `/planning_scene` | Collision objects in MoveIt | Reason about reachability |
| `/tf` | Full transform tree | Coordinate transforms |
| `/beambot/execution_state` | IDLE/RUNNING/PAUSED | Check orchestrator state |

---

## 6. Context Document Plan

**Status**: Plan written, awaiting implementation.
**Plan file**: `~/.claude/plans/kind-watching-sonnet.md`

Two files:
1. **CLAUDE.md section** (~45 lines) — action name→type table, MCP usage pattern, critical gotchas
2. **`docs/mcp_ros_reference.md`** (~200 lines) — detailed per-task-type fields, timeouts, gripper config

---

## 7. Implementation Priority

```
Phase 1 (Now):     Context document (CLAUDE.md + docs/mcp_ros_reference.md)
                   → Makes current MCP setup more reliable and efficient

Phase 2 (Soon):    erobs-mcp-server with capture_zivid_3d + detect_object
                   → Eliminates the Python script workarounds

Phase 3 (Later):   Additional tools (get_planning_scene, get_gripper_state)
                   → Richer context for novel tasks

Phase 4 (Future):  Sensor integration (force/contact feedback)
                   → Closes the reactive grasping gap

Phase 5 (If needed): VLA for specific subtasks
                   → Only if MCP + sensors proves insufficient for some task class
```

---

## 8. Error Observability for MCP — Making Failures Diagnosable

**Problem**: When an action fails, the orchestrator returns generic messages like `"Batch execution failed at step 1"`. The actual reason (collision, IK failure, joint limits, controller timeout) is logged to the node logger but isn't accessible to Claude. This forces guessing instead of diagnosing.

**Goal**: Every failure should be diagnosable by Claude without a human reading RViz or terminal logs. The LLM should be able to answer "why did that fail?" and decide what to do next.

### Areas to improve

#### A. Richer error messages in action results

Currently `base_stages.py:load_plan_execute()` logs the MoveIt error code but only returns `True/False` up to the orchestrator. The error code and reason should propagate all the way to the `MTCExecution.Result.error_message`.

**Files**: `base_stages.py`, `orchestrator.py`, all `*_stages.py`

| Current | Target |
|---|---|
| `"Batch execution failed at step 1"` | `"moveto failed: Planning failed - GOAL_IN_COLLISION (error code: -31)"` |
| `"vision_moveto step failed"` | `"vision_moveto failed: No markers detected after 3 attempts (tag_id=5)"` |
| `"Failed to initialize MoveIt stack"` | `"MoveIt launch failed: timed out after 30s waiting for move_group"` |

Changes needed:
- `base_stages.py:load_plan_execute()` → return error string instead of bool
- Each `*_stages.py` → propagate specific failure reason (planning failed, IK failed, execution failed + error code)
- `orchestrator.py:_execute_batch()` / `_execute_step()` → pass error string to result message
- Vision stages → include detection failure details (no markers found, timeout, camera error)
- Tool exchange → include which phase failed (approach, dock, retract, MoveIt restart)

#### B. Pre-flight checks before execution

Query the system state before sending a goal to prevent predictable failures:

| Check | How | Prevents |
|---|---|---|
| Current state in collision? | Service call to `/check_state_validity` or read `/planning_scene` | "Why did my moveto fail?" — because start state was already in collision |
| Controllers active? | Check `/controller_manager/list_controllers` | "Execution failed" — because controllers were stopped by UR driver |
| MoveIt running? | Check if `/move_group` node exists via `get_nodes()` | "Failed to initialize MoveIt" — it wasn't launched yet |
| Gripper matches config? | Compare physical gripper vs MoveIt config | Wrong collision model loaded |

These can be implemented as:
- Custom MCP tools in `erobs-mcp-server` (Phase 2)
- Or context instructions for Claude to check manually via existing MCP tools (immediate)

#### C. Post-failure diagnostics

When an action fails, Claude should be able to query what went wrong:

| Diagnostic | Method |
|---|---|
| Read recent errors | Subscribe to `/rosout` filtered by logger name (e.g., `beambot_orchestrator`, `move_group`) |
| Check planning scene | `/get_planning_scene` service — see collision objects, ACM, attached objects |
| Check joint state validity | `/check_state_validity` service with current joint state |
| Check controller state | `/controller_manager/list_controllers` service |

#### D. Structured error taxonomy

Define error categories so Claude can take appropriate corrective action:

| Error Category | Typical Cause | Corrective Action |
|---|---|---|
| `GOAL_IN_COLLISION` | Target pose collides with obstacle | Try different pose, clear octomap, check planning scene |
| `START_IN_COLLISION` | Current state is in collision (stale planning scene) | Clear planning scene, re-capture point cloud |
| `PLANNING_FAILED` | No path found (workspace limits, obstacles) | Try joint planner, shorter distance, different approach |
| `IK_FAILED` | No IK solution at target | Target may be outside workspace, try nearby pose |
| `EXECUTION_FAILED` | Controller error during motion | Check UR driver, controller state, e-stop |
| `TIMEOUT` | Action server didn't respond | Check if server is running, rosbridge connected |
| `DETECTION_FAILED` | Vision couldn't find target | Check lighting, camera connection, marker visibility |
| `GRIPPER_MISMATCH` | Wrong MoveIt config for physical gripper | Ask user, restart with correct gripper |

### Implementation phases

```
Phase 1a (Now):    Context doc says "ask user for gripper" ← DONE
Phase 1b (Soon):   Improve error messages in base_stages.py + orchestrator.py
                   → ~30 min, high impact, no interface changes
Phase 2a:          Add pre-flight check instructions to context doc
                   → Claude checks /rosout and planning scene manually via MCP
Phase 2b:          Build pre-flight checks into erobs-mcp-server
                   → Automated, reliable, fast
Phase 3:           Add /beambot/current_gripper topic to orchestrator
                   → Eliminates "ask user" workaround
```

---

## 9. Key Decisions Made

1. **MCP over VLA** for core EROBS workflow — structured environment, known objects, safety requirements
2. **Separate MCP server** over fork — cleaner architecture, no upstream conflicts
3. **Tiered model approach** — fast models for routine dispatch, Opus for novel reasoning
4. **Sensor augmentation** over VLA for reactive control — discrete checkpoints sufficient for static samples
5. **VLA deferred** to Phase 5 — only if specific task classes can't be handled by MCP + sensors
