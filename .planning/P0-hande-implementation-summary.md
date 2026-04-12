# P0 Hand-E Grasp Detection — Implementation Summary

## What Was Implemented

Two complementary layers for detecting whether the Hand-E gripper is holding an object:

### 1. MCP Tools (`beambot_mcp_server.py`)

- **`get_gripper_state()`** — Returns detailed gripper status (position in m/mm, `object_detected` boolean). Delegates to `get_vacuum_status()` when ePick is attached, so callers don't need to know which gripper is active.
- **`check_grasp()`** — Quick unified go/no-go check returning `object_detected` + `method` for any gripper type. Intended as a fast post-grasp verification step.

Both tools read `hande_position` from the `ROS2BridgeNode`, which is updated by a `/joint_states` subscription that extracts the `robotiq_hande_left_finger_joint` value.

### 2. Orchestrator Watchdog (`orchestrator.py`)

A real-time grasp monitor that runs during task execution:

1. **ARM** — After an `hande_closed` end-effector action, the watchdog sets `_hande_grasp_armed = True` and does an immediate position check.
2. **MONITOR** — The `/joint_states` callback continuously checks finger position while armed. If position drops below `HANDE_CLOSED_THRESHOLD` (0.002 m), it sets `_hande_grasp_lost = True`.
3. **CHECK** — Between every task step, `_check_grasp_lost()` inspects the flag. If set, it aborts execution with a `GRASP_LOST` error message.
4. **DISARM** — An `hande_open` action resets both flags.

Task batching is automatically disabled when Hand-E is attached so the watchdog can check between every move.

## Key Design Decisions

| Decision | Rationale |
|---|---|
| **Finger-position threshold (0.002 m)** | Below this the fingers are fully closed with no object. Simple and reliable — no extra hardware needed. |
| **Shared `_check_grasp_lost()` for ePick + Hand-E** | Single checkpoint in the execution loop handles both gripper types, reducing code duplication. |
| **Immediate check after close** | Catches failed picks before the robot starts moving, avoiding unnecessary transport of nothing. |
| **No off-on cycle for retry** | Unlike ePick, Hand-E just needs a re-close to retry. This is documented in the error message and CLAUDE.md recovery table. |
| **Batching disabled for Hand-E** | The watchdog needs step boundaries to detect drops mid-sequence. Same pattern already used for ePick. |

## How to Test

### MCP tool verification (robot running, Hand-E attached)

1. **Close on object** → call `check_grasp()` → expect `object_detected: true`
2. **Close on nothing** → call `check_grasp()` → expect `object_detected: false`
3. **Detailed state** → call `get_gripper_state()` → verify `position_mm` is reasonable (~0 closed, ~25 open)

### Watchdog verification

1. **Successful pick-and-place** — Run a `pick_and_place` task through `/beambot_execution`. Should complete normally with "Hand-E grasp monitor ARMED" in logs.
2. **Failed pick** — Close Hand-E on empty air, then send a `moveto`. The watchdog should abort with `GRASP_LOST` before/after the move.
3. **Drop during transport** — Pick an object, then manually pull it out mid-move. Next step boundary should trigger `GRASP_LOST`.

### Edge cases

- Call `get_gripper_state()` before system is running → should return `available: false` gracefully.
- Switch to ePick → `check_grasp()` should delegate to vacuum detection automatically.
