# MCP ↔ ROS2 Action Reference

Detailed field reference for all beambot action interfaces. This document is the authoritative source for building task JSON scripts sent via the MCP → orchestrator pipeline.

**Primary interface**: All tasks go through `/beambot_execution` (`MTCExecution` action). The orchestrator parses the JSON, manages gripper state, launches MoveIt, and dispatches to individual action servers.

---

## Table of Contents

1. [Task JSON Structure](#1-task-json-structure)
2. [moveto](#2-moveto)
3. [end_effector](#3-end_effector)
4. [pick_sample](#4-pick_sample)
5. [place_sample](#5-place_sample)
6. [tool_exchange](#6-tool_exchange)
7. [vision_moveto](#7-vision_moveto)
8. [vision_scan](#8-vision_scan)
9. [pipettor](#9-pipettor)
10. [Gripper Configuration](#10-gripper-configuration)
11. [Timeouts](#11-timeouts)
12. [Common Gotchas](#12-common-gotchas)

---

## 1. Task JSON Structure

Every task script sent to `/beambot_execution` has this top-level structure:

```json
{
  "start_gripper": "<gripper_name>",
  "tasks": [ ... ],
  "poses": { "<pose_name>": [j1, j2, j3, j4, j5, j6], ... }
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `start_gripper` | string | **Yes** | Gripper physically attached at start. Must match reality — no auto-detection. Valid: `"hande"`, `"epick"`, `"pipettor"`, `"none"` |
| `tasks` | array | **Yes** | Ordered list of task steps to execute sequentially |
| `poses` | object | **Yes** | Map of pose names → 6-element joint arrays. **Values are in degrees** (converted to radians internally). Referenced by name in task steps |

### MCP Send Pattern

```
send_action_goal(
  action_name="/beambot_execution",
  action_type="beambot_interfaces/action/MTCExecution",
  goal={"full_json": "<serialized JSON string>"}
)
```

The `full_json` value must be a **serialized JSON string**, not a nested object.

### Feedback Fields (published during execution)

| Field | Type | Description |
|-------|------|-------------|
| `current_step` | int32 | Current task index (0-based) |
| `current_action` | string | Task type being executed |
| `progress_percentage` | float32 | 0.0 – 100.0 |
| `status_message` | string | Human-readable status |
| `current_gripper` | string | Currently tracked gripper |

### Result Fields

| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | Whether all tasks completed |
| `error_message` | string | Error description if failed |
| `completed_steps` | int32 | Number of tasks completed before failure/success |
| `total_steps` | int32 | Total tasks in the script |

---

## 2. moveto

Move the robot arm to a target pose.

**Action**: `/beambot_moveto` (`beambot_interfaces/action/MoveToAction`)
**Timeout**: 120s

### Task JSON Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `task_type` | string | — | Must be `"moveto"` |
| `target` | string | `""` | Pose name (key in `poses`), SRDF state name, or empty for relative moves |
| `planning_type` | string | `"joint"` | `"joint"`, `"cartesian"`, `"pilz"` (Pilz LIN — deterministic straight-line), or `"pilz_ptp"` (Pilz PTP — predictable joint motion) |
| `direction` | string | `""` | For relative moves: `"forward"`, `"backward"`, `"left"`, `"right"`, `"up"`, `"down"` |
| `distance` | float | `0.0` | Distance in meters (for relative moves) |
| `cartesian_target` | float[] | `[]` | `[x,y,z]` or `[x,y,z,roll,pitch,yaw]` — meters + **degrees**. When 3 values: straight-down orientation. Auto-detects gripper tip frame |
| `frame_id` | string | `"base_link"` | Reference frame for `cartesian_target` |

### Three Move Modes

**Mode 1 — Named joint pose** (most common):
```json
{"task_type": "moveto", "target": "home", "planning_type": "joint"}
```

**Mode 2 — Cartesian target** (absolute position):
```json
{"task_type": "moveto", "cartesian_target": [0.3, -0.2, 0.15], "frame_id": "base_link"}
```
With explicit orientation (degrees):
```json
{"task_type": "moveto", "cartesian_target": [0.3, -0.2, 0.15, 180, 0, 0], "frame_id": "base_link"}
```

**Mode 3 — Relative move** (direction + distance):
```json
{"task_type": "moveto", "direction": "backward", "distance": 0.1, "planning_type": "cartesian"}
```

### Direction Vectors

Directions are in the **`flange` frame**, not world frame. At a typical downward-looking pose:

| Direction | Flange Frame Vector | World Effect (approx.) |
|-----------|-------------------|----------------------|
| `forward` | +Z | Down toward table |
| `backward` | -Z | Up away from table |
| `up` | -Y | Varies with wrist |
| `down` | +Y | Varies with wrist |
| `left` | -X | Varies with wrist |
| `right` | +X | Varies with wrist |

### Cartesian Target Notes

- When only `[x,y,z]` is given, orientation defaults to straight-down (180° around X)
- The planner auto-detects the gripper tip frame (`epick_tip`, `robotiq_hande_end`) via TF and places the **tip** at the target, not the flange
- `frame_id` orientation matters: use `flange` frame RPY (not `tool0` — see [Gotchas](#12-common-gotchas))

---

## 3. end_effector

Open or close a gripper.

**Action**: `/beambot_endeffector` (`beambot_interfaces/action/EndEffectorAction`)
**Timeout**: 30s

### Task JSON Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `task_type` | string | — | Must be `"end_effector"` |
| `end_effector_action` | string | `""` | SRDF group state name (see table below) |
| `end_effector_type` | string | current gripper | Gripper name override. Defaults to `start_gripper` / current gripper |

### SRDF State Names per Gripper

| Gripper | Open/Release | Close/Grasp |
|---------|-------------|-------------|
| `hande` | `"hande_open"` | `"hande_closed"` |
| `epick` | `"vacuum_off"` | `"vacuum_on"` |
| `pipettor` | (no gripper states) | (no gripper states) |

### Examples

Using current gripper (recommended):
```json
{"task_type": "end_effector", "end_effector_action": "vacuum_on"}
```

Explicit gripper override:
```json
{"task_type": "end_effector", "end_effector_type": "hande", "end_effector_action": "hande_open"}
```

### How It Works

The orchestrator resolves the gripper type → looks up `gripper_group` from beamline config → sends the MoveIt group name and SRDF state name to the action server. The action server executes a MoveIt group state change (for Hand-E: physical finger motion; for ePick: tool I/O voltage toggle).

---

## 4. pick_sample

Unified pick operation — supports hardcoded joint poses or vision-guided pickup with deterministic IK.

**Action**: `/beambot_pick_sample` (`beambot_interfaces/action/PickSampleAction`)
**Timeout**: 180s

### Task JSON Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `task_type` | string | — | Must be `"pick_sample"` |
| `use_vision` | bool | `true` | `true` = vision-guided, `false` = hardcoded poses |
| `detection_type` | string | `"marker"` | `"marker"`, `"circle"`, or `"contour"` (vision mode) |
| `tag_id` | int | `0` | ArUco marker ID (vision mode) |
| `sample_index` | int | `1` | Sample number for contour detection (1-indexed) |
| `z_offset` | float | config default | Height offset from detected point (meters) |
| `scan_pose` | string | `""` | Pose key — scan/retreat position (vision mode) |
| `marker_offset_x` | float | `0.0` | Marker-frame X offset (meters) |
| `marker_offset_y` | float | `0.0` | Marker-frame Y offset (meters) |
| `marker_offset_z` | float | `0.0` | Marker-frame Z offset (meters) |
| `offset_direction` | string | `""` | Flange-frame direction (e.g. `"right"`) |
| `offset_distance` | float | `0.0` | Distance for offset_direction (meters) |
| `approach_pose` | string | `""` | Pose key — approach position (hardcoded mode) |
| `target_pose` | string | `""` | Pose key — grasp target (hardcoded mode) |
| `gripper` | string | current gripper | Gripper override |
| `settle_time` | float | `1.0` | Seconds to wait for robot to settle (vision mode) |

### Execution Sequence (vision mode)

```
Task 1:  open gripper → move to scan_pose
         [settle_time wait → detect marker → compute approach → deterministic IK]
Task 2:  move to approach (PTP) → close gripper → retreat to scan_pose
         [check vacuum status]
```

### Execution Sequence (hardcoded mode)

```
Single task: open → approach_pose → target_pose → close → retreat to approach_pose
             [check vacuum status]
```

### Result Fields

| Field | Type | Description |
|-------|------|-------------|
| `vacuum_ok` | bool | `true` if ePick reports object detected (or non-ePick gripper) |
| `detected_position` | float[3] | `[x,y,z]` in base_link (vision mode only) |
| `detected_orientation` | float[4] | `[x,y,z,w]` quaternion (vision mode only) |

### Examples

```json
// Vision pick with marker offsets (from MCP detect_sample)
{"task_type": "pick_sample", "use_vision": true, "tag_id": 0,
 "scan_pose": "sample_scan_1", "marker_offset_x": 0.02159,
 "marker_offset_y": 0.00121, "z_offset": -0.001}

// Vision pick at tag center
{"task_type": "pick_sample", "use_vision": true, "tag_id": 30,
 "scan_pose": "hotplate_scan", "offset_direction": "right",
 "offset_distance": 0.0537, "z_offset": -0.001}

// Hardcoded pick (joint poses)
{"task_type": "pick_sample", "use_vision": false,
 "approach_pose": "pickup_approach", "target_pose": "pickup"}
```

---

## 5. place_sample

Unified place operation — supports hardcoded joint poses or vision-guided placement with deterministic IK.

**Action**: `/beambot_place_sample` (`beambot_interfaces/action/PlaceSampleAction`)
**Timeout**: 180s

### Task JSON Fields

Same fields as `pick_sample` except:
- No `sample_index` (contour detection not used for placement)
- No `vacuum_ok` in result

### Execution Sequence (vision mode)

```
Task 1:  move to scan_pose (NO gripper open — holding object)
         [settle_time wait → detect marker → compute approach → deterministic IK]
Task 2:  move to approach (PTP) → open gripper → retreat to scan_pose
```

### Execution Sequence (hardcoded mode)

```
Single task: approach_pose → target_pose → open → retreat to approach_pose
```

### Examples

```json
// Vision place on hotplate
{"task_type": "place_sample", "use_vision": true, "tag_id": 30,
 "scan_pose": "hotplate_scan", "offset_direction": "right",
 "offset_distance": 0.0533, "z_offset": -0.001}

// Hardcoded place
{"task_type": "place_sample", "use_vision": false,
 "approach_pose": "place_approach", "target_pose": "place"}
```

---

## 5. tool_exchange

Dock (detach) or load (attach) a gripper from/to the magnetic tool changer.

**Action**: `/beambot_toolexchange` (`beambot_interfaces/action/ToolExchangeAction`)
**Timeout**: 180s

### Task JSON Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `task_type` | string | — | Must be `"tool_exchange"` |
| `operation` | string | `""` | `"dock"` (detach gripper) or `"load"` (attach gripper) |
| `gripper` | string | `""` | Gripper being docked or loaded |
| `dock_number` | int | `0` | Physical dock position (1–5) |
| `approach_pose` | string | `""` | Pose key — approach position for the dock |

### Example: Swap epick → pipettor

```json
{
  "tasks": [
    {
      "task_type": "tool_exchange",
      "operation": "dock",
      "gripper": "epick",
      "dock_number": 3,
      "approach_pose": "dock_approach"
    },
    {
      "task_type": "tool_exchange",
      "operation": "load",
      "gripper": "pipettor",
      "dock_number": 4,
      "approach_pose": "load_approach"
    }
  ]
}
```

### What Happens After Tool Exchange

1. Physical docking/loading motion executes
2. Orchestrator updates `_current_gripper` based on operation:
   - `dock` → `"none"`
   - `load` → the loaded gripper name
3. If gripper changed: **MoveIt restarts** with the new gripper's MoveIt config package (~2–5s delay)
4. Subsequent tasks automatically use the new gripper

---

## 6. vision_moveto

Move to a vision-detected target (ArUco marker, circle, or contour).

**Action**: `/beambot_vision_moveto` (`beambot_interfaces/action/VisionMoveToAction`)
**Timeout**: 60s

### Task JSON Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `task_type` | string | — | Must be `"vision_moveto"` |
| `detection_type` | string | `"marker"` | Detection method: `"marker"`, `"circle"`, `"contour"` |
| `tag_id` | int | `0` | ArUco marker ID (when `detection_type` is `"marker"`) |
| `sample_index` | int | `1` | Which detected object to target (1-indexed, for circle/contour). Objects sorted left-to-right, top-to-bottom |
| `z_offset` | float | `0.0` | Height offset above detected point (meters). `0.0` = use gripper default |
| `timeout` | float | `10.0` | Detection timeout in seconds |
| `settle_time` | float | `1.0` | Seconds to wait before capture for robot vibrations to settle |
| `scan_positions` | string[] | `[]` | Pose keys for multi-position averaging (optional). Robot moves to each position, captures, and averages detections |

### Examples

**Single marker detection** (most common):
```json
{
  "task_type": "vision_moveto",
  "detection_type": "marker",
  "tag_id": 16,
  "timeout": 10.0
}
```

**Circle detection with index**:
```json
{
  "task_type": "vision_moveto",
  "detection_type": "circle",
  "sample_index": 2,
  "z_offset": 0.01
}
```

**Multi-position averaging** (higher accuracy):
```json
{
  "task_type": "vision_moveto",
  "detection_type": "marker",
  "tag_id": 5,
  "scan_positions": ["sample_scan_1", "sample_scan_2", "sample_scan_3"]
}
```

### How It Works

1. Orchestrator waits `settle_time` seconds (robot vibration dampening)
2. If `scan_positions` provided: converts pose keys → radians, sends as flat array
3. Action server triggers Zivid capture at current (or each scan) position
4. Detects target using specified method
5. Transforms detection to `base_link` frame using TF
6. Plans and executes Cartesian move to detected pose + z_offset

### Multi-Position Mode

When `scan_positions` is provided, the robot:
1. Moves to each scan position in order
2. Captures + detects at each position
3. Averages the detected poses
4. Moves to the averaged position

This reduces noise from single-capture detection errors.

---

## 7. vision_scan

Batch-scan all visible markers from multiple positions and cache their poses.

**Action**: `/beambot_vision_scan` (`beambot_interfaces/action/VisionScanAction`)
**Timeout**: 180s

### Task JSON Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `task_type` | string | — | Must be `"vision_scan"` |
| `scan_positions` | string[] | **required** | Pose keys — robot moves to each and captures |
| `scans_per_position` | int | `3` | Number of captures at each position (averaged) |
| `timeout` | float | `10.0` | Per-capture timeout in seconds |

### Example

```json
{
  "task_type": "vision_scan",
  "scan_positions": ["sample_scan_1", "sample_scan_2", "sample_scan_3"],
  "scans_per_position": 3,
  "timeout": 10.0
}
```

### Result Fields

| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | Whether scan completed |
| `error_message` | string | Error if failed |
| `tags_detected` | int32 | Number of unique markers found and cached |

### How It Works

1. Robot moves to each scan position sequentially
2. At each position: captures N times (`scans_per_position`), detects ALL visible markers
3. Averages detected poses across all captures and positions
4. Caches results — subsequent `vision_moveto` calls use cached poses instead of re-detecting

### When to Use

Use `vision_scan` **before** a series of `vision_moveto` calls when:
- You need to pick multiple samples in sequence (scan once, use cached poses for all picks)
- Accuracy matters (multi-position + multi-capture averaging)
- The camera can't see all markers from a single position

---

## 9. pipettor

Control the custom pipettor tool for liquid handling.

**Action**: `/beambot_pipettor` (`beambot_interfaces/action/PipettorAction`)
**Timeout**: 60s

### Task JSON Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `task_type` | string | — | Must be `"pipettor"` |
| `operation` | string | `""` | `"SUCK"`, `"EXPEL"`, `"EJECT_TIP"`, `"SET_LED"` |
| `volume_pct` | float | `0.0` | Volume as fraction 0.0–1.0 (for `SUCK`/`EXPEL`) |
| `led_color` | object | — | RGB color object (for `SET_LED` or status indication) |

### LED Color Object

```json
{"r": 0.0, "g": 1.0, "b": 0.0, "a": 1.0}
```

Values are floats 0.0–1.0. The `a` (alpha) field is always `1.0`.

### Examples

**Aspirate 50% volume**:
```json
{"task_type": "pipettor", "operation": "SUCK", "volume_pct": 0.5}
```

**Dispense with green LED**:
```json
{
  "task_type": "pipettor",
  "operation": "EXPEL",
  "volume_pct": 0.5,
  "led_color": {"r": 0.0, "g": 1.0, "b": 0.0, "a": 1.0}
}
```

**Eject tip**:
```json
{"task_type": "pipettor", "operation": "EJECT_TIP"}
```

### Typical Pipettor Workflow

```
moveto → safe_pipettor_tip       (safe height)
moveto → pre_pipettor_tip_1      (above tip rack)
moveto → pipettor_tip_1          (push onto tip)
moveto → pre_pipettor_tip_1      (retreat)
moveto → pre_pipettor_suck       (above vial)
moveto → pipettor_suck           (tip in liquid)
pipettor → SUCK 0.5              (aspirate)
moveto → pre_pipettor_suck       (retreat)
moveto → pipettor_dispense       (over target)
pipettor → EXPEL 0.5             (dispense)
moveto → tip_disposal            (over waste)
pipettor → EJECT_TIP             (drop tip)
```

---

## 10. Gripper Configuration

Gripper behavior is defined in the beamline config (`config/default_beamline.yaml`), not in task JSON.

### Available Grippers

| Name | MoveIt Package | MoveIt Group | Grasp State | Release State |
|------|---------------|--------------|-------------|---------------|
| `none` | `ur_standalone_moveit_config` | (none) | — | — |
| `hande` | `ur_zivid_hande_moveit_config` | `hande_gripper` | `hande_closed` | `hande_open` |
| `epick` | `ur_zivid_epick_moveit_config` | `epick_gripper` | `vacuum_on` | `vacuum_off` |
| `pipettor` | `ur_zivid_pipettor_moveit_config` | (none) | — | — |

### Gripper Auto-Resolution

Tasks that need gripper info (`end_effector`, `pick_sample`, `place_sample`) automatically use the current gripper if not specified. The orchestrator:
1. Starts with `start_gripper` from the task JSON
2. Updates after each `tool_exchange` operation
3. Injects `gripper_group` and `gripper_states_json` into action goals

### Gripper State at Startup

The orchestrator initializes with `_current_gripper = "none"`. The gripper is only known after the first `MTCExecution` goal sets `start_gripper`. There is **no auto-detection** — the user must declare the physically attached gripper. Between goals, the last known gripper state persists.

### Querying Current Gripper

The orchestrator publishes the current gripper to `/beambot/current_gripper` (`std_msgs/String`) with **transient local** (latched) QoS. Late subscribers immediately receive the last published value.

```bash
ros2 topic echo /beambot/current_gripper --once
```

Via MCP:
```
subscribe_once(topic="/beambot/current_gripper", msg_type="std_msgs/msg/String")
```

Values: `"none"` (startup, or after docking), `"hande"`, `"epick"`, `"pipettor"`. Updated on first goal and after each `tool_exchange`.

---

## 11. Timeouts

Default timeouts (configurable via ROS parameters `timeout.<action_type>`):

| Action Type | Default Timeout | ROS Parameter |
|-------------|----------------|---------------|
| `moveto` | 120s | `timeout.moveto` |
| `end_effector` | 30s | `timeout.end_effector` |
| `pick_sample` | 180s | `timeout.pick_sample` |
| `place_sample` | 180s | `timeout.place_sample` |
| `tool_exchange` | 180s | `timeout.tool_exchange` |
| `vision_moveto` | 60s | `timeout.vision_moveto` |
| `vision_scan` | 180s | `timeout.vision_scan` |
| `pipettor` | 60s | `timeout.pipettor` |

Override at launch:
```bash
ros2 launch beambot beambot_bringup.launch.py timeout.moveto:=60.0
```

---

## 12. Common Gotchas

### Nothing Is Running Before the First Goal
The beambot orchestrator launches MoveIt **lazily on the first goal**. Before that, there is no TF tree, no topics, and no services. Do NOT query TF transforms, subscribe to topics, or call services (via ros-mcp-server) before sending the first `/beambot_execution` goal — they will fail.

**Exception**: `get_robot_state` (beambot-mcp-server) is safe to call anytime. It reads from persistent subscriptions and returns `system_running: false` with `gripper: "unknown"` when nothing is up.

**Recommended workflow**: Call `get_robot_state` → if system is running, use the returned gripper → if not, ask the user → construct JSON → send goal.

### start_gripper Must Match Reality
Call `get_robot_state` first — if the system is already running, it returns the current gripper. If `gripper: "unknown"`, ask the user. Sending the wrong gripper loads the wrong MoveIt config and causes planning failures or dangerous motions.

### Joint Poses Are in Degrees
All joint arrays in the `poses` object are in **degrees**. The orchestrator converts to radians before sending to action servers. This matches what you see on the UR teach pendant.

### tool0 vs flange Frame
`tool0` and `flange` are at the same position but **rotated by (-90°, -90°, 0°)**. When querying current orientation for a `cartesian_target` move, use `flange` frame (MoveIt/ROS convention). Using `tool0` RPY will result in ~90° wrong orientation. Use `tool0` only for comparing with UR teach pendant values.

### Cartesian Planning May Fail for Long Moves
MTC `CartesianPath` uses incremental IK stepping (1mm steps). Long moves can fail due to singularities or joint limits. Use `"planning_type": "joint"` as fallback, or break into shorter Cartesian segments.

### MoveIt Restarts After Tool Exchange
After a `tool_exchange` that changes the gripper, MoveIt restarts with the new config. This takes ~2–5s. The orchestrator handles this automatically, but if sending goals manually (not through the orchestrator), you must wait.

### Direction Vectors Are in Flange Frame
Relative move directions (`forward`, `backward`, etc.) are in the flange coordinate frame, not the world frame. At a downward-looking pose, `"forward"` moves the robot **down** (toward the table), not forward in the world.

### Zivid Single-Shot Capture Cannot Use MCP subscribe_once
The Zivid camera publishes one point cloud per trigger. MCP's `subscribe_once` has a QoS timing race and will miss the message. Use the orchestrator's vision tasks, or the `beambot-mcp-server`'s `capture_image` tool which handles subscription timing correctly.

### settle_time Matters for Vision Accuracy
Robot vibrations after a move take ~0.5–2s to dampen. The `settle_time` field (default 1.0s for `vision_moveto`, `pick_sample`, `place_sample`) adds a wait before Zivid capture. Reduce for speed, increase for accuracy.

### Vision Scan Before Vision MoveTo
For best accuracy with multiple samples, run `vision_scan` first to batch-detect and cache all marker poses, then use `vision_moveto` which will use cached poses instead of re-detecting.

### Pipettor Has No Gripper States
The pipettor tool uses its own action server (`/beambot_pipettor`) for operations. It has no MoveIt gripper group or SRDF states. All motion for pipettor workflows is done via `moveto` tasks.
