# EROBS - MCP Operations Reference

> For architecture, development setup, TODOs, and design decisions, see [`docs/development.md`](docs/development.md).

## Task JSON Format

All tasks are sent as a JSON string to `/beambot_execution`. The top-level structure:

```json
{
  "start_gripper": "hande",
  "tasks": [
    {"task_type": "moveto", "target": "home"},
    {"task_type": "pick_sample", "use_vision": true, "tag_id": 5, "scan_pose": "scan_1"},
    {"task_type": "vision_moveto", "tag_id": 5}
  ],
  "poses": {"home": [0, -90, 90, -90, -90, 0]}
}
```
Note: Joint poses are in **degrees**, converted to radians internally.

### pick_sample Task Format
Unified pick operation — supports hardcoded poses or vision-guided pickup.
```json
// Vision-guided pick (detect marker, move with deterministic IK)
{"task_type": "pick_sample", "use_vision": true, "tag_id": 0,
 "scan_pose": "sample_scan_1", "z_offset": -0.001,
 "marker_offset_x": 0.02, "marker_offset_y": 0.001}

// Hardcoded pick (joint poses)
{"task_type": "pick_sample", "use_vision": false,
 "approach_pose": "pickup_approach", "target_pose": "pickup"}
```
- `use_vision`: `true` = detect marker then move, `false` = use joint poses directly
- Vision fields: `tag_id`, `scan_pose`, `z_offset`, `marker_offset_x/y/z`, `offset_direction`, `offset_distance`, `detection_type` ("marker"/"circle"/"contour")
- Hardcoded fields: `approach_pose`, `target_pose`
- Executes: open → scan → [detect] → approach (deterministic IK) → close → retreat → vacuum check
- Result includes `vacuum_ok` (ePick seal status) and `detected_position`
- For contour-based pickup: use MCP `detect_sample` first to get `marker_offset_x/y`, then pass them here

### place_sample Task Format
Unified place operation — supports hardcoded poses or vision-guided placement.
```json
// Vision-guided place
{"task_type": "place_sample", "use_vision": true, "tag_id": 30,
 "scan_pose": "hotplate_scan", "offset_direction": "right",
 "offset_distance": 0.0533, "z_offset": -0.001}

// Hardcoded place
{"task_type": "place_sample", "use_vision": false,
 "approach_pose": "place_approach", "target_pose": "place"}
```
- Same field structure as `pick_sample` but opens gripper instead of closing
- Does NOT open gripper before scanning (holding object)
- Executes: scan → [detect] → approach (deterministic IK) → open → retreat

### tool_exchange Task Format
```json
{"task_type": "tool_exchange", "operation": "dock", "gripper": "epick", "dock_number": 4, "approach_pose": "dock_approach"}
{"task_type": "tool_exchange", "operation": "load", "gripper": "pipettor", "dock_number": 2, "approach_pose": "load_approach"}
```
- `operation`: `"dock"` (put gripper away) or `"load"` (pick up gripper)
- `gripper`: Which gripper to dock/load
- `dock_number`: Physical dock slot number
- `approach_pose`: Joint pose name for approaching the dock
- `dock_number`: Physical dock slot. **Look up from `default_beamline.yaml` `grippers.<name>.dock_number`** — do NOT hardcode.
- **ALWAYS move to `safe_tool_exchange` BEFORE and AFTER any tool exchange operation.** This pose provides clearance for all gripper lengths (including pipettor). Move there before docking to avoid collisions on approach, and after loading to ensure safe departure.
- **ALWAYS use `"dock_approach"` for dock operations and `"load_approach"` for load operations.** These are different poses tuned for each operation direction. Using the wrong approach pose causes collisions or failed exchanges.

### pipettor Task Format
```json
{"task_type": "pipettor", "operation": "SUCK", "volume_pct": 0.5}
{"task_type": "pipettor", "operation": "EXPEL", "volume_pct": 0.5}
{"task_type": "pipettor", "operation": "EJECT_TIP"}
{"task_type": "pipettor", "operation": "SET_LED", "led_color": {"r": 1.0, "g": 0.0, "b": 0.0}}
```
- `operation`: `"SUCK"` (aspirate), `"EXPEL"` (dispense), `"EJECT_TIP"`, or `"SET_LED"`
- `volume_pct`: 0.0-1.0 for SUCK/EXPEL (fraction of full pipette volume)
- `led_color`: `{r, g, b}` floats 0.0-1.0 for SET_LED
- Requires `start_gripper: "pipettor"` and pipettor physically attached

### Gripper Auto-Detection
Tasks that need gripper info (`end_effector`, `pick_sample`, `place_sample`) default to the currently attached gripper if not specified. The orchestrator tracks `start_gripper` and updates it after `tool_exchange` operations.

**Simplified JSON** (uses current gripper):
```json
{"task_type": "end_effector", "end_effector_action": "close"}
{"task_type": "pick_sample", "use_vision": true, "tag_id": 0, "scan_pose": "scan_1"}
```

**Explicit override** (when you need a specific gripper):
```json
{"task_type": "end_effector", "end_effector_type": "epick", "end_effector_action": "close"}
{"task_type": "pick_sample", "gripper": "epick", "use_vision": true, ...}
```

### vision_moveto Task Format
Vision-guided movement to a detected marker with optional offsets.
```json
{
  "task_type": "vision_moveto",
  "tag_id": 30,
  "marker_offset_x": 0.02,       // Optional: marker-frame X offset (meters)
  "marker_offset_y": 0.0,        // Optional: marker-frame Y offset (meters)
  "marker_offset_z": 0.0,        // Optional: marker-frame Z offset (meters)
  "offset_direction": "right",   // Optional: flange-frame direction offset (uses DIRECTION_VECTORS)
  "offset_distance": 0.0312,     // Optional: distance for offset_direction (meters)
  "detect_only": true,            // Optional: return position without moving
  "z_offset": 0.003              // Optional: override approach height
}
```
- `marker_offset_x/y/z`: Offset from marker center in the **marker's local frame**. Transformed to base_link internally using the marker's orientation. Use for config-driven targets.
- `offset_direction` + `offset_distance`: Additional offset in **flange frame** (uses live flange TF). Applied after marker offset. Use for ad-hoc offsets.
- `detect_only`: Detects and returns `detected_position` and `detected_orientation` in the result without moving. Offsets are still applied to the returned position.
- **WARNING**: `detected_orientation` in the result is the **IK frame** orientation (e.g. epick_tip, robotiq_hande_end), NOT the flange orientation. There is a ~90° rotation between them (tool_block joint). Do NOT use `detected_orientation` to manually compute flange-frame direction offsets — use `offset_direction`/`offset_distance` or `marker_offset_x/y/z` instead. Using the wrong frame will send the robot to the wrong position.

### Contour-Based Sample Detection (detect_sample MCP tool)
For precise off-center sample pickup (e.g., X-ray beam needs the center clear):
1. Move to `sample_scan_1`
2. `capture_image(mode="3d")` — captures image + point cloud
3. `detect_sample(tag_id=0)` — returns `marker_offset_x/y` and `offset_from_center_mm`
4. `pick_sample` with `marker_offset_x/y` from detect_sample — single action goal handles detect → approach → vacuum → retreat
5. When placing on hotplate: add `offset_from_center_mm` to the base 51.2mm offset
   (e.g., 51.2 + 2.5 = 53.7mm right of tag 30) so the sample center aligns with the hotplate center
6. `place_sample` with `offset_direction="right"`, `offset_distance=0.0537` — single action goal

### Vision Target Framework
Config-driven vision targets are defined in `default_beamline.yaml` under `vision_targets`. Use the `vision_target` MCP tool to build task JSON from config. Two modes:
- **offset**: detect marker → move directly to marker-frame offset position (single move)
- **grid**: detect marker → move to marker → relative cartesian moves for grid element + approach/retreat
- **NOTE**: The `vision_target` tool returns task JSON with movement steps only. For targets that require end-effector actions (e.g. pipettor aspirate/dispense at a vial), you must manually insert the action step into the task list before sending to the orchestrator. Insert it between the last approach move and the retreat move. Example: for `vial_rack`, insert `{"task_type": "pipettor", ...}` between the forward (insert) and backward (retreat) steps.

### Experiment Protocols
Experiment protocols are defined in `src/cms/experiments.md`. Read this file before running experiments — it contains the step-by-step protocol, parameters (tag IDs, gripper, etc.), and any experiment-specific notes. The user edits this file before each experiment session. Execute protocols step by step using existing MCP tools.

### Optional Path Constraints

Any task step can include a `constraints` key to apply MoveIt path constraints during planning. If omitted, no constraints are applied (default behavior).

```json
{
  "task_type": "moveto",
  "target": "scan_position",
  "constraints": {
    "joint_constraints": [
      {"joint_name": "wrist_3_joint", "position": 0.0, "tolerance_above": 5.0, "tolerance_below": 5.0, "weight": 1.0}
    ],
    "orientation_constraints": [
      {"link_name": "flange", "frame_id": "base_link", "orientation": [180, 0, 0], "tolerance": [5, 5, 360], "weight": 1.0}
    ]
  }
}
```

- All angles (position, tolerance, orientation) are in **degrees** (converted to radians internally)
- `orientation` is `[roll, pitch, yaw]` (converted to quaternion internally)
- `tolerance` is `[x_axis, y_axis, z_axis]` tolerance in degrees
- Constraints apply to all arm movement stages in that task step (not gripper stages)
- Supported on `moveto`, `pick_sample`, and `place_sample` task types
- **Note**: OMPL handles path constraints via constraint-aware sampling. Tight tolerances may cause slow or failed planning -- prefer loose tolerances (10-30 degrees) when possible.

---

## MCP/ROS Action Interface

The [ros-mcp-server](https://github.com/robotmcp/ros-mcp-server) bridges LLM to ROS2 via rosbridge WebSocket (port 9090). On Humble, `get_action_details` cannot auto-resolve action types -- use the mapping below.

### Action Name -> Type Mapping

| Action Topic | Type | Purpose |
|---|---|---|
| `/beambot_execution` | `beambot_interfaces/action/MTCExecution` | **Primary entry point** -- JSON task dispatch |
| `/beambot_moveto` | `beambot_interfaces/action/MoveToAction` | Joint/Cartesian/relative moves |
| `/beambot_endeffector` | `beambot_interfaces/action/EndEffectorAction` | Open/close grippers |
| `/beambot_pick_sample` | `beambot_interfaces/action/PickSampleAction` | Unified pick (vision or hardcoded) |
| `/beambot_place_sample` | `beambot_interfaces/action/PlaceSampleAction` | Unified place (vision or hardcoded) |
| `/beambot_toolexchange` | `beambot_interfaces/action/ToolExchangeAction` | Dock/load grippers |
| `/beambot_vision_moveto` | `beambot_interfaces/action/VisionMoveToAction` | Vision-guided movement |
| `/beambot_vision_scan` | `beambot_interfaces/action/VisionScanAction` | Batch marker scanning |
| `/beambot_pipettor` | `beambot_interfaces/action/PipettorAction` | Liquid handling |

### MCP Startup -- Nothing Is Running Initially

**IMPORTANT**: When the user sends their first command, the robot system (MoveIt, TF, action servers) may **NOT be running yet**. The beambot orchestrator launches MoveIt lazily on the first goal.

**First-command workflow**:
1. Call `get_robot_state` to check if the system is running and which gripper is attached
2. If `system_running: true` and `gripper` is known -> use it for `start_gripper`
3. If `system_running: false` or `gripper: "unknown"` -> **ask the user** which gripper is attached
4. Construct task JSON and send to `/beambot_execution` -- the orchestrator handles MoveIt launch

**Note**: Do NOT query TF, topics, or services (via ros-mcp-server) before the first goal -- they will fail. `get_robot_state` is safe to call anytime because it reads from persistent subscriptions in the beambot-mcp-server.

### MCP Usage -- Always Use the Orchestrator

Send all tasks through `/beambot_execution` (not individual servers) -- it handles gripper tracking, MoveIt lifecycle, and stage batching:

```
send_action_goal(
  action_name="/beambot_execution",
  action_type="beambot_interfaces/action/MTCExecution",
  goal={"full_json": "<serialized JSON string>"}
)
```

The `full_json` value is a **JSON string** (not a nested object). Use `get_saved_poses()` from beambot-mcp-server to look up named positions from the pose registry (`src/cms/poses.yaml`).

**Relative move example** (no poses needed):
```json
{"start_gripper": "epick", "tasks": [{"task_type": "moveto", "target": "", "planning_type": "cartesian", "direction": "backward", "distance": 0.1}], "poses": {}}
```

**Named pose example** (poses defined inline):
```json
{"start_gripper": "epick", "tasks": [{"task_type": "moveto", "target": "sample_scan_1"}], "poses": {"sample_scan_1": [92.69, -109.33, -101.1, -59.48, 90.1, 2.6]}}
```

---

## MCP Gotchas

- **`start_gripper` must match the physically attached gripper**. Call `get_robot_state` first -- if the system is running it returns the current gripper. If `gripper: "unknown"`, **ask the user**. Valid values: `"hande"`, `"epick"`, `"pipettor"`, `"none"`. Sending the wrong gripper loads the wrong MoveIt config and causes planning failures.
- **Direction vectors** (`base_stages.py:DIRECTION_VECTORS`) — use these strings exactly as the user says them. Available: `forward`, `backward`, `left`, `right`, `up`, `down` (aliases: `x`, `-x`, `y`, `-y`, `z`, `-z`). These are in the `flange` frame. Do NOT remap or reinterpret directions.
- **Zivid single-shot capture** cannot be triggered via MCP `subscribe_once` (QoS timing race). Use beambot-mcp-server's `capture_image` tool (preferred), orchestrator's `vision_moveto`/`vision_scan` tasks, or a Python script with RELIABLE+VOLATILE QoS
- **MoveIt restarts after tool exchange** -- wait ~5s before sending the next goal
- **Cartesian planning may fail** for longer moves; use `"planning_type": "joint"` as fallback
- **`cartesian_target` orientation is in the `flange` frame** (MoveIt/ROS convention), NOT `tool0` (UR convention). `flange` and `tool0` are at the same position but rotated by (-90 deg, -90 deg, 0 deg). When querying current orientation for a Cartesian move, use `get_tf_transform(source_frame="flange")`. Using `tool0` RPY will make the robot reach a ~90 deg wrong orientation. Use `tool0` only when comparing with UR teach pendant values.
- **Joint poses are in DEGREES** in JSON -- converted to radians internally
- **Detailed field reference**: See `docs/mcp_ros_reference.md` for per-task-type goal fields, timeouts, and gripper config

---

## ePick Suction Cup Profiles

The ePick gripper supports swappable suction cups. Cup dimensions are defined in `epick_config/config/suction_cups.yaml` and affect the URDF geometry (collision, tip frame position).

**Changing cup profile via MCP** (no rebuild needed):
```
set_cup_profile(name="3mm_dia")
```
This sets the `cup_profile` ROS parameter on the orchestrator. Takes effect on the **next MoveIt launch** for ePick (next goal with `start_gripper="epick"`, or after tool exchange to ePick).

**Available profiles** (defined in `suction_cups.yaml`):
- `pen_vacuum` -- custom extension nozzle + small cup (34.5mm extension, 2mm cup)
- `7mm_dia` -- 7mm diameter cup with short extension (18mm extension, 6mm cup)
- `default` -- stock ePick suction cup (no extension, 20mm cup)

**Default**: Set in `default_beamline.yaml` under `grippers.epick.cup_profile`. Currently `"3mm_dia"`. The MCP `set_cup_profile` tool overrides this for the current session.

**Adding a new cup**: Add an entry to `suction_cups.yaml` with `extension_length`, `extension_radius`, `suction_cup_height`, `suction_cup_radius` (all in meters), then `colcon build --packages-select epick_config`.

---

## Error Handling -- Taxonomy & Recovery Policy

After sending a goal to `/beambot_execution`, **always read `error_message`** from the result before deciding what to do next. The orchestrator propagates specific error strings from MoveIt and the stage implementations.

### Error Taxonomy

| Error Pattern in `error_message` | Category | Corrective Action |
|---|---|---|
| `PLANNING_FAILED` (with `cartesian` in task) | Planning | Retry with `planning_type: joint` instead of `cartesian`. |
| `PLANNING_FAILED` (with `joint` in task) | Planning | Target may be unreachable from current configuration. Ask user for alternative target. |
| `GOAL_IN_COLLISION` | Planning | Do NOT retry same pose. The target is in collision. Ask user for an alternative position. |
| `START_STATE_IN_COLLISION` | Planning | Stale planning scene. If octomap is active, re-capture point cloud. Otherwise ask user. |
| `NO_IK_SOLUTION` | Planning | Target is outside robot workspace or kinematically unreachable. Ask user for different target. |
| `EXECUTION_FAILED` | Execution | Controller error. Ask user to check UR driver, e-stop status, and teach pendant. |
| `CONTROL_FAILED` | Execution | Same as EXECUTION_FAILED -- robot controller reported an error. |
| `TIMED_OUT` or `TIMEOUT` | Timeout | Check if the action server is running. Ask user to verify the system. |
| `DETECTION_FAILED` | Vision | Recapture image (`capture_image`). If 2nd attempt fails, ask user to check marker/lighting/camera. |
| `Pose '...' not found` | Configuration | The pose name doesn't exist in `poses` dict. Check spelling and available poses. |
| `Invalid pose format` | Configuration | The pose value isn't a list of 6 joint angles. Fix the poses JSON. |
| `Failed to parse poses_json` | Configuration | The poses JSON is malformed. Fix the JSON syntax. |
| `Pipettor action server ... not available` | Connectivity | Pipettor driver isn't running. Ask user to start it. |
| `Controller activation failed` | Connectivity | UR driver may have disconnected. Ask user to check robot connection. |
| `VACUUM_LOST` | Grasp | Object dropped during transport. Send `vacuum_off` then `vacuum_on` to retry pick. Do NOT proceed to place. |
| Unknown / doesn't match above | Unknown | Call `get_recent_logs(severity="ERROR", count=30)`, explain findings to user, propose fix. |

### Recovery Policy

- **NEVER move to an arbitrary position after a failure** without explicit user approval
- **NEVER retry the exact same goal** more than once -- if it failed once with the same parameters, it will fail again
- **ALWAYS read `error_message`** in the action result before deciding the next action
- **For known errors** (matches table above): Follow the prescribed corrective action deterministically
- **For unknown errors**: Call `get_recent_logs(severity="ERROR", count=30)`, read the MoveIt/beambot logs, explain your findings to the user, and wait for their approval before taking action
- **After PLANNING_FAILED with cartesian**: The single allowed automatic retry is switching to `planning_type: joint`. Do NOT try other arbitrary poses.
- **After DETECTION_FAILED**: One automatic re-capture is allowed. If the second detection also fails, ask the user.
- **After ePick vacuum pick**: Call `get_vacuum_status()` to verify the object was grasped. If `object_detected` is false (`NO_OBJECT_DETECTED`), do NOT proceed to transport. Report to user and ask whether to retry the pick or abort.
- **ePick retry requires off→on cycle**: The ePick hardware will NOT re-attempt suction automatically. To retry a failed pick, you MUST send `vacuum_off` first, then `vacuum_on` again — even if no object was detected. Skipping the off step means the vacuum won't activate.

---

## Operational Warnings

- The orchestrator manages MoveIt lifecycle -- don't launch MoveIt separately
- MoveIt restarts when switching grippers (2-5s delay)
- Always `source install/setup.bash` after building
