# EROBS - MCP Operations Reference

> For architecture, development setup, TODOs, and design decisions, see [`docs/development.md`](docs/development.md).

## Task JSON Format

All tasks are sent as a JSON string to `/beambot_execution`. The top-level structure:

```json
{
  "start_gripper": "hande",
  "tasks": [
    {"task_type": "moveto", "target": "home"},
    {"task_type": "pick_and_place", "gripper": "hande", ...},
    {"task_type": "vision_moveto", "tag_id": 5}
  ],
  "poses": {"home": [0, -90, 90, -90, -90, 0]}
}
```
Note: Joint poses are in **degrees**, converted to radians internally.

### pick_and_place Task Format
```json
{
  "task_type": "pick_and_place",
  "gripper": "hande",            // Optional - defaults to current gripper
  "pick_approach": "pickup_approach",
  "pick_target": "pickup",
  "place_approach": "place_approach",
  "place_target": "place"
}
```
Executes 9-stage sequence: open -> approach -> pick -> close -> retreat -> approach -> place -> open -> retreat

### tool_exchange Task Format
```json
{"task_type": "tool_exchange", "operation": "dock", "gripper": "epick", "dock_number": 4, "approach_pose": "dock_approach"}
{"task_type": "tool_exchange", "operation": "load", "gripper": "pipettor", "dock_number": 2, "approach_pose": "load_approach"}
```
- `operation`: `"dock"` (put gripper away) or `"load"` (pick up gripper)
- `gripper`: Which gripper to dock/load
- `dock_number`: Physical dock slot number
- `approach_pose`: Joint pose name for approaching the dock
- **IMPORTANT: ALWAYS use `"dock_approach"` for dock operations and `"load_approach"` for load operations.** These are different poses tuned for each operation direction. Using the wrong approach pose causes collisions or failed exchanges.

### Gripper Auto-Detection
Tasks that need gripper info (`end_effector`, `pick_and_place`, `vision_pick_place`) default to the currently attached gripper if not specified. The orchestrator tracks `start_gripper` and updates it after `tool_exchange` operations.

**Simplified JSON** (uses current gripper):
```json
{"task_type": "end_effector", "end_effector_action": "close"}
{"task_type": "pick_and_place", "pick_approach": "...", ...}
```

**Explicit override** (when you need a specific gripper):
```json
{"task_type": "end_effector", "end_effector_type": "epick", "end_effector_action": "close"}
{"task_type": "pick_and_place", "gripper": "epick", ...}
```

### vision_pick_place Task Format
Hybrid operation: vision-guided pick + hardcoded place positions.
```json
{
  "task_type": "vision_pick_place",
  "detection_type": "marker",        // "marker" or "circle"
  "tag_id": 5,                       // ArUco marker ID (for marker detection)
  "z_offset": 0.0,                   // Optional: height above detected point (default: from beamline config z_offset)
  "sample_approach": "scan_position", // Joint pose key - robot scans from here
  "place_approach": "place_approach", // Joint pose key
  "place_target": "place"             // Joint pose key
}
```
Executes: open -> sample_approach -> [detect] -> grasp -> close -> retreat -> place_approach -> place -> open -> retreat

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
- Supported on `moveto`, `pick_and_place`, and `vision_pick_place` task types
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
| `/beambot_pickplace` | `beambot_interfaces/action/PickPlaceAction` | 9-stage pick-and-place |
| `/beambot_toolexchange` | `beambot_interfaces/action/ToolExchangeAction` | Dock/load grippers |
| `/beambot_vision_moveto` | `beambot_interfaces/action/VisionMoveToAction` | Vision-guided movement |
| `/beambot_vision_scan` | `beambot_interfaces/action/VisionScanAction` | Batch marker scanning |
| `/beambot_vision_pickplace` | `beambot_interfaces/action/VisionPickPlaceAction` | Vision pick + hardcoded place |
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
set_cup_profile(name="7mm_dia")
```
This sets the `cup_profile` ROS parameter on the orchestrator. Takes effect on the **next MoveIt launch** for ePick (next goal with `start_gripper="epick"`, or after tool exchange to ePick).

**Available profiles** (defined in `suction_cups.yaml`):
- `pen_vacuum` -- custom extension nozzle + small cup (34.5mm extension, 2mm cup)
- `7mm_dia` -- 7mm diameter cup with short extension (18mm extension, 6mm cup)
- `default` -- stock ePick suction cup (no extension, 20mm cup)

**Default**: Set in `default_beamline.yaml` under `grippers.epick.cup_profile`. Currently `"7mm_dia"`. The MCP `set_cup_profile` tool overrides this for the current session.

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
