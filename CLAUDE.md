# EROBS - Extensible Robotic Beamline Scientist

## Overview
Autonomous robotic sample handling system for synchrotron beamlines at NSLS-II. Integrates ROS2 robotics with Bluesky experiment orchestration to enable **self-driving beamlines** that can run 24/7 without human intervention.

**Goal**: Make this framework beamline-agnostic so any beamline can use UR robots for their sample manipulation needs.

## Architecture

```
Bluesky Adaptive (AI agent suggests next sample)
         ↓
Bluesky RunEngine (experiment orchestration)
         ↓
Ophyd Device (ROS2 Action Client wrapper)
         ↓
MTCOrchestratorActionServer (JSON task dispatcher)
         ↓
Specialized Action Servers (8 types)
    ├── move_to, pick_place, end_effector
    ├── tool_exchange, vision_moveto, vision_scan
    └── vision_pick_place, pipettor
         ↓
MoveIt Task Constructor (motion planning)
         ↓
UR5e Robot + Grippers
```

**Deployment**: VM with two Docker containers communicating via ROS2 DDS:
- **bsui**: Bluesky/experiment orchestration, sends JSON task goals
- **erobs-common-img**: MTC pipeline servers, MoveIt, Zivid SDK
- **erobs-mcp-server**: MCP bridge for LLM control (Zivid capture, detection, TF, robot state, pose registry)

## Key Packages

| Package | Purpose |
|---------|---------|
| **beambot** | Python action servers, orchestrator, detection algorithms, MCP server |
| **beambot_interfaces** | Action definitions (8 actions) |
| **mtc_gui** | GUI client for task execution |
| **custom-ur-descriptions** | MoveIt configs per gripper type |
| **vision** | Zivid 3D camera driver + ROS2 nodes |
| **end_effectors** | Gripper drivers (Hand-E, ePick, pipettor) |
| **bluesky_ros** | Bluesky-ROS integration (Ophyd devices) |
| **cms** | CMS beamline task JSONs and pose registry (`poses.yaml`) |

## Hardware

- **Robot**: UR5e 6-DOF arm
- **Cameras**: Zivid 2+ 3D (eye-in-hand, single-shot), ZED (external, streaming)
- **Grippers** (swappable): Robotiq Hand-E, Robotiq ePick, Pipettor

### Hand-Eye Calibration History

The Zivid camera is mounted on the robot arm (eye-in-hand). The transform `tool0 → zivid_optical_frame` is stored in `ur5e_robot_description/urdf/zivid_camera_mount.xacro`.

**Re-run calibration when**: Robot is moved to a different location, camera mount is disturbed, or vision accuracy degrades.

| Date | xyz (meters) | rpy (radians) | Notes |
|------|--------------|---------------|-------|
| **2026-01-15** | 0.05675 0.10322 0.05489 | -0.00615 0.04362 3.13541 | Current. Recalibration. Residuals: rot < 0.22°, trans < 0.47mm |
| 2026-01-13 | 0.05646 0.10182 0.05680 | -0.03542 0.04745 3.13222 | After robot moved to new room |
| 2025-12-17 | 0.05659 0.10548 0.05660 | -0.01432 0.04829 3.13430 | Original location |
| 2025-10-09 | 0.02803 0.07664 0.0 | 0.53964 -1.53712 -2.13794 | Initial calibration (different mount?) |

**Calibration tool**: `zivid-python-samples/source/applications/advanced/hand_eye_calibration/hand_eye_gui.py` or Zivid Studio → Tools → Hand-Eye Calibration

## Commands

```bash
colcon build && source install/setup.bash
ros2 launch beambot beambot_bringup.launch.py
ros2 launch beambot beambot_bringup.launch.py use_fake_hardware:=true  # simulation
ros2 run mtc_gui mtc_gui_client  # GUI
```

## Task JSON Format

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
Executes 9-stage sequence: open → approach → pick → close → retreat → approach → place → open → retreat

### Gripper Auto-Detection
Tasks that need gripper info (`end_effector`, `pick_and_place`, `vision_pick_place`) now default to the currently attached gripper if not specified. The orchestrator tracks `start_gripper` and updates it after `tool_exchange` operations.

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
  "z_offset": 0.02,                  // Optional: height above detected point (default: 0.02m)
  "sample_approach": "scan_position", // Joint pose key - robot scans from here
  "place_approach": "place_approach", // Joint pose key
  "place_target": "place"             // Joint pose key
}
```
Executes: open → sample_approach → [detect] → grasp → close → retreat → place_approach → place → open → retreat

---

## Current Work

### Completed

| # | Feature | Key Files |
|---|---------|-----------|
| 1 | Beambot Framework (mtc_pipeline → Python) | `src/beambot/` |
| 2 | GUI Client (direct ActionClient) | `src/mtc_gui/` |
| 4 | MTC Stage Batching (~1.5s saved per task) | `orchestrator.py`, `*_stages.py` |
| 6 | Separate interfaces package | `src/beambot_interfaces/` |
| 7 | Clean up cancel_callback | `base_action_server.py` |
| 8 | Simplify MTC node init | `base_stages.py` |
| 9 | Rename mtc_py → beambot | All packages updated |
| 10 | Update bluesky_ros for beambot | `mtc_ophyd_device*.py` |
| 11 | Camera mount recalibration | `zivid_camera_mount.xacro` |
| 12 | Vision detection retry logic | `vision_stages.py` |
| 13 | Camera-agnostic vision (factory pattern) | `beambot/camera/` |
| 14 | Pause/Resume functionality | `orchestrator.py`, `mtc_gui_client.py` |
| 15 | GUI: Clear/Reorder tasks | `mtc_gui_client.py` |
| 19 | Dedicated PickPlaceAction (single task, 9 stages) | `pick_place_stages.py` |
| 20 | Auto-detect current gripper (no need to specify in task JSON) | `orchestrator.py` |
| 21 | Hybrid VisionPickPlace (vision pick + hardcoded place) | `vision_pick_place_stages.py` |
| 17 | Point Cloud Obstacle Avoidance (Zivid → Octomap → MoveIt) | `octomap_to_planning_scene.py`, `pointcloud_relay.py` |

### 3. Motion Planning Improvements
- **Status**: TODO (detailed plan ready)
- **Goal**: Improve planning reliability, speed, and trajectory quality
- **Current**: OMPL/RRTConnect (`goal_bias=0.15`), MTC `CartesianPath` (`min_fraction=0.95`), 30% velocity/accel scaling
- **Known issues**:
  - CartesianPath fails for longer moves (incremental IK stepping hits singularities/joint limits)
  - RRTConnect produces unintuitive joint-space paths (random sampling, not shortest path)
- **Plan** (4 phases):
  1. **Pilz LIN for Cartesian targets** — deterministic straight-line, more robust than CartesianPath for longer distances. Already in MoveIt2 Humble (`moveit_planners_pilz`), just needs config.
  2. **OMPL tuning** — increase `goal_bias` to 0.3+, verify path simplification adapters
  3. **MTC Fallbacks container** — CartesianPath → Pilz LIN → OMPL cascade
  4. **Pilz PTP for joint moves** (optional) — predictable industrial-standard joint motion
- **Detailed notes**: `~/.claude/projects/.../memory/motion_planning_improvements.md`
- **Files**: `base_stages.py`, `move_to_stages.py`, `ur5e_moveit_configs/*/config/ompl_planning.yaml`

### 5. Minimal bsui Container
- **Status**: TODO
- **Goal**: Reduce bsui from ~5GB to ~500MB (only needs rclpy + beambot_interfaces)
- **Files**: `docker/bsui/Dockerfile`

### 16. Smart Tagless Sample Detection
- **Status**: IN PROGRESS (circle + contour detection done, needs refinement)
- **Goal**: Replace ArUco markers with smart detection (circles, ML, depth segmentation)
- **Completed**:
  - Hough circle detection in `zivid.py`
  - Contour detection (any shape) with reading-order sorting
  - `detection_type` field in action/GUI ("marker", "circle", "contour")
  - `sample_index` field for selecting which detected object (1-indexed)
  - GUI "Detect Contours" button for visualization before execution
- **Files**: `beambot/camera/zivid.py`, `vision_stages.py`, `VisionMoveToAction.action`, `mtc_gui_client.py`
- **Investigation**: `src/beambot/docs/aruco_detection_variance_investigation.md` — variance analysis of ArUco detection across captures

#### Known Issues (TODO)
1. **Detection reliability**: Contours don't always get detected consistently (lighting/contrast dependent)
2. **Label stability**: If detection changes between captures, sample labels shift (object that was #2 might become #1)
3. **Centroid accuracy**: Robot doesn't move to exact center of detected object (grasp offset)

#### Implementation Steps

**Step 1: Pre-filtering (if required)**
- Filter image to isolate sample wafers before detection
- Use depth segmentation (Zivid advantage): filter point cloud to table height ± tolerance
- ROI masking: define valid sample regions, ignore background
- Color/HSV filtering if samples have distinctive appearance
- Purpose: Reduce false positives from circular objects that aren't samples

**Step 2: Expand Detection Methods**
- Current: Hough Circle Transform (edge-based, finds mathematical circles)
- Try: `cv2.SimpleBlobDetector` (region-based, better for filled circles)
- Try: Contour analysis + `cv2.approxPolyDP` for non-circular shapes
- Try: Template matching for known sample types
- Try: Ellipse fitting (`cv2.fitEllipse`) for angled circular samples
- Combine multiple methods with confidence scoring

**Step 3: Multi-Sample Labeling & Selection**
- Problem: Multiple samples detected - how to select which one to pick?
- Options:
  - **Spatial indexing**: Label by grid position (A1, A2, B1...) based on detected layout
  - **Geometric ordering**: Sort by x,y coordinates, assign sequential IDs
  - **Hybrid with AprilTags**: Use AprilTags on racks (not samples) to define coordinate system
  - **User selection**: GUI shows detected samples, user clicks to select
- Reliability consideration: AprilTags on rack corners provide stable reference frame
- Store sample registry: `{sample_id: (x, y, z, detection_confidence, timestamp)}`

**Step 4: Sample Orientation Detection**
- Problem: If sample is tilted/not flat, affects grasp accuracy
- Approaches:
  - **Plane fitting**: Fit plane to sample surface points, extract normal vector
  - **PCA on point cloud**: Principal components give orientation axes
  - **Ellipse fitting**: If circle appears as ellipse, tilt angle = arccos(minor/major)
  - **Surface normal from depth gradient**: `cv2.Sobel` on depth image
- Impact on pick-and-place:
  - Adjust gripper approach angle to match sample orientation
  - Add pre-grasp alignment stage in MTC pipeline
  - Set tolerance threshold: if tilt > X°, flag for manual intervention

**Step 5: AI/ML-Based Detection (YOLOv8)**
- When to use: When rule-based detection is insufficient or too brittle
- Dataset creation:
  - Capture 200-500 images from Zivid during normal operation
  - Label with Roboflow (free tier) or Label Studio (self-hosted)
  - Classes: `vial`, `plate`, `wafer`, `petri_dish`, etc.
  - Export in YOLO format
- Training:
  ```bash
  pip install ultralytics
  yolo train data=samples.yaml model=yolov8n.pt epochs=100 imgsz=640
  ```
- Integration options:
  - Direct: Add to `ZividCamera` class alongside existing methods
  - ROS2 node: Separate `sample_detector_node.py` publishing to `/sample_detections`
- Hybrid approach (recommended):
  - ML detection with lower confidence threshold (find candidates)
  - Geometric verification with depth data (validate candidates)
  - Fallback to pure geometric if ML fails
- Estimated effort: ~1-2 days for basic working pipeline

### 17. Point Cloud Obstacle Avoidance with Octomap
- **Status**: ✅ COMPLETE
- **Goal**: Use Zivid point cloud for dynamic collision avoidance
- **Architecture**:
  ```
  Zivid → pointcloud_relay (downsample) → octomap_server → octomap_to_planning_scene → MoveIt
  ```
- **Why not MoveIt native perception?** tf2 MessageFilter has hardcoded 5-message queue and no configurable timeout — unsuitable for single-shot cameras. Standalone octomap_server has `transform_tolerance: 5.0s`.
- **Usage**:
  ```bash
  ros2 launch beambot beambot_bringup.launch.py   # Terminal 1
  ros2 launch beambot octomap_test.launch.py      # Terminal 2
  # Trigger Zivid capture → octomap appears in Planning Scene
  ```
- **TODO**: Integrate into `beambot_bringup.launch.py` (IncludeLaunchDescription) with `use_octomap:=true` arg
- **Files**: `octomap_test.launch.py`, `pointcloud_relay.py`, `octomap_to_planning_scene.py`

### 18. Improve Cartesian Path Planner Reliability
- **Status**: TODO
- **Goal**: Make Cartesian planning more reliable
- **Verified**: MTC CartesianPath DOES check collisions (via `is_valid` callback at every 1mm step)
- **Completed**: `min_fraction` updated from 0.6 → 0.95
- **TODO**: Implement Fallbacks container (Cartesian first, OMPL backup)
- **Files**: `base_stages.py`, `pick_place_stages.py`

---

## Design Decisions

### PickPlaceAction Execution Mode
**Decision**: Use single MTC task (all 9 stages together) as default instead of split tasks.

**Context**: MTC has no native delay/wait stage. When gripper closes, the next motion can start before the gripper physically completes. We implemented two modes:

1. `run()` - **DEFAULT**: All 9 stages in one MTC task
   - Fastest execution, smoothest trajectory
   - Gripper may still be closing when arm starts moving
   - **Works fine in practice** - tested and grip/release complete in time

2. `run_with_gripper_settle()` - Split into 3 MTC tasks
   - Task 1: open → approach → pick → close
   - Task 2: retreat → approach → place → open
   - Task 3: retreat
   - Planning time between tasks provides natural delay for gripper
   - Use if gripper timing becomes an issue

**To switch**: In `pick_place_server.py`, change `self._stages.run()` to `self._stages.run_with_gripper_settle()`

**Files**: `pick_place_stages.py`

---

## Important Warnings

- Always `source install/setup.bash` after building
- MoveIt restarts when switching grippers (2-5s delay)
- Vision requires Zivid camera connected and calibrated
- The orchestrator manages MoveIt lifecycle - don't launch MoveIt separately

## Debugging

```bash
ros2 action list                          # Check action servers
ros2 run tf2_tools view_frames            # TF tree (vision issues)
ros2 topic echo /joint_states             # Joint states
ros2 service call /beambot/pause std_srvs/srv/Trigger  # Pause execution
ros2 topic echo /beambot/execution_state  # Monitor state
```

## MCP/ROS Action Interface

The [ros-mcp-server](https://github.com/robotmcp/ros-mcp-server) bridges LLM to ROS2 via rosbridge WebSocket (port 9090). On Humble, `get_action_details` cannot auto-resolve action types — use the mapping below.

### Action Name → Type Mapping

| Action Topic | Type | Purpose |
|---|---|---|
| `/beambot_execution` | `beambot_interfaces/action/MTCExecution` | **Primary entry point** — JSON task dispatch |
| `/beambot_moveto` | `beambot_interfaces/action/MoveToAction` | Joint/Cartesian/relative moves |
| `/beambot_endeffector` | `beambot_interfaces/action/EndEffectorAction` | Open/close grippers |
| `/beambot_pickplace` | `beambot_interfaces/action/PickPlaceAction` | 9-stage pick-and-place |
| `/beambot_toolexchange` | `beambot_interfaces/action/ToolExchangeAction` | Dock/load grippers |
| `/beambot_vision_moveto` | `beambot_interfaces/action/VisionMoveToAction` | Vision-guided movement |
| `/beambot_vision_scan` | `beambot_interfaces/action/VisionScanAction` | Batch marker scanning |
| `/beambot_vision_pickplace` | `beambot_interfaces/action/VisionPickPlaceAction` | Vision pick + hardcoded place |
| `/beambot_pipettor` | `beambot_interfaces/action/PipettorAction` | Liquid handling |

### MCP Startup — Nothing Is Running Initially

**IMPORTANT**: When the user sends their first command, the robot system (MoveIt, TF, action servers) may **NOT be running yet**. The beambot orchestrator launches MoveIt lazily on the first goal.

**First-command workflow**:
1. Call `get_robot_state` to check if the system is running and which gripper is attached
2. If `system_running: true` and `gripper` is known → use it for `start_gripper`
3. If `system_running: false` or `gripper: "unknown"` → **ask the user** which gripper is attached
4. Construct task JSON and send to `/beambot_execution` — the orchestrator handles MoveIt launch

**Note**: Do NOT query TF, topics, or services (via ros-mcp-server) before the first goal — they will fail. `get_robot_state` is safe to call anytime because it reads from persistent subscriptions in the erobs-mcp-server.

### MCP Usage — Always Use the Orchestrator

Send all tasks through `/beambot_execution` (not individual servers) — it handles gripper tracking, MoveIt lifecycle, and stage batching:

```
send_action_goal(
  action_name="/beambot_execution",
  action_type="beambot_interfaces/action/MTCExecution",
  goal={"full_json": "<serialized JSON string>"}
)
```

The `full_json` value is a **JSON string** (not a nested object). See "Task JSON Format" section above for the JSON structure. Use `get_saved_poses()` from erobs-mcp-server to look up named positions from the pose registry (`src/cms/poses.yaml`).

**Relative move example** (no poses needed):
```json
{"start_gripper": "epick", "tasks": [{"task_type": "moveto", "target": "", "planning_type": "cartesian", "direction": "backward", "distance": 0.1}], "poses": {}}
```

**Named pose example** (poses defined inline):
```json
{"start_gripper": "epick", "tasks": [{"task_type": "moveto", "target": "sample_scan_1"}], "poses": {"sample_scan_1": [92.69, -109.33, -101.1, -59.48, 90.1, 2.6]}}
```

### MCP Gotchas

- **`start_gripper` must match the physically attached gripper**. Call `get_robot_state` first — if the system is running it returns the current gripper. If `gripper: "unknown"`, **ask the user**. Valid values: `"hande"`, `"epick"`, `"pipettor"`, `"none"`. Sending the wrong gripper loads the wrong MoveIt config and causes planning failures.
- **Direction vectors are in `flange` frame**, not world frame. At a downward-looking pose, "forward" ≈ down toward table. Left/right and up/down are swapped due to 180° wrist rotation compensation. See `base_stages.py:DIRECTION_VECTORS`
- **Zivid single-shot capture** cannot be triggered via MCP `subscribe_once` (QoS timing race). Use erobs-mcp-server's `capture_image` tool (preferred), orchestrator's `vision_moveto`/`vision_scan` tasks, or a Python script with RELIABLE+VOLATILE QoS
- **MoveIt restarts after tool exchange** — wait ~5s before sending the next goal
- **Cartesian planning may fail** for longer moves; use `"planning_type": "joint"` as fallback
- **`cartesian_target` orientation is in the `flange` frame** (MoveIt/ROS convention), NOT `tool0` (UR convention). `flange` and `tool0` are at the same position but rotated by (-90°, -90°, 0°). When querying current orientation for a Cartesian move, use `get_tf_transform(source_frame="flange")`. Using `tool0` RPY will make the robot reach a ~90° wrong orientation. Use `tool0` only when comparing with UR teach pendant values.
- **Joint poses are in DEGREES** in JSON — converted to radians internally
- **Detailed field reference**: See `docs/mcp_ros_reference.md` for per-task-type goal fields, timeouts, and gripper config

### Error Handling — Taxonomy & Recovery Policy

After sending a goal to `/beambot_execution`, **always read `error_message`** from the result before deciding what to do next. The orchestrator now propagates specific error strings from MoveIt and the stage implementations.

#### Error Taxonomy

| Error Pattern in `error_message` | Category | Corrective Action |
|---|---|---|
| `PLANNING_FAILED` (with `cartesian` in task) | Planning | Retry with `planning_type: joint` instead of `cartesian`. |
| `PLANNING_FAILED` (with `joint` in task) | Planning | Target may be unreachable from current configuration. Ask user for alternative target. |
| `GOAL_IN_COLLISION` | Planning | Do NOT retry same pose. The target is in collision. Ask user for an alternative position. |
| `START_STATE_IN_COLLISION` | Planning | Stale planning scene. If octomap is active, re-capture point cloud. Otherwise ask user. |
| `NO_IK_SOLUTION` | Planning | Target is outside robot workspace or kinematically unreachable. Ask user for different target. |
| `EXECUTION_FAILED` | Execution | Controller error. Ask user to check UR driver, e-stop status, and teach pendant. |
| `CONTROL_FAILED` | Execution | Same as EXECUTION_FAILED — robot controller reported an error. |
| `TIMED_OUT` or `TIMEOUT` | Timeout | Check if the action server is running. Ask user to verify the system. |
| `DETECTION_FAILED` | Vision | Recapture image (`capture_image`). If 2nd attempt fails, ask user to check marker/lighting/camera. |
| `Pose '...' not found` | Configuration | The pose name doesn't exist in `poses` dict. Check spelling and available poses. |
| `Invalid pose format` | Configuration | The pose value isn't a list of 6 joint angles. Fix the poses JSON. |
| `Failed to parse poses_json` | Configuration | The poses JSON is malformed. Fix the JSON syntax. |
| `Pipettor action server ... not available` | Connectivity | Pipettor driver isn't running. Ask user to start it. |
| `Controller activation failed` | Connectivity | UR driver may have disconnected. Ask user to check robot connection. |
| Unknown / doesn't match above | Unknown | Call `get_recent_logs(severity="ERROR", count=30)`, explain findings to user, propose fix. |

#### Recovery Policy

- **NEVER move to an arbitrary position after a failure** without explicit user approval
- **NEVER retry the exact same goal** more than once — if it failed once with the same parameters, it will fail again
- **ALWAYS read `error_message`** in the action result before deciding the next action
- **For known errors** (matches table above): Follow the prescribed corrective action deterministically
- **For unknown errors**: Call `get_recent_logs(severity="ERROR", count=30)`, read the MoveIt/beambot logs, explain your findings to the user, and wait for their approval before taking action
- **After PLANNING_FAILED with cartesian**: The single allowed automatic retry is switching to `planning_type: joint`. Do NOT try other arbitrary poses.
- **After DETECTION_FAILED**: One automatic re-capture is allowed. If the second detection also fails, ask the user.

## File Locations

| What | Where |
|------|-------|
| Action definitions | `beambot_interfaces/action/` |
| Action servers | `beambot/beambot/action_servers/` |
| Stage implementations | `beambot/beambot/stages/` |
| Detection algorithms | `beambot/beambot/detection/` |
| MCP server (erobs) | `beambot/mcp/erobs_mcp_server.py` |
| Gripper configs | `beambot/config/grippers.yaml` |
| Beamline configs | `beambot/config/*.yaml` |
| Pose registry | `cms/poses.yaml` |
| MoveIt configs | `custom-ur-descriptions/ur5e_moveit_configs/` |
| Launch files | `beambot/launch/` |
| MCP architecture design | `docs/mcp_architecture_design.md` |
| MCP detailed reference | `docs/mcp_ros_reference.md` |
| Isaac Sim integration | `docs/isaac_sim_integration.md` |

## References

- [Digital Discovery Paper (2025)](https://doi.org/10.1039/d5dd00036j) - Full architecture
- [ICRA 2024 Paper](https://doi.org/10.1109/ICRA57147.2024.10611706) - Bluesky-ROS integration
- [MoveIt Task Constructor](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor.html)

---

## Roadmap: Robust Pick-and-Place

**Target**: Scientists write `yield from robot_plans.load_sample(robot, "A1")` - no JSON knowledge needed.

```
Phase 1: Scaffold Grasp Pipeline ← NEXT
├── MTC GenerateGraspPose stages (multiple candidates)
├── Grasp type config (TOP_DOWN, SIDE_X per object/gripper)
└── Goal: Working pipeline (~50-60% success OK)

Phase 2: Tune Planning for Grasp Workload
├── Benchmark IK/planning pass rates
├── Tune for multi-candidate pattern (fast queries, fail fast)
└── Goal: 80%+ success rate

Phase 3: Add Path Constraints
├── "Keep level" OrientationConstraint for transfers
└── Goal: Objects stay horizontal

Phase 4: Vision Integration
├── ArUco → object pose → grasp pipeline
└── Goal: Full vision-guided pick-and-place

Phase 5: Scientist Interface
├── Typed DSL (Python dataclasses with autocomplete)
├── High-level plans library
└── Goal: Clean scientist experience
```

**Key Decisions**:
- Keep orchestrator (needed for tool exchange, batching)
- MTC grasp stages over AI grasping (objects are KNOWN, not arbitrary)
- Grasping before planning tuning (tuning is workload-dependent)
