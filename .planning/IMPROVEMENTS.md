# Improvements - Ranked by Priority
*Date: 2026-03-06 | Branch: humble-experimental*

Based on documentation and codebase audits.

---

## HIGH Priority

### 1. Extract Shared Detection Module
**Impact**: Eliminates ~200 lines of duplicated code, prevents bug divergence
**Effort**: Small (2-4 hours)

The detection functions (`_detect_hough_circles`, `_detect_contours_in_image`, `_sort_contours_reading_order`, `_get_3d_position`, `CircleDetectionParams`, `ContourDetectionParams`) are duplicated between `camera/zivid.py` and `mcp/erobs_mcp_server.py`.

**Action**: Create `beambot/detection/` module with pure OpenCV functions (no ROS dependencies). Both `zivid.py` and `erobs_mcp_server.py` import from it.

### 2. Add MCP Tool for Task Execution
**Impact**: Enables Claude Code to execute full robot task sequences via MCP
**Effort**: Medium (4-8 hours)

The `erobs_mcp_server.py` currently only has vision tools (capture, detect, TF). It cannot send task JSONs to the orchestrator. Claude Code must use `ros-mcp-server` for action calls, which requires manual JSON construction.

**Action**: Add an `execute_task` MCP tool that:
- Accepts task JSON (same format as `beambot_client.py`)
- Sends it to `beambot_execution` action server
- Returns progress feedback and final result
- Supports cancel

### 3. Update Documentation for MCP Era
**Impact**: Reduces confusion, aligns team understanding
**Effort**: Medium (4-8 hours)

README.md, .planning/ files, and architecture diagrams are all pre-MCP. The MCP approach is now the primary interaction model but isn't documented in the main README.

**Action**:
- Update README.md with MCP architecture
- Archive or update stale .planning/ docs
- Add MCP tools reference to docs/

### 4. Add Gripper Feedback to MCP Server
**Impact**: Enables Claude to verify grip success before proceeding
**Effort**: Medium (4-8 hours)

Currently, the MCP server has no way to check if the gripper actually grasped an object. Adding a `check_gripper_status` tool would let Claude verify grasp before continuing.

**Action**: Add MCP tool that reads gripper state (HandE position/force, ePick vacuum level) from ROS topics.

### 5. Make Velocity/Acceleration Scaling Configurable
**Impact**: Allows tuning speed per-task without code changes
**Effort**: Small (1-2 hours)

Currently hardcoded at 20% in `base_stages.py:44-45`. Should be configurable via ROS parameters or beamline config.

**Action**: Add `velocity_scaling` and `acceleration_scaling` to beamline config and/or as per-task parameters in the JSON format.

---

## MEDIUM Priority

### 6. Archive Legacy Packages
**Impact**: Reduces build time, simplifies repo
**Effort**: Small (1-2 hours)

`aruco_pose`, `pdf/pdf_beamtime`, `bluesky_ros/archive/` are unused. They add build complexity and confusion.

**Action**: Move to `src/archive/` or remove from colcon build (add COLCON_IGNORE markers).

### 7. Add Task JSON Schema Validation
**Impact**: Catch errors before execution, better error messages
**Effort**: Medium (4-8 hours)

No schema validation for task JSON files. Invalid JSON gets cryptic error messages deep in the action server.

**Action**: Add JSON Schema for task format. Validate in orchestrator's `_parse_goal()` with clear error messages listing exactly what's wrong.

### 8. Centralize Camera Config Loading
**Impact**: Removes code duplication in vision servers
**Effort**: Small (1-2 hours)

`vision_server.py` and `vision_pick_place_server.py` both have identical camera config loading logic (~15 lines each).

**Action**: Extract to a shared utility function in `beambot/core/`.

### 9. Fix Typo in default_beamline.yaml
**Impact**: Cosmetic
**Effort**: Trivial

Line 3: "This cnfig" should be "This config"

### 10. Add Health Check MCP Tool
**Impact**: Enables Claude to diagnose issues before attempting operations
**Effort**: Medium (4-8 hours)

**Action**: Add `system_health` MCP tool that checks:
- Robot driver status (is UR5e connected?)
- Camera status (is Zivid running?)
- Controller status (are controllers active?)
- MoveIt status (is move_group running?)
- Gripper status (which gripper is active?)

### 11. Remove ZED from vision.repos
**Impact**: Simplifies build, removes unused dependency
**Effort**: Trivial

`zed-ros2-wrapper` is referenced in `vision.repos` but not used.

### 12. Consolidate CMS Task Files
**Impact**: Easier to find and manage task sequences
**Effort**: Small (2-4 hours)

35+ JSON files in `src/cms/tasks/`, some are tests, some are production. File with space in name (`beamline_test copy.json`).

**Action**: Organize into subdirectories (test/, beamtime/, development/), remove space from filename, add index/description.

---

## LOW Priority

### 13. Add MCP Resource for Current Robot State
**Impact**: Enables Claude to query robot state naturally
**Effort**: Medium

Use MCP Resources (not tools) to expose robot state as queryable data: joint positions, current gripper, execution state, TF frames.

### 14. Improve Error Messages in Stages
**Impact**: Better debugging experience
**Effort**: Small

Some stage failures produce generic "Planning failed" messages. Add more context about which constraint failed, what the target was.

### 15. Add Unit Tests for Detection Functions
**Impact**: Prevent regression, validate detection reliability
**Effort**: Medium (8+ hours)

No unit tests for detection functions. Once extracted to shared module (Improvement #1), add tests with sample images.

### 16. Evaluate Pilz Planner Integration
**Impact**: More predictable Cartesian paths
**Effort**: Medium-Large

Already noted in CLAUDE.md as a planned improvement. Pilz LIN would be more deterministic than CartesianPath for longer moves.

### 17. Add Execution Logging/History
**Impact**: Enables post-hoc analysis of experiment runs
**Effort**: Medium

Currently no persistent logging of task executions. Add a structured log (JSON or database) recording: timestamp, task JSON, result, duration, errors.

### 18. Move Dock Spacing to Beamline Config
**Impact**: Supports different tool dock configurations
**Effort**: Trivial

`DOCK_SPACING_METERS = 0.1524` in `tool_exchange_stages.py` should be in beamline config.

### 19. Clean Up pixi.toml
**Impact**: Reduces confusion about build system
**Effort**: Trivial

pixi.toml references bluesky/ophyd/pyepics which are bsui container deps, not robot container deps. Either remove them or document that pixi is for development tools only.

### 20. Consider Single-Node Architecture
**Impact**: Simplifies deployment, reduces IPC overhead
**Effort**: Large

Currently 8+ separate action server nodes + orchestrator. For the MCP use case, a single composite node could reduce startup time and IPC overhead. However, the multi-node architecture enables independent testing and is good for traditional ROS use.
