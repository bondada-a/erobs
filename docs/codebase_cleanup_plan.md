# EROBS Codebase Cleanup Plan

> Generated 2026-03-30 from comprehensive 5-agent analysis of the entire workspace.
> **Do NOT apply changes blindly** — test each section on hardware before committing.

## Change Log

| Date | Item | Action | Details |
|------|------|--------|---------|
| 2026-03-30 | §1.1 | **KEPT** | `run_with_gripper_settle()` is NOT dead — needed for grippers that report "done" before physically closing. MTC executes stages sequentially but gripper controllers return early. The 3-task split provides natural settling delay via inter-task planning time. Wire up as option later. |
| 2026-03-30 | §1.2 | **KEPT** | `_set_tool_voltage_via_io()` is NOT dead — called at lines 1056 and 1107 during tool exchange (dock: 0V to release QC, load: restore voltage). Agent analysis was incorrect. |
| 2026-03-30 | §1.3 | **DONE** | Removed `_ensure_tf()` no-op and its 2 call sites from `move_to_stages.py`. TF buffer/listener initialized eagerly in `__init__`, so lazy-init guard was unnecessary. |
| 2026-03-30 | §1.4 | **KEPT** | `vision_objects` config loading is NOT dead — `_add_collision_object_for_tag()` is called at line 580 during marker detection. `vision_objects.json` exists with a sample_bar entry. Working collision object system, just lightly populated. |
| 2026-03-30 | §1.5 | **KEPT** | `hello_orchestrator_py` demo accurately mirrors beambot's architecture (JSON dispatch → orchestrator → action servers → stages). Useful onboarding tutorial — pattern matches, just simplified. |
| 2026-03-30 | §1.6 | **KEPT** | `mtc_gui` is NOT incomplete — it's a fully functional ~1950-line Tkinter app with task editor, live camera view, ArUco/contour detection, pose management, pause/resume, and correct MTCExecution action client. The `ROS2_AVAILABLE` guard is defensive coding, not abandonment. |
| 2026-03-30 | §2.1 | **SKIPPED** | Pose/constraint/gripper-state parsing duplication is trivial 3-line boilerplate. Extracting to helpers adds indirection for minimal gain — each instance is clear and readable inline. |
| 2026-03-30 | §2.2 | **DEFERRED** | Vision config loading duplication exists between `vision_server.py` and `vision_pick_place_server.py`. Depends on outcome of #47 — if `vision_pick_place` is consolidated, duplication resolves itself. |
| 2026-03-30 | §2.3 | **DEFERRED** | Gripper stage creation duplication between `pick_place_stages.py` and `vision_pick_place_stages.py`. Same dependency on #47. |
| 2026-03-30 | §3.1 | **ISSUE #48** | MoveIt config consolidation — 5 packages → 1 parameterized package. Confirmed feasible: SRDF supports xacro, MoveItConfigsBuilder supports mappings, OpaqueFunction handles Humble's eager load_yaml(). |
| 2026-03-30 | §4.1-4.2 | **COMMENTED on #38** | 2FG7 config bugs (wrong joint names in initial_positions.yaml and joint_limits.yaml) added as comment on existing 2FG7 MoveIt issue. Fix when 2FG7 integration resumes. |
| 2026-03-30 | §4.3 | **DONE** | Added `python3-pymodbus` to onrobot_2fg7_driver/package.xml, `python3-serial` to pipettor/pipette_driver/package.xml. |
| 2026-03-30 | §4.4 | **DONE** | Deleted misnamed `ur_standalone_isaac.usd` and `ur_with_zivid_epick_isaac.usd` from `hande_isaac/` dir — copy-paste artifacts that could confuse MCP context. Kept correct `ur_with_zivid_hande_isaac.usd`. |
| 2026-03-30 | §5.1 | **SKIPPED** | IK frame mapping is a URDF implementation detail, not user-facing config. Fixed per gripper, changes only if URDF changes. Hardcoded dict is correct. |
| 2026-03-30 | §5.2 | **SKIPPED** | Tool exchange distances are physical hardware constants (dock spacing, approach/retreat). Fixed by hardware design. |
| 2026-03-30 | §5.3 | **SKIPPED** | Timeouts already configurable — declared as ROS parameters with defaults, overridable via `--ros-args -p timeout.moveto:=60.0`. |
| 2026-03-30 | §5.4 | **SKIPPED** | XACRO mounting offsets are calibrated physical measurements. Fixed by hardware. |
| 2026-03-30 | §5.5 | **SKIPPED** | Detection params already overridable via MCP tool args and direct constructor. Dataclass defaults are reasonable fallbacks. |
| 2026-03-30 | §6 | **DEFERRED** | Build & repo hygiene — housekeeping items, skipped for now. |
| 2026-03-30 | §7 | **DEFERRED** | Stale root-level artifacts — skipped for now. |
| 2026-03-30 | §8.1 | **SKIPPED** | Polling in orchestrator is necessary (can't spin from callback thread). 10ms poll is negligible CPU. |
| 2026-03-30 | §8.2-8.8 | **SKIPPED** | Settle time (needs further testing), calibration comments (cosmetic), add_to_task interface (by design), 2FG7 test scripts (active dev), CAD models (upstream), __pycache__ (already gitignored), pipettor monolith (upstream). |
| 2026-03-30 | §9-10 | **DEFERRED** | Docs cleanup and long-term refactoring — deferred. |

---

## Table of Contents

1. [Priority Summary](#priority-summary)
2. [Critical: Dead Code & Stale Packages](#1-critical-dead-code--stale-packages)
3. [High: Code Duplication](#2-high-code-duplication)
4. [High: MoveIt Config Duplication](#3-high-moveit-config-duplication)
5. [High: Configuration Bugs](#4-high-configuration-bugs)
6. [Medium: Hardcoded Values → Config](#5-medium-hardcoded-values--config)
7. [Medium: Build & Repo Hygiene](#6-medium-build--repo-hygiene)
8. [Medium: Stale Root-Level Artifacts](#7-medium-stale-root-level-artifacts)
9. [Low: Code Quality Improvements](#8-low-code-quality-improvements)
10. [Low: Documentation Cleanup](#9-low-documentation-cleanup)
11. [Optional: Long-Term Refactoring](#10-optional-long-term-refactoring)

---

## Priority Summary

| Priority | Count | Category |
|----------|-------|----------|
| CRITICAL | 6 | Dead code, stale packages, config bugs |
| HIGH | 8 | Duplications, missing dependencies |
| MEDIUM | 10 | Hardcoded values, repo hygiene, artifacts |
| LOW | 8 | Code quality, docs, test coverage |
| OPTIONAL | 5 | Long-term architectural improvements |

---

## 1. CRITICAL: Dead Code & Stale Packages

### 1.1 Remove `run_with_gripper_settle()` — Dead Alternative Implementation
- **File:** `src/beambot/beambot/stages/pick_place_stages.py` lines 163-333
- **What:** 170-line alternative pick-and-place that splits into 3 MTC tasks. Never called anywhere.
- **Why remove:** Dead code path that must be maintained with every change to pick_place logic.
- **Risk:** None — grep confirms zero callers.

### 1.2 Remove `_set_tool_voltage_via_io()` — Unused Method
- **File:** `src/beambot/beambot/action_servers/orchestrator.py` lines 987-1018
- **What:** ROS service-based tool voltage setter. The orchestrator uses `MoveItLifecycleManager._set_tool_voltage()` (raw socket) instead.
- **Why remove:** Dead code, confusing to have two approaches.
- **Risk:** None — grep confirms zero callers.

### 1.3 Remove `_ensure_tf()` No-Op
- **File:** `src/beambot/beambot/stages/move_to_stages.py` lines 42-43
- **What:** Empty method with docstring "No-op kept for backward compatibility." Called at lines 59 and 88.
- **Why remove:** No-ops add confusion. Remove the method and its two call sites.
- **Risk:** None.

### 1.4 Remove Unused `vision_objects` Config Loading
- **File:** `src/beambot/beambot/stages/vision_stages.py` lines 139-143, 171-198
- **What:** Loads `vision_objects.json` into `self._object_database` but the database is never queried by any method.
- **Why remove:** Dead initialization path — the collision object management system was never completed.
- **Risk:** None — only loaded, never used.

### 1.5 Archive or Remove `demos/hello_orchestrator_py`
- **Files:** `src/demos/hello_orchestrator_py/` + `src/demos/hello_orchestrator_py_interfaces/`
- **What:** Educational demo using old `OrchestratorTask` action pattern. The real system uses `MTCExecution`.
- **Why remove:** Outdated architecture pattern. New developers following this demo would learn the wrong approach.
- **Action:** Move to `docs/archive/` or delete entirely.

### 1.6 Archive or Remove `mtc_gui/`
- **Files:** `src/mtc_gui/`
- **What:** Incomplete TK-based GUI for task building. Abandoned mid-development (has `ROS2_AVAILABLE = False` fallback guards).
- **Why remove:** Non-functional, creates false impression of GUI support.
- **Action:** Move to `docs/archive/` or delete entirely.

---

## 2. HIGH: Code Duplication

### 2.1 Duplicated Pose/Constraint/Gripper-State Parsing
- **Files affected:**
  - `move_to_stages.py` (line 237-241)
  - `tool_exchange_stages.py` (line 68-71)
  - `pick_place_stages.py` (lines 71-73, 82-84)
  - `vision_pick_place_stages.py` (lines 103-105, 108-111, 116-118)
  - `vision_stages.py` (line 338-344)
- **What:** Every stage class re-implements the same `parse_poses()` → check None → return error pattern, plus identical `gripper_states_json` parsing and `constraints_json` parsing.
- **Fix:** Add utility methods to `BaseStages`:
  ```python
  def _parse_goal_poses(self, goal) -> tuple[dict | None, str | None]:
  def _parse_goal_constraints(self, goal) -> tuple[dict | None, str | None]:
  def _parse_goal_gripper_states(self, goal) -> tuple[dict | None, str | None]:
  ```

### 2.2 Duplicated Vision Config Loading
- **Files:**
  - `src/beambot/beambot/action_servers/vision_server.py` (lines 56-80)
  - `src/beambot/beambot/action_servers/vision_pick_place_server.py` (lines 32-55)
- **What:** Identical code loading camera config from beamline YAML (camera type, frame names, etc.)
- **Fix:** Extract to a shared `load_vision_config(beamline_config)` utility function.

### 2.3 Duplicated Gripper Stage Creation
- **Files:**
  - `pick_place_stages.py` (lines 92-96, 115-119, 147-151)
  - `vision_pick_place_stages.py` (similar pattern)
- **What:** Near-identical `make_gripper_stage()` calls with same error handling.
- **Fix:** Extract to shared helper or add to `BaseStages`.

---

## 3. HIGH: MoveIt Config Duplication

### 3.1 Five Nearly-Identical MoveIt Config Packages
- **Location:** `src/custom-ur-descriptions/ur5e_moveit_configs/`
- **Packages:** `ur_standalone`, `ur_zivid_epick`, `ur_zivid_hande`, `ur_zivid_pipettor`, `ur_zivid_2fg7`
- **Identical across all 5:**
  - `ompl_planning.yaml` (100%)
  - `pilz_industrial_motion_planner_planning.yaml` (100%)
  - `pilz_cartesian_limits.yaml` (100%)
  - `kinematics.yaml` (100%)
  - `rviz/view_robot_mtc.rviz` (100%)
  - `.setup_assistant` (100%)
  - CMakeLists.txt structure (100%)
- **Only differ in:**
  - `joint_limits.yaml` — gripper joint entries
  - `moveit_controllers.yaml` — gripper controller definitions
  - `ur.ros2_control.xacro` — gripper joint in mock_components
  - `ur.srdf` — gripper groups, end effectors, group states
  - `initial_positions.yaml` — gripper initial position
  - `robot_bringup.launch.py` — gripper-specific args, payload
- **Fix approach:** Create a single `ur_zivid_base_moveit_config` package with shared files, and gripper-specific overlay packages that only contain their diffs. Alternatively, parameterize a single package with gripper selection at launch time.
- **Note:** This is a significant refactor — plan carefully and test each gripper config.

---

## 4. HIGH: Configuration Bugs

### 4.1 2FG7 `initial_positions.yaml` Uses Wrong Joint Name
- **File:** `ur5e_moveit_config/config/2fg7/initial_positions.yaml`
- **Bug:** References `robotiq_hande_left_finger_joint: 0.0` — this is the Hand-E joint name!
- **Fix:** Change to `2fg7_left_finger_joint: 0.0`

### 4.2 2FG7 `joint_limits.yaml` Missing Gripper Joints
- **File:** `ur5e_moveit_config/config/2fg7/joint_limits.yaml`
- **Bug:** Contains only arm joints (49 lines). Missing entries for `2fg7_left_finger_joint` and `2fg7_right_finger_joint`.
- **Fix:** Add joint limit entries matching the URDF constraints.

### 4.3 Missing Dependency Declarations
- **File:** `src/end_effectors/onrobot_2fg7_driver/package.xml`
  - Missing: `<exec_depend>pymodbus</exec_depend>`
- **File:** `src/end_effectors/pipettor/pipette_driver/package.xml`
  - Missing: `<exec_depend>pyserial</exec_depend>`
- **Impact:** Runtime ImportError on fresh installs.

### 4.4 Isaac URDF Misnamed Files
- **Location:** `ur5e_robot_description/urdf/ur_with_zivid_hande_isaac/`
- **Bug:** Contains `ur_standalone_isaac.usd` and `ur_with_zivid_epick_isaac.usd` — wrong filenames (copy-paste error).
- **Fix:** Rename to correct hande variants, or delete if Isaac is not actively used.

---

## 5. MEDIUM: Hardcoded Values → Config

### 5.1 IK Frame Mapping Hardcoded in Orchestrator
- **File:** `orchestrator.py` lines 975-980
- **What:** `_GRIPPER_IK_FRAMES = {"epick": "epick_tip", "hande": "robotiq_hande_end", ...}`
- **Fix:** Move to `default_beamline.yaml` under each gripper's config section.

### 5.2 Tool Exchange Distances Hardcoded
- **File:** `tool_exchange_stages.py`
- **What:** `DOCK_SPACING_METERS = 0.1524`, approach/retreat distances (0.2m, 0.15m)
- **Fix:** Move to beamline config under `tool_exchange` section.

### 5.3 Timeouts Hardcoded
- **File:** `orchestrator.py` lines 69-76
- **What:** `DEFAULT_TIMEOUTS` dict, plus scattered 5.0s/10.0s/2.0s/0.5s throughout.
- **Fix:** Move to beamline config, allow per-operation override.

### 5.4 Gripper Mounting Offsets Hardcoded in XACRO
- **Files:** All `ur_with_zivid_*.xacro` files
- **What:** `xyz="0 -0.03235 0"` (tool block offset), `xyz="0 0 0.049"` (Zivid mount)
- **Fix:** Parameterize with xacro args and defaults. Single source of truth.

### 5.5 Detection Parameters in Dataclass Defaults
- **File:** `detection/params.py`
- **What:** `CircleDetectionParams` and `ContourDetectionParams` have hardcoded defaults.
- **Fix:** Load from beamline config, use dataclass defaults as fallback only.

---

## 6. MEDIUM: Build & Repo Hygiene

### 6.1 Build Artifacts Checked Into `ros2_epick_gripper`
- **Location:** `src/end_effectors/ros2_epick_gripper/{build,install,log}/`
- **Size:** ~1.8 MB of CMake artifacts, installed packages, build logs
- **Fix:** Delete directories, add to `.gitignore`.

### 6.2 Pipettor Submodule Not in `.gitmodules`
- **Location:** `src/end_effectors/pipettor/`
- **What:** Has `.git/` directory, tracked as gitlink (mode 160000), but NOT in `.gitmodules`. Locally modified.
- **Risk:** Changes lost on re-clone.
- **Fix:** Either add to `.gitmodules` or document the local modifications and pin commit in `end_effectors.repos`.

### 6.3 `build.sh` References Removed Packages
- **File:** `build.sh`
- **What:** References `ur3e_hande_robot_description` (renamed to `ur5e_robot_description`) and `pdf_beamtime_interfaces` (removed).
- **Fix:** Update package names.

### 6.4 Stale `lix/` Placeholder
- **Location:** `src/lix/`
- **What:** Empty directory with only `.gitkeep`.
- **Fix:** Delete or add README explaining planned purpose.

---

## 7. MEDIUM: Stale Root-Level Artifacts

### 7.1 YOLO Model File at Root
- **File:** `yolov8n.pt` (~540 MB)
- **Fix:** Move to appropriate data directory or add to `.gitignore` if generated.

### 7.2 TF Tree Visualization Files
- **Files:** `frames_2026-03-06_15.20.32.gv`, `frames_2026-03-06_15.20.32.pdf`
- **Fix:** Delete (regenerable via `ros2 run tf2_tools view_frames`).

### 7.3 Orphaned `package-lock.json`
- **File:** `package-lock.json` at workspace root
- **Fix:** Delete (no Node.js in this workspace).

### 7.4 Unused Zivid Mesh
- **File:** `ur5e_robot_description/meshes/zivid/zivid_custom_mount.dae` (1.7 MB)
- **What:** Not referenced by any xacro file.
- **Fix:** Delete after confirming it's not referenced.

---

## 8. LOW: Code Quality Improvements

### 8.1 Polling-Based Blocking in Orchestrator
- **File:** `orchestrator.py` lines 904-910, 922-929
- **What:** `while not done: time.sleep(0.01)` instead of futures/events.
- **Fix:** Use `threading.Event` or `asyncio` for proper blocking.

### 8.2 Settle Time Feature Disabled
- **File:** `vision_stages.py` line 70
- **What:** `DEFAULT_SETTLE_TIME = 0.0` with comment "Disabled for now - testing TF timestamp fix"
- **Fix:** Either complete the feature or remove the dead parameter.

### 8.3 Calibration History in XACRO Comments
- **File:** `ur5e_robot_description/urdf/zivid_camera_mount.xacro` lines 30-51
- **What:** 6 old calibration values commented out (dates from Oct 2025 to Mar 2026).
- **Fix:** Keep only the current calibration. Move history to a calibration log file or git history.

### 8.4 Inconsistent `add_to_task()` Interface
- **What:** `MoveToStages` and `EndEffectorStages` implement `add_to_task()` for batching. `PickPlaceStages`, `VisionStages` etc. do not.
- **Fix:** Either document that batching only applies to simple stages, or extend the interface.

### 8.5 Redundant Test Script
- **File:** `src/end_effectors/onrobot_2fg7_driver/test_scripts/test_modbus_direct.py`
- **What:** Lower-level duplicate of `test_modbus_client.py`.
- **Fix:** Remove (keep `test_modbus_client.py`).

### 8.6 CAD Models in `robotiq_hande_description`
- **Location:** `src/end_effectors/robotiq_hande_description/cad_models/` (6.4 MB)
- **What:** FreeCAD source files not needed at runtime.
- **Fix:** Consider removing from this branch (keep in upstream repo).

### 8.7 `__pycache__` in Source Tree
- **Location:** `src/beambot/mcp/__pycache__/`, `src/beambot/test/__pycache__/`, `src/end_effectors/onrobot_2fg7_driver/onrobot_2fg7_driver/__pycache__/`
- **Fix:** Add to `.gitignore` and delete tracked instances.

### 8.8 Pipettor Driver Monolith
- **File:** `src/end_effectors/pipettor/pipette_driver/pipette_driver/pipette_driver_node.py` (742 lines)
- **Fix:** Consider splitting into modules (driver, action servers, utilities). Lower priority since it's a vendored package.

---

## 9. LOW: Documentation Cleanup

### 9.1 CMS `beamtime_poses.yaml` References Non-Existent File
- **File:** `src/cms/beamtime_poses.yaml`
- **What:** Header states "For conflicts, values from all_positions.json (latest) are used" — file doesn't exist.
- **Fix:** Remove reference or archive the file with clear "historical only" header.

### 9.2 CMS `experiments.md` References Old API
- **File:** `src/cms/experiments.md`
- **What:** References old `vision_moveto` syntax, old pose names.
- **Fix:** Update to match current `vision_target` framework.

### 9.3 CMS `spincoat_to_hotplate.json` Uses Old Format
- **File:** `src/cms/tasks/spincoat_to_hotplate.json`
- **What:** Missing `start_gripper` field, uses old task format.
- **Fix:** Update to current `MTCExecution` format or move to `docs/archive/`.

### 9.4 Isaac URDF Status
- **Location:** `ur5e_robot_description/urdf/*_isaac*`
- **What:** Isaac Sim URDF variants exist for standalone/epick/hande/pipettor but NOT 2fg7.
- **Fix:** If Isaac is actively used, add 2fg7 variant. If not, document as stale or remove.

---

## 10. OPTIONAL: Long-Term Refactoring

### 10.1 Structured Error Types
- **Current:** All stages return `Optional[str]` (None=success, string=error).
- **Improvement:** Custom exception hierarchy (`PlanningError`, `ExecutionError`, `DetectionError`) for better type safety and programmatic error handling.

### 10.2 Unified Stage Configuration
- **Current:** Default values scattered across `base_stages.py`, `vision_stages.py`, `zed.py`, `orchestrator.py`.
- **Improvement:** Single `StageConfig` object loaded from beamline YAML, passed to all stages.

### 10.3 Proper Git Submodule Management
- **Current:** Vendored repos cloned in-place, managed via `.repos` files.
- **Improvement:** Convert to proper `.gitmodules` for repos that are modified locally (especially `pipettor`).

### 10.4 MoveIt Config Parameterization
- **Current:** 5 separate MoveIt config packages with ~70% identical files.
- **Improvement:** Single parameterized package or base + overlay pattern.
- **Risk:** High — MoveIt config is sensitive. Requires thorough testing per gripper.

### 10.5 Integration Test Suite
- **Current:** Unit tests for base_stages, orchestrator parsing, batch planner, detection algorithms.
- **Gap:** No integration tests for complete workflows, action servers, or vision pipeline.
- **Improvement:** Add mock-based integration tests for key workflows.

---

## Execution Order

Recommended sequence for implementing changes:

```
Phase 1 — Safe Deletions (no behavior change)
  1.1  Remove dead code (§1.1-1.4)
  1.2  Archive stale packages (§1.5-1.6)
  1.3  Clean root artifacts (§7.1-7.4)
  1.4  Clean build artifacts (§6.1, §8.7)
  → Test: colcon build + basic MCP operation

Phase 2 — Bug Fixes (correctness)
  2.1  Fix 2FG7 config bugs (§4.1-4.2)
  2.2  Add missing dependencies (§4.3)
  2.3  Fix Isaac URDF names (§4.4)
  2.4  Fix build.sh references (§6.3)
  → Test: 2FG7 MoveIt launch + planning

Phase 3 — Deduplication (maintainability)
  3.1  Extract shared parsing to BaseStages (§2.1)
  3.2  Extract shared vision config loading (§2.2)
  3.3  Extract shared gripper stage creation (§2.3)
  → Test: Full pick-and-place + vision workflows

Phase 4 — Configuration Externalization (flexibility)
  4.1  Move IK frames to config (§5.1)
  4.2  Move tool exchange params to config (§5.2)
  4.3  Move timeouts to config (§5.3)
  → Test: All gripper operations + tool exchange

Phase 5 — Structural Improvements (optional)
  5.1  MoveIt config consolidation (§3.1 / §10.4)
  5.2  Submodule management (§6.2 / §10.3)
  5.3  Documentation updates (§9.1-9.4)
  → Test: Full system regression
```

---

## Files Scanned

Analysis covered 609 source files across:
- `src/beambot/` — Core orchestrator, stages, action servers, MCP server, detection, camera
- `src/custom-ur-descriptions/` — URDF descriptions, 5 MoveIt config packages
- `src/end_effectors/` — 9 gripper packages (epick, hande, pipettor, 2fg7, serial)
- `src/demos/`, `src/bluesky_ros/`, `src/mtc_gui/`, `src/lix/` — Peripheral packages
- `src/cms/` — Beamline config, poses, experiments
- `src/beambot_interfaces/` — 9 ROS action definitions
- `src/vision/` — Zivid, ZED, calibration tools
- Root-level scripts, build configs, documentation
