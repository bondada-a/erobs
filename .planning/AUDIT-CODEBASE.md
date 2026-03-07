# Codebase Audit
*Date: 2026-03-06 | Branch: humble-experimental*

## Summary

Audited every source file in the repository. The beambot core package is well-architected and clean. Main issues: duplicated detection code between zivid.py and erobs_mcp_server.py, legacy/demo packages that add complexity, and some stale configs. The MCP approach is the right direction but the MCP server currently duplicates rather than reuses the existing camera module.

---

## 1. beambot (Main Package) - GOOD

### Architecture: Clean 3-tier pattern
```
orchestrator.py → action_servers/ → stages/ → MTC
                                  → camera/  → Zivid SDK
```

### action_servers/

| File | Status | Notes |
|------|--------|-------|
| `base_action_server.py` | CLEAN | Good base class, proper threading, error handling |
| `orchestrator.py` (1063 lines) | GOOD | Well-structured, proper pause/resume, batch planning |
| `move_to_server.py` | CLEAN | Thin wrapper, delegates to stages |
| `end_effector_server.py` | CLEAN | Thin wrapper |
| `pick_place_server.py` | CLEAN | Thin wrapper |
| `tool_exchange_server.py` | CLEAN | Thin wrapper |
| `vision_server.py` | CLEAN | Handles both VisionMoveTo and VisionScan actions |
| `vision_pick_place_server.py` | CLEAN | Good camera config loading pattern |
| `pipettor_server.py` | CLEAN | Thin wrapper |

### stages/

| File | Status | Notes |
|------|--------|-------|
| `base_stages.py` | CLEAN | Good MTC utilities, proper module-level rclcpp init |
| `move_to_stages.py` | CLEAN | Supports relative, Cartesian, joint, SRDF targets |
| `end_effector_stages.py` | CLEAN | Supports batch add_to_task pattern |
| `pick_place_stages.py` | CLEAN | Two modes: single-task and split-for-settle |
| `tool_exchange_stages.py` | CLEAN | Proper state validation |
| `vision_stages.py` (~800 lines) | LARGE but OK | Complex but necessary - handles detection, TF, cache, IK |
| `vision_pick_place_stages.py` | CLEAN | Good hybrid vision-pick/hardcoded-place pattern |
| `pipettor_stages.py` | CLEAN | Direct action client (not MTC-based) |

### camera/

| File | Status | Notes |
|------|--------|-------|
| `__init__.py` | CLEAN | Good factory pattern |
| `zivid.py` (~784 lines) | OK but DUPLICATED | Circle/contour/Hough detection code is duplicated in erobs_mcp_server.py |

### mcp/

| File | Status | Notes |
|------|--------|-------|
| `erobs_mcp_server.py` (910 lines) | GOOD but DUPLICATED | Detection functions copied from zivid.py instead of importing. Also has independent ROS2Bridge (correct for MCP standalone use) |
| `__init__.py` | CLEAN | Empty |

**Key Issue**: `erobs_mcp_server.py` contains copy-pasted detection functions from `camera/zivid.py`:
- `_detect_hough_circles` - identical
- `_detect_contours_in_image` - identical
- `_sort_contours_reading_order` - identical
- `_get_3d_position` - identical
- `CircleDetectionParams` / `ContourDetectionParams` - identical

This means bug fixes must be applied in two places. These should be extracted to a shared module.

### batch_planner.py
- **Status**: CLEAN - Well-extracted from orchestrator, clear batching logic

### pointcloud_relay.py
- **Status**: CLEAN - Good Open3D voxel downsampling with fallback

### octomap_to_planning_scene.py
- **Status**: CLEAN - Proper octomap bridge with throttling

### scripts/

| File | Status | Notes |
|------|--------|-------|
| `beambot_client.py` | CLEAN | CLI task sender |
| `live_stitcher.py` | UNKNOWN | Point cloud stitching script |
| `stitch_from_bag.py` | UNKNOWN | Bag file stitching |
| `test_contour_detection.py` | OK | Test/debug script |
| `test_pointcloud_stability.py` | OK | Test/debug script |
| `test_wafer_detection.py` | OK | Test/debug script |

### config/

| File | Status | Notes |
|------|--------|-------|
| `default_beamline.yaml` | GOOD | Clean beamline config with typo ("cnfig" line 3) |
| `beamline_scene.yaml` | GOOD | Collision obstacles, some commented-out values suggest tuning |
| `grippers.yaml` | UNKNOWN | May be superseded by default_beamline.yaml |
| `ur3e_beamline.yaml` | OK | UR3e-specific config |
| `zivid_*.yml` | OK | Camera settings files |
| `scene_capture.yml` | OK | Zivid capture settings |

### launch/

| File | Status | Notes |
|------|--------|-------|
| `beambot_bringup.launch.py` | GOOD | Clean launch with conditional nodes, proper arg handling |
| `octomap_test.launch.py` | OK | Test launch for octomap pipeline |

---

## 2. beambot_interfaces

| File | Status | Notes |
|------|--------|-------|
| `MTCExecution.action` | CLEAN | Main orchestrator interface |
| `MoveToAction.action` | CLEAN | Supports relative, Cartesian, joint targets |
| `EndEffectorAction.action` | CLEAN | |
| `PickPlaceAction.action` | CLEAN | |
| `ToolExchangeAction.action` | CLEAN | |
| `VisionMoveToAction.action` | CLEAN | Supports multi-position scan |
| `VisionScanAction.action` | CLEAN | Batch scan interface |
| `VisionPickPlaceAction.action` | CLEAN | Hybrid vision pick/place |
| `PipettorAction.action` | CLEAN | |

No dead interfaces found. All are actively used.

---

## 3. bluesky_ros - PARTIALLY STALE

| File | Status | Notes |
|------|--------|-------|
| `mtc_ophyd_device.py` | FUNCTIONAL but BASIC | Works but has hardcoded status codes (4=SUCCEEDED, etc.) |
| `mtc_ophyd_device_async.py` | STALE | Async version, likely unused |
| `simple_mtc_bluesky.py` | STALE | Simple demo, likely unused |
| `task_builder.py` | FUNCTIONAL | Good task building API, but references `src/cms/tasks/complete_sequence.json` |
| `archive/` | STALE | Old PDF beamline Bluesky code |

**Issues**:
- `mtc_ophyd_device.py:112`: Hardcoded GoalStatus constants (4, 5, 6) instead of using GoalStatus enum
- `task_builder.py` uses emojis in print statements (cosmetic)
- `task_builder.py:19` references `src/cms/tasks/complete_sequence.json` as default - CMS-specific

---

## 4. aruco_pose - STALE/LEGACY

| File | Status | Notes |
|------|--------|-------|
| `aruco_pose.cpp` | LEGACY | Standalone ArUco detection, superseded by beambot vision pipeline |
| `aruco_pose_fixed_cam.cpp` | LEGACY | Fixed camera ArUco, not needed with eye-in-hand |
| `redis_insert.py` | LEGACY | Redis integration, not used in current architecture |

**Recommendation**: Archive or remove. All ArUco detection is now handled by Zivid's native detection or the MCP server.

---

## 5. mtc_gui - FUNCTIONAL

| File | Status | Notes |
|------|--------|-------|
| `mtc_gui_client.py` | FUNCTIONAL | Main GUI client with task building, contour detection |
| `pose_editor.py` | FUNCTIONAL | Pose editing dialog |
| `poses_manager.py` | FUNCTIONAL | Pose YAML management |
| `save_current_pose_dialog.py` | FUNCTIONAL | Save pose dialog |

**Minor Issues**:
- GUI may not reflect MCP tools (designed for direct action server interaction)
- Some hardcoded detection parameters

---

## 6. pdf/pdf_beamtime - LEGACY

| File | Status | Notes |
|------|--------|-------|
| All .cpp/.hpp files | LEGACY | C++ MTC implementation, fully superseded by Python beambot |
| `pdf_beamtime_client.py` | LEGACY | Old Python client |
| `pdf_beamtime_fidpose_*.py` | LEGACY | Fiducial detection clients |

**Recommendation**: Archive. The entire pdf_beamtime package is the old C++ implementation that has been replaced by the Python beambot package.

---

## 7. demos/ - EDUCATIONAL

| Package | Status | Notes |
|---------|--------|-------|
| `hello_moveit` | OK | C++ MoveIt tutorial |
| `hello_moveit_interfaces` | OK | Tutorial interfaces |
| `hello_orchestrator` | OK | C++ orchestrator tutorial |
| `hello_orchestrator_interfaces` | OK | Tutorial interfaces |
| `hello_orchestrator_py` | OK | Python orchestrator tutorial - useful reference |
| `hello_orchestrator_py_interfaces` | OK | Tutorial interfaces |

**Note**: Demos are educational but add build time. Consider moving to a separate repo or making them optional.

---

## 8. custom-ur-descriptions - FUNCTIONAL

| Package | Status | Notes |
|---------|--------|-------|
| `ur5e_robot_description` | OK | URDF with Zivid mount, calibration data |
| `ur5e_moveit_configs/*` | OK | 4 MoveIt configs (standalone, hande, epick, pipettor) |
| `ur3e_hande_robot_description` | OK | UR3e config (secondary robot) |
| `ur3e_hande_moveit_config` | OK | UR3e MoveIt config |

**Minor**: `ur5e_robot_description/urdf/isaac_sim_joint_params.yaml` suggests Isaac Sim work that isn't documented.

---

## 9. end_effectors/ - EXTERNAL DEPS

| File | Status | Notes |
|------|--------|-------|
| `end_effectors.repos` | OK | References robotiq_hande_driver, ros2_epick_gripper, pipettor |
| `epick_config/` | OK | ePick launch configuration |

---

## 10. vision/ - EXTERNAL DEPS

| File | Status | Notes |
|------|--------|-------|
| `vision.repos` | OK | References zivid-ros, zed-ros2-wrapper |

**Note**: ZED wrapper is included but not used in current setup.

---

## 11. cms/ - TASK SEQUENCES

Contains 35+ JSON task sequence files for the CMS beamline. These are configuration, not code.

**Issues**:
- `beamline_test copy.json` - Space in filename (bad practice)
- Many test files that could be consolidated
- No schema validation for task JSON files

---

## 12. lix/ - EMPTY

Only contains `.gitkeep`. Placeholder for LIX beamline configurations.

---

## 13. Infrastructure Files

### Root Scripts

| File | Status | Notes |
|------|--------|-------|
| `start_mcp.sh` | GOOD | Launches rosbridge + beambot for MCP |
| `build.sh` | OK | Simple colcon build wrapper |
| `setup.sh` | OK | ROS2 workspace setup |
| `test.sh` | OK | Test runner |
| `start_ursim.sh` | OK | URSim launcher |

### .mcp.json
- **Status**: GOOD - Dynamic repo root detection, two MCP servers configured

### pixi.toml
- **Status**: OK but MINIMAL
- **Issue**: Lists bluesky/ophyd/pyepics/IPython as deps but these are for the bsui container, not the robot container. The `robostack-humble` channel is referenced but pixi isn't the primary build tool.

### .pre-commit-config.yaml
- **Status**: OK - Uses ruff for linting

### .github/
- **Status**: Unknown - Need to check CI workflows

### .devcontainer/
- **Status**: OK - VSCode devcontainer config

---

## 14. Code Duplication Summary

| Duplicated Code | Location 1 | Location 2 | Lines |
|-----------------|-----------|-----------|-------|
| `_detect_hough_circles` | `camera/zivid.py` | `mcp/erobs_mcp_server.py` | ~40 |
| `_detect_contours_in_image` | `camera/zivid.py` | `mcp/erobs_mcp_server.py` | ~50 |
| `_sort_contours_reading_order` | `camera/zivid.py` | `mcp/erobs_mcp_server.py` | ~30 |
| `_get_3d_position` | `camera/zivid.py` | `mcp/erobs_mcp_server.py` | ~40 |
| `CircleDetectionParams` | `camera/zivid.py` | `mcp/erobs_mcp_server.py` | ~10 |
| `ContourDetectionParams` | `camera/zivid.py` | `mcp/erobs_mcp_server.py` | ~15 |
| `_make_move_to_named_stage` | `pick_place_stages.py` | `vision_pick_place_stages.py` | ~15 |
| Camera config loading | `vision_server.py` | `vision_pick_place_server.py` | ~15 |

**Total duplicated**: ~215 lines

---

## 15. Dead Code / Unused Items

| Item | Location | Reason |
|------|----------|--------|
| `aruco_pose` package | `src/aruco_pose/` | Superseded by Zivid native detection |
| `pdf_beamtime` package | `src/pdf/` | Superseded by Python beambot |
| `bluesky_ros/archive/` | `src/bluesky_ros/archive/` | Old Bluesky integration |
| `mtc_ophyd_device_async.py` | `src/bluesky_ros/` | Unused async variant |
| `simple_mtc_bluesky.py` | `src/bluesky_ros/` | Unused demo |
| ZED wrapper in vision.repos | `src/vision/vision.repos` | Not used in current setup |
| `grippers.yaml` | `src/beambot/config/` | May be superseded by beamline config |
| `zivid_3d_settings_old.yml` | `src/beambot/config/` | Old settings |
| `isaac_sim_joint_params.yaml` | `ur5e_robot_description/urdf/` | Isaac Sim leftover |

---

## 16. Hardcoded Values That Should Be Configurable

| Value | Location | Current | Recommendation |
|-------|----------|---------|----------------|
| Velocity/accel scaling | `base_stages.py:44-45` | 0.2 (20%) | Make ROS parameter |
| Capture timeout | `erobs_mcp_server.py:389` | 30s | Already parameterized |
| MoveIt ready timeout | `moveit_lifecycle_manager.py:112` | 45s | Make configurable |
| UR secondary port | `moveit_lifecycle_manager.py:39` | 30002 | OK as constant |
| Dock spacing | `tool_exchange_stages.py:11` | 0.1524m (6in) | Move to beamline config |
| Default save paths | `erobs_mcp_server.py:68-69` | /tmp/erobs_*.jpg | OK for now |

---

## 17. TODO/FIXME/HACK Comments

None found in the codebase. This is either very clean or comments were stripped.

---

## 18. Security Considerations

| Issue | Location | Severity |
|-------|----------|----------|
| Raw socket to robot | `moveit_lifecycle_manager.py:364` | LOW - Expected for UR protocol |
| File path in construct_goal_message | `mtc_ophyd_device.py:54` | LOW - No path traversal risk in local context |
| MCP stdio transport | `erobs_mcp_server.py:904` | LOW - Local only |

No critical security issues found. The codebase operates in a controlled beamline environment.
