# EROBS Codebase Cleanup Log

**Date:** 2026-01-31  
**Branch:** `refactor/codebase-cleanup`  
**Author:** Roc 🦖 (overnight cleanup for Rocky)

---

## Summary

This document logs all changes made during the codebase cleanup, and lists items flagged for discussion.

---

## ✅ Changes Made

### 1. Root-Level File Organization

**Problem:** Several files at repo root that don't belong there (PDFs, screenshots, debug output).

**Actions:**
| File | Action | Reason |
|------|--------|--------|
| `Contour Detection_screenshot_09.01.2026.png` | → `docs/images/` | Screenshot belongs with docs |
| `Facile_Integration_of_Robots_into...pdf` | → `docs/references/` | Reference paper |
| `d5dd00036j.pdf` | → `docs/references/` | Reference paper (DOI-style name) |
| `frames_2025-12-18_14.04.48.gv` | → `docs/` | Debug output, useful for reference |
| `frames_2025-12-18_14.04.48.pdf` | → `docs/` | Debug output, useful for reference |
| `notes.md` | → `docs/development_notes.md` | Development notes belong in docs |
| `send_pose_goal.py` | → `scripts/` | Utility script belongs in scripts |

### 2. Gitignore Updates

**Problem:** Should prevent future accumulation of debug files at root.

**Actions:**
- Added patterns for common debug outputs (*.gv, frame dumps, etc.)
- Added vision packages (vcs-imported)
- Added pipettor (vcs-imported)
- Ignore ROS bag recordings

### 3. License Declarations

**Problem:** Many package.xml and setup.py files had "TODO: License declaration" placeholder.

**Actions:**
- Updated 8 package.xml files with `BSD-3-Clause` license
- Updated `mtc_gui/setup.py` with proper license and maintainer
- Consistent with repo LICENSE file

**Files fixed:**
- `src/aruco_pose/package.xml`
- `src/beambot/package.xml`
- `src/beambot_interfaces/package.xml`
- `src/demos/hello_moveit/package.xml`
- `src/demos/hello_moveit_interfaces/package.xml`
- `src/mtc_gui/package.xml`
- `src/mtc_gui/setup.py`
- `src/pdf/pdf_beamtime/package.xml`
- `src/pdf/pdf_beamtime_interfaces/package.xml`

---

## 📁 Archive Folders

The following archive folders exist and contain old code:

### `src/bluesky_ros/archive/pdf/`
Contains 4 Python files from earlier Bluesky-ROS integration:
- `ophyd_ros.py` - Old abstract base class for ROS action clients
- `pdf_beamtime.py` - Old PDF beamline implementation
- `pdf_beamtime_demo.py` - Demo script
- `re_demo.py` - RunEngine demo

**Status:** Left in place (clearly marked as archive). Current implementation in `src/bluesky_ros/` is the active version.

### `docker/archive/`
Contains old Dockerfile versions:
- `bsui/` - Old BSUI container
- `erobs-common-img/` - Old common image

**Status:** Left in place. These may be useful for reference.

---

## 🤔 Discussion Items

These items need Rocky's input before changing:

### 1. Demo vs Production Code Similarity

**Location:** 
- `src/demos/hello_orchestrator_py/hello_orchestrator_py/base_action_server.py`
- `src/beambot/beambot/action_servers/base_action_server.py`

**Observation:** Both files implement similar `BaseActionServer` classes. The beambot version has:
- Threading locks for concurrent access
- More detailed docstrings
- Slightly different error handling

**Question:** Is this intentional (demo is simplified for learning) or should the demo inherit from beambot's version?

Same pattern exists for `base_stages.py`.

### 2. TODO Comments in Code

Found several TODOs that may need attention:

```
src/demos/hello_moveit/src/pick_place_repeat_server.cpp:
  - Line 157: "take the goal handle and add cancellation infrastructure"
  - Line 254: "Add gripper close action here"
  - Line 302: "Add gripper open action here"

src/pdf/pdf_beamtime/src/pdf_beamtime_server.cpp:
  - Line 244: "Break these following statements to functions"

src/mtc_gui/setup.py:
  - Line 20: License still says "TODO: License declaration"
```

**Question:** Are these active work items or technical debt to track?

### 3. Empty Vision Directories

**Location:** `src/vision/`

These directories are empty (need `vcs import`):
- `zivid-ros/`
- `zed-ros2-wrapper/`
- `zivid-python-samples/`

**Question:** Should these be gitignored since they're external deps, or should there be a note about running `vcs import`?

### 4. `.planning` Directory

**Location:** `.planning/`

Contains what looks like AI-generated planning documents:
- `PROJECT.md`, `ROADMAP.md`, `STATE.md`
- `phases/` with research docs
- `codebase/` with architecture docs

**Question:** Is this still accurate/useful, or should it be archived/removed?

### 5. Test Scripts in beambot

**Location:** `src/beambot/scripts/`

Found test files that may be development artifacts:
- `test_contour_detection.py` (24KB)
- `test_pointcloud_stability.py` (11KB)
- `test_wafer_detection.py` (42KB)

**Question:** Are these active tests or should they move to a `tests/` folder?

### 6. Large Test Files

**Location:** `src/beambot/scripts/test_wafer_detection.py`

This file is 42KB - quite large for a test script. May contain embedded test data or be doing more than testing.

**Question:** Should this be refactored or is it intentional?

---

## 🔍 Deep Dive Findings (Iteration 1)

After reviewing each file in detail, here are specific code-level observations:

### Code Quality: Generally Excellent ✅

The codebase is well-structured with:
- Consistent docstrings and type hints in Python
- Good separation of concerns (stages, action servers, orchestrator)
- Smart abstractions (camera module, base classes)
- Comprehensive error handling
- Good use of dataclasses for structured data

### Specific Issues Found

#### 1. Debug Print Statement in Production Code

**File:** `src/beambot/beambot/stages/vision_stages.py` (Line ~545)
```python
print(f"  BASE_LINK pose: ({pose_base.pose.position.x*1000:.2f}, ...")
```
**Status:** ⚠️ Should be removed or converted to logger.debug()

#### 2. Commented Out Code Block

**File:** `src/beambot/beambot/stages/vision_stages.py` (in `_move_to_pose`)
```python
# OLD: Build MTC task with Cartesian planner
# task = self.create_task_template("Vision Move")
# cartesian = self.make_cartesian_planner()
# ...
```
**Status:** ⚠️ ~10 lines of commented code. Consider removing if IK trajectory approach is final.

#### 3. Hardcoded Magic Values

**File:** `src/beambot/beambot/stages/vision_stages.py`
```python
SAMPLE_OFFSET_X = 0.02   # 20mm offset
USE_IK_TRAJECTORY = True
IK_TRAJECTORY_DURATION = 2.0
```
**Question:** Should these be configurable via beamline config?

**File:** `src/beambot/beambot/stages/base_stages.py`
```python
VELOCITY_SCALING = 0.2
ACCELERATION_SCALING = 0.2
```
**Question:** Same question - should these be configurable?

#### 4. Typo in C++ Code

**File:** `src/pdf/pdf_beamtime/src/pdf_beamtime_server.cpp` (Line ~107)
```cpp
RCLCPP_ERROR(node_->get_logger(), "Incorrect interrput type");
                                            // ^ typo: "interrput"
```
**Status:** Minor typo, should be "interrupt"

### Architectural Observations

#### Strengths
1. **Orchestrator batching** - Smart optimization reducing planning overhead
2. **Pause/Resume** - Well-implemented with proper state management
3. **Controller recovery** - Handles UR driver socket drops gracefully
4. **Camera abstraction** - Clean interface for different camera types
5. **Multi-position scanning** - Sophisticated pose averaging for accuracy

#### Potential Improvements
1. **Test coverage** - Test scripts exist but no formal test framework (pytest)
2. **Configuration** - Some values hardcoded that could be in beamline config
3. **Demo/Production split** - Similar base classes in demos and beambot

---

## 🔍 Deep Dive Findings (Iteration 2)

Second pass focused on patterns and potential bugs:

### Pattern: Module-Level Initialization

**File:** `src/beambot/beambot/stages/base_stages.py`
```python
rclcpp.init()  # Module-level
_mtc_node = rclcpp.Node("beambot", _options)
```
**Observation:** Comment explains why this is safe, but worth being aware of. Each action server process has its own Python interpreter, so no conflict.

### Pattern: Retry Logic

**File:** `src/beambot/beambot/stages/vision_stages.py`
```python
DEFAULT_RETRY_COUNT = 10  # Number of retries after first attempt
DEFAULT_RETRY_DELAY = 0.5  # Seconds between retries
```
**Observation:** Good robust detection with configurable retries.

### Pattern: Tag Pose Cache

**File:** `src/beambot/beambot/stages/vision_stages.py`
```python
self._tag_pose_cache: Dict[int, PoseStamped] = {}
```
**Observation:** Smart caching for batch scans. Cache cleared explicitly via `clear_cache()`.

### Potential Improvement Areas

1. **Unified configuration** - Consider a single beamline config that includes:
   - Velocity/acceleration scaling
   - Sample offsets
   - IK trajectory duration
   - Retry counts

2. **Logging consistency** - Mix of `self.logger.info()` and `print()` in some files

3. **Type hints** - Most files have good type hints, but some C++ interfaces could use more Python typing

---

## 📊 Codebase Statistics

```
Python files:        83
C++ source files:    ~20 (src/)
Docker images:       10 (+ 2 archived)
ROS packages:        15+
Lines of code:       TBD
```

---

## 🗂️ Project Structure Overview

```
erobs/
├── src/
│   ├── beambot/          # Main package - robotic sample handling
│   ├── bluesky_ros/      # Bluesky-ROS integration
│   ├── custom-ur-descriptions/  # Robot URDF/configs
│   ├── end_effectors/    # Gripper drivers
│   ├── demos/            # Demo packages for learning
│   ├── pdf/              # PDF beamline specific
│   ├── cms/              # CMS beamline (active, has task JSONs)
│   ├── lix/              # LIX beamline (placeholder)
│   ├── vision/           # Vision system (external deps)
│   ├── aruco_pose/       # ArUco marker detection
│   ├── mtc_gui/          # MTC GUI client
│   └── beambot_interfaces/  # ROS interfaces
├── docker/               # Container definitions
├── docs/                 # Documentation + references
├── scripts/              # Utility scripts
└── .planning/            # Planning docs (AI-generated?)
```

---

## Next Steps (After Review)

1. Discuss items above with Rocky
2. Decide on archive folder fate
3. Consider consolidating demo/production base classes
4. Add proper test infrastructure if needed
5. Update README if structure changed significantly

---

*Generated during overnight cleanup session. Review and merge what makes sense!*

---

## 🔎 COMPREHENSIVE REVIEW (534 Files)

**Date:** 2026-01-31  
**Method:** 5 parallel sub-agents reviewed all file categories

Detailed reports in:
- `REVIEW_CONFIGS.md` (JSON/YAML)
- `REVIEW_ROBOT_DESCRIPTIONS.md` (XACRO/URDF/SRDF)
- `REVIEW_DOCUMENTATION.md` (Markdown/LaTeX)
- `REVIEW_BUILD_DEPLOY.md` (Docker/Scripts/CI)
- `REVIEW_INTERFACES.md` (ROS Actions/Services/USD)

---

### 🔴 CRITICAL ISSUES (Must Fix)

| Issue | Location | Impact |
|-------|----------|--------|
| **HandE mass off by 100,000x** | `ur3e_hande_robot_description/.../hande.yaml` | `86387` should be `0.86387` kg - physics simulation broken |
| **Kinematics timeout too low** | UR3e MoveIt config | `0.005s` vs `1.0s` for UR5e - IK failures |
| **Obstacle z-values typo** | `beamline_scene.yaml` | `z: 10.475` should be `z: 0.475` |
| **Tests disabled in CI** | `.github/actions/test/run.sh` | `./test.sh` is **commented out** - builds pass without testing |
| **Invalid SRDF virtual joint** | UR3e SRDF | Connects world to `right_finger` instead of `base_link` |
| **Broken README link** | Root `README.md` | Links to wrong path for pdf_beamtime |
| **Dead code reference** | `bluesky_ros/archive/pdf/pdf_beamtime.py` | Imports non-existent `custom_msgs.action.PickPlace` |
| **Old repo URLs** | 2 Dockerfiles | Still reference `nsls2/erobs` instead of `bondada-a/erobs` |

---

### 🟡 MAJOR ISSUES (Should Fix)

**Configuration:**
- Inconsistent naming: `"epick"` vs `"epick_gripper"` across task files
- Massive duplication: 17-step sequences copy-pasted 6x in some task files
- Empty task arrays: 4 files with `"tasks": []`
- File with space in name: `beamline_test copy.json`
- Hardcoded IPs and dock numbers throughout

**Robot Descriptions:**
- Different root frames: UR3e uses `world`, UR5e uses `map`
- Duplicate SRDF files in UR3e config
- Hardcoded IP `192.168.1.101` in generated URDFs
- UR3e finger joints are `fixed` instead of `prismatic`

**Build/Deploy:**
- No multi-stage Docker builds (large images)
- Missing `set -euo pipefail` in most shell scripts
- Hardcoded user paths (`/home/aditya/...`) in scripts
- CMake version inconsistency (3.8 vs 3.22)
- Ruff CI uses `--fix` which modifies code

**Documentation:**
- Missing READMEs for beambot and beambot_interfaces (the main packages!)
- Outdated container registry references
- Legacy UR3e/mtc_pipeline references (renamed to UR5e/beambot)
- Action server count inconsistency (7 vs 8 in docs)

**Interfaces:**
- Duplicate action definitions in `hello_orchestrator/action/`
- Unused package `hello_orchestrator_interfaces`
- No documentation on legacy pdf_beamtime interfaces

---

### 🟢 MINOR ISSUES (Nice to Fix)

- 24 formatting/documentation gaps in configs
- Missing metadata in some JSON files
- Isaac Sim USD files have no README explaining their purpose
- Some deprecated scripts in `pdf-launch-scripts/`
- No dependency version constraints in package.xml files

---

### ✅ WHAT'S GOOD

**Code Quality:**
- Well-structured XACRO macros with good parameterization
- Clean camera abstraction module
- Comprehensive beambot_interfaces (9 well-documented actions)
- Smart batching optimization in orchestrator
- Proper pause/resume and error recovery

**Robot Descriptions:**
- All 16 local mesh files present and valid
- Clean separation between visual and collision meshes
- Proper MoveIt collision matrices

**Documentation:**
- Excellent CLAUDE.md with architecture overview
- .planning/ has good architecture analysis
- LaTeX docs have pre-generated PDFs

---

### 📊 REVIEW STATISTICS

| Category | Files Reviewed | Issues Found |
|----------|---------------|--------------|
| JSON/YAML configs | 134 | 47 |
| Robot descriptions | 36 | 8 |
| Documentation | 40+ | 15 |
| Build/Deploy | 50+ | 20+ |
| ROS Interfaces | 54 | 7 |
| **TOTAL** | **534** | **~97** |

---

### 🎯 RECOMMENDED PRIORITY ORDER

**Immediate (potential bugs):**
1. Fix HandE mass value (100,000x error)
2. Fix UR3e kinematics timeout
3. Fix obstacle z-value typos
4. Enable tests in CI
5. Fix SRDF virtual joint

**Soon (cleanup):**
6. Update old repo URLs in Dockerfiles
7. Consolidate duplicate task sequences
8. Add READMEs for beambot packages
9. Fix broken README link
10. Standardize naming (epick vs epick_gripper)

**Later (improvements):**
11. Multi-stage Docker builds
12. Add shell script error handling
13. Remove hardcoded paths/IPs
14. Consolidate demo interface packages
15. Add dependency version constraints

---

*Full review completed using 5 parallel agents covering all 534 files.*
