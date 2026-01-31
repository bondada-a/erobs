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
