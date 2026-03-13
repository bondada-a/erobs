# EROBS Migration Plan: humble-dev -> upstream (NSLS2/erobs)

## Overview

Migrate polished code from `humble-dev` (bondada-a/erobs fork) to upstream `NSLS2/erobs` (branch: `humble`) via focused, testable PRs. No deletions until all additions land — the pre-cleanup upstream/humble branch serves as an archive.

**Source of truth**: `/home/aditya/work/github_ws/dev/erobs` (branch: `humble-dev`)
**PR workspace**: `/home/aditya/work/github_ws/pr_branch/` (fresh branch from upstream/humble per PR)
**Upstream**: https://github.com/NSLS2/erobs (branch: `humble`)
**Fork**: https://github.com/bondada-a/erobs

---

## PR Status

### Previously Merged PRs (before this migration)

| PR # | Title | Status |
|------|-------|--------|
| #70 | Add end_effectors package organization | MERGED |
| #73 | Add UR5e robot descriptions and MoveIt configs | MERGED |
| #74 | Switch to upstream epick driver, fix build issue | MERGED |
| #78 | Add hello_orchestrator_py demo package | MERGED |
| #79 | UR5e MoveIt config updates | MERGED |

### Current Migration PRs

| PR # | Title | Status | Wave |
|------|-------|--------|------|
| #80 | Update Zivid camera mount and MoveIt configs with calibration data | SUBMITTED | 1 |
| - | Add beambot framework (interfaces + orchestrator) | TODO | 2 |
| - | Add MTC GUI and Bluesky integration | TODO | 3 |
| - | Add CMS tasks, docs, and project config | TODO | 4 |
| - | Remove legacy packages (cleanup) | TODO | Later |

---

## PR 1: UR5e Description & Config Updates [SUBMITTED - PR #80]

**Branch**: `pr/zivid-camera-mount-and-config-updates`
**Commit**: `c57d03f`
**Status**: Waiting for review/merge

### What's in it (31 files, +237/-131)

**ur5e_robot_description:**
- `urdf/zivid_camera_mount.xacro` — complete rewrite with direct hand-eye calibration frame chain (tool0 -> zivid_optical_frame)
- `config/hand_eye_calibration.yaml` — NEW: raw 4x4 calibration matrix
- `meshes/zivid/zivid_onarm_mount.stl` — NEW: mount mesh (~0.5MB)
- 4 xacro files — fix `initial_positions_file` path + camera mount `rpy`

**All 4 MoveIt configs (standalone, epick, hande, pipettor):**
- `config/joint_limits.yaml` — acceleration limits (5.0 rad/s^2)
- `config/initial_positions.yaml` — real robot joint positions
- `config/pilz_industrial_motion_planner_planning.yaml` — NEW: Pilz planner support
- `config/ur.srdf` — wrist_3_joint home -> -pi
- `config/ompl_planning.yaml` — goal_bias: 0.15 (all 4 configs)
- `launch/robot_bringup.launch.py` — Pilz pipeline added

---

## PR 2: Beambot Framework [TODO - after PR #80 merges]

**Suggested branch**: `pr/add-beambot-framework`
**Scope**: `beambot_interfaces` + `beambot` + `ros2.repos` update
**Size**: 65 files, +12,168 lines (all new)

### What to test after merge
- `colcon build` succeeds
- `ros2 launch beambot beambot_bringup.launch.py use_fake_hardware:=true`
- Send action goals to beambot action servers
- Run MTC tasks via orchestrator

### Files to copy from dev/erobs

**src/beambot_interfaces/ (11 files):**
- `CMakeLists.txt`, `package.xml`
- 9 action definitions: `EndEffectorAction`, `MTCExecution`, `MoveToAction`, `PickPlaceAction`, `PipettorAction`, `ToolExchangeAction`, `VisionMoveToAction`, `VisionPickPlaceAction`, `VisionScanAction`

**src/beambot/ (53 files):**
- `CMakeLists.txt`, `package.xml`, `resource/beambot`
- `beambot/__init__.py`
- `beambot/action_servers/` — orchestrator.py, base_action_server.py, move_to_server.py, pick_place_server.py, end_effector_server.py, tool_exchange_server.py, pipettor_server.py, vision_server.py, vision_pick_place_server.py
- `beambot/stages/` — base_stages.py, move_to_stages.py, pick_place_stages.py, end_effector_stages.py, tool_exchange_stages.py, pipettor_stages.py, vision_stages.py, vision_pick_place_stages.py
- `beambot/camera/` — __init__.py, zivid.py, zed.py
- `beambot/core/` — __init__.py, moveit_lifecycle_manager.py
- `beambot/detection/` — __init__.py, algorithms.py, params.py
- `beambot/batch_planner.py`, `beambot/octomap_to_planning_scene.py`, `beambot/pointcloud_relay.py`
- `mcp/` — __init__.py, erobs_mcp_server.py, point_selector_gui.py
- `config/` — default_beamline.yaml, beamline_scene.yaml, grippers.yaml, ur3e_beamline.yaml, vision_objects.json, zivid_settings.yml, scene_capture.yml
- `launch/` — beambot_bringup.launch.py, octomap_test.launch.py
- `scripts/` — beambot_client.py, live_stitcher.py, stitch_from_bag.py, test_contour_detection.py, test_pointcloud_stability.py, test_wafer_detection.py
- `docs/aruco_detection_variance_investigation.md`

**src/ros2.repos:**
- Add `moveit_task_constructor` (humble branch) — required build dependency for beambot

### Commands to execute

```bash
cd /home/aditya/work/github_ws/pr_branch
git fetch upstream humble
git checkout -b pr/add-beambot-framework upstream/humble

DEV=/home/aditya/work/github_ws/dev/erobs
PR=/home/aditya/work/github_ws/pr_branch

# Copy beambot_interfaces (entirely new)
cp -r "$DEV/src/beambot_interfaces" "$PR/src/beambot_interfaces"

# Copy beambot (entirely new)
cp -r "$DEV/src/beambot" "$PR/src/beambot"

# Copy ros2.repos
cp "$DEV/src/ros2.repos" "$PR/src/ros2.repos"

# Stage and verify
cd "$PR"
git add src/beambot_interfaces/ src/beambot/ src/ros2.repos
git diff --cached --stat

# Verify nothing outside scope
git status --short | grep -v "^??" | grep -v "src/beambot" | grep -v "src/ros2.repos"
# ^ should be empty

# Commit
git commit -m "Add beambot framework: unified Python orchestrator for robotic beamline operations

- Add beambot_interfaces: 9 ROS2 action definitions for robot task execution
- Add beambot: orchestrator with action servers, MTC stages, camera drivers,
  detection algorithms, MCP server, and beamline configs
- Update ros2.repos: add moveit_task_constructor dependency

beambot replaces the previous C++ mtc_pipeline approach with a unified
Python framework supporting MoveTo, PickPlace, ToolExchange, Vision,
Pipettor, and EndEffector actions."

# Push and create PR
git push -u origin pr/add-beambot-framework
gh pr create --repo NSLS2/erobs --base humble \
  --head bondada-a:pr/add-beambot-framework \
  --title "Add beambot framework: unified Python orchestrator" \
  --body "$(cat <<'PREOF'
## Summary

Adds the beambot framework — a unified Python orchestrator for robotic beamline operations. This is the core package that drives the UR5e robot through MoveIt Task Constructor (MTC) for multi-step experimental workflows.

### New packages

**beambot_interfaces** (9 action definitions):
- MTCExecution, MoveToAction, PickPlaceAction, EndEffectorAction
- ToolExchangeAction, PipettorAction
- VisionScanAction, VisionMoveToAction, VisionPickPlaceAction

**beambot** (orchestrator + action servers):
- Task orchestrator: sequences multi-step tasks from JSON configs
- 7 action servers: MoveTo, PickPlace, EndEffector, ToolExchange, Pipettor, VisionScan, VisionPickPlace
- MTC stage library: composable stage definitions for each action type
- Camera abstraction: Zivid 3D + ZED drivers with ArUco/contour detection
- MCP server: experimental AI-assisted robot operation
- Beamline configs, launch files, and test scripts

**ros2.repos update**: adds moveit_task_constructor (humble branch)

## Test plan
- [ ] \`colcon build\` succeeds
- [ ] Launch: \`ros2 launch beambot beambot_bringup.launch.py use_fake_hardware:=true\`
- [ ] Action servers are visible: \`ros2 action list\`
- [ ] Send a MoveTo goal via CLI or beambot_client.py
PREOF
)"
```

### Notes
- beambot depends on `beambot_interfaces` — both must be in the same PR
- `ros2.repos` adds `moveit_task_constructor` which beambot imports for MTC stage execution
- CI may need `--skip-keys` for `zivid_interfaces` in rosdep (upstream already has this in their workflow)
- The MCP server (`mcp/`) is experimental but included since it's part of the package

---

## PR 3: MTC GUI + Bluesky Integration [TODO - after PR #2 merges]

**Suggested branch**: `pr/add-mtc-gui-and-bluesky`
**Scope**: `mtc_gui` + `bluesky_ros` updates + `end_effectors.repos`
**Size**: 24 files, +4,497/-1 lines

### What to test after merge
- `ros2 launch mtc_gui mtc_gui_client.launch.py` — GUI opens
- Compose tasks in GUI, send to beambot orchestrator
- Test Bluesky Ophyd devices connect to action servers

### Files to copy from dev/erobs

**src/mtc_gui/ (11 files — entirely new):**
- `package.xml`, `setup.py`, `setup.cfg`, `resource/mtc_gui`
- `README.md`
- `launch/mtc_gui_client.launch.py`
- `mtc_gui/__init__.py`, `mtc_gui/mtc_gui_client.py`, `mtc_gui/pose_editor.py`, `mtc_gui/poses_manager.py`, `mtc_gui/save_current_pose_dialog.py`

**src/bluesky_ros/ (12 files — restructure + new):**
- Move existing files to `archive/pdf/`: `ophyd_ros.py`, `pdf_beamtime.py`, `pdf_beamtime_demo.py`, `re_demo.py`
- NEW: `archive/README.md`, `archive/local_bsui.sh`
- NEW: `README.md`, `COMMAND_REFERENCE.md`
- NEW: `mtc_ophyd_device.py`, `mtc_ophyd_device_async.py`, `simple_mtc_bluesky.py`, `task_builder.py`

**src/end_effectors/end_effectors.repos:**
- Add pipettor repository entry

### Commands to execute

```bash
cd /home/aditya/work/github_ws/pr_branch
git fetch upstream humble
git checkout -b pr/add-mtc-gui-and-bluesky upstream/humble

DEV=/home/aditya/work/github_ws/dev/erobs
PR=/home/aditya/work/github_ws/pr_branch

# Copy mtc_gui (entirely new)
cp -r "$DEV/src/mtc_gui" "$PR/src/mtc_gui"

# Copy bluesky_ros (overwrite - handles moves + new files)
rm -rf "$PR/src/bluesky_ros"
cp -r "$DEV/src/bluesky_ros" "$PR/src/bluesky_ros"

# Copy end_effectors.repos
cp "$DEV/src/end_effectors/end_effectors.repos" "$PR/src/end_effectors/end_effectors.repos"

# Stage
cd "$PR"
git add src/mtc_gui/ src/bluesky_ros/ src/end_effectors/end_effectors.repos
git diff --cached --stat

# Commit
git commit -m "Add MTC GUI, update Bluesky integration, add pipettor to end_effectors

- Add mtc_gui: Tkinter GUI for composing and executing MTC tasks
- Restructure bluesky_ros: archive PDF-specific files, add MTC Ophyd devices
  and task builder for beamline-agnostic Bluesky integration
- Update end_effectors.repos: add pipettor repository"

# Push and create PR
git push -u origin pr/add-mtc-gui-and-bluesky
gh pr create --repo NSLS2/erobs --base humble \
  --head bondada-a:pr/add-mtc-gui-and-bluesky \
  --title "Add MTC GUI and Bluesky integration updates" \
  --body "## Summary
...
## Test plan
- [ ] \`colcon build\` succeeds
- [ ] \`ros2 launch mtc_gui mtc_gui_client.launch.py\` opens GUI
- [ ] GUI connects to beambot action servers
- [ ] Bluesky Ophyd devices visible"
```

### Notes
- `mtc_gui` depends on `beambot_interfaces` (from PR 2)
- `bluesky_ros` is NOT a ROS2 package (no package.xml) — it's a collection of Python scripts
- The archive/ move preserves old PDF-specific Bluesky files for reference

---

## PR 4: CMS Tasks, Docs, and Project Config [TODO - after PR #2 merges]

**Suggested branch**: `pr/add-cms-tasks-and-docs`
**Scope**: CMS task JSONs + docs + README + .gitignore + notes.md
**Size**: ~36 files, +3,286/-170 lines

### What to test after merge
- Task JSONs load correctly via beambot orchestrator
- README accurately describes project
- Docs render properly

### Files to copy from dev/erobs

**src/cms/tasks/ (15 JSON files — new, replacing .gitkeep):**
- beamline_test.json, beamline_test copy.json, bsui_test.json
- complete_sequence.json, docktest.json, moveto_test.json
- new_test_updated.json, pick_place_hande_test.json
- te_test.json, tool_exchange_test.json, tool_voltage_test.json
- triple_test.json, triple_test_no_tasks.json
- vacuum_place.json, vision_test.json

**docs/ (new + replacing old):**
- NEW: `.gitignore`, `architecture_diagram.tex`, `architecture_diagram.pdf`
- NEW: `beamline_config_diagram.tex`, `beamline_config_diagram.pdf`
- NEW: `mcp_ros_reference.md`
- NOTE: Old docs (ArUco_detection.md, Robustness_tests.md, State_flow.md) and images/ are NOT deleted in this PR — deletions happen later

**Root files:**
- `README.md` — rewrite for current architecture
- `.gitignore` — add entries for rosbags, CLAUDE.md, submodules, debug outputs
- `notes.md` — NEW: development notes (UR driver keep_alive_count issue)

### Commands to execute

```bash
cd /home/aditya/work/github_ws/pr_branch
git fetch upstream humble
git checkout -b pr/add-cms-tasks-and-docs upstream/humble

DEV=/home/aditya/work/github_ws/dev/erobs
PR=/home/aditya/work/github_ws/pr_branch

# CMS tasks
mkdir -p "$PR/src/cms/tasks"
cp "$DEV"/src/cms/tasks/*.json "$PR/src/cms/tasks/"

# Docs (only NEW files — do NOT delete old docs in this PR)
cp "$DEV/docs/.gitignore" "$PR/docs/.gitignore"
cp "$DEV/docs/architecture_diagram.tex" "$PR/docs/architecture_diagram.tex"
cp "$DEV/docs/architecture_diagram.pdf" "$PR/docs/architecture_diagram.pdf"
cp "$DEV/docs/beamline_config_diagram.tex" "$PR/docs/beamline_config_diagram.tex"
cp "$DEV/docs/beamline_config_diagram.pdf" "$PR/docs/beamline_config_diagram.pdf"
cp "$DEV/docs/mcp_ros_reference.md" "$PR/docs/mcp_ros_reference.md"

# Root files
cp "$DEV/README.md" "$PR/README.md"
cp "$DEV/.gitignore" "$PR/.gitignore"
cp "$DEV/notes.md" "$PR/notes.md"

# Stage
cd "$PR"
git add src/cms/tasks/ docs/ README.md .gitignore notes.md
git diff --cached --stat

# Commit
git commit -m "Add CMS task configs, architecture docs, and update README

- Add 15 CMS beamline task JSON configurations
- Add architecture diagrams (TikZ source + PDF)
- Add MCP-ROS action reference documentation
- Update README for current beambot architecture
- Update .gitignore for rosbags, submodules, debug outputs
- Add development notes (notes.md)"

# Push and create PR
git push -u origin pr/add-cms-tasks-and-docs
gh pr create --repo NSLS2/erobs --base humble \
  --head bondada-a:pr/add-cms-tasks-and-docs \
  --title "Add CMS task configs, architecture docs, and update README" \
  --body "## Summary
...
## Test plan
- [ ] Task JSONs parse correctly
- [ ] README links are valid
- [ ] Architecture diagrams render"
```

### Notes
- This PR only ADDS new docs — old docs are NOT deleted (that's the cleanup PR)
- The README references beambot, which should exist if PR 2 has merged
- `setup.sh` is intentionally NOT updated — see warnings section below
- The `beamline_test copy.json` filename has a space — consider renaming

---

## PR 5 (Later): Remove Legacy Packages [TODO - after all additions merge]

**Suggested branch**: `pr/cleanup-legacy-packages`
**Scope**: Delete packages replaced by beambot
**Size**: ~101 files deleted, ~7,169 lines removed

### What to delete

| Package | Files | Replaced by |
|---------|-------|-------------|
| `src/aruco_pose/` | 11 | beambot/camera/zivid.py |
| `src/pdf/pdf_beamtime/` | 18 | beambot action servers |
| `src/pdf/pdf_beamtime_interfaces/` | 6 | beambot_interfaces |
| `src/custom-ur-descriptions/ur3e_hande_moveit_config/` | 6 | Project uses UR5e only |
| `src/custom-ur-descriptions/ur3e_hande_robot_description/` | 25 | Project uses UR5e only |
| `src/demos/hello_moveit/` + `hello_moveit_interfaces/` | 13 | Obsolete demos |
| `src/demos/hello_orchestrator_py/` + interfaces | ? | Consider keeping or removing |
| `docs/ArUco_detection.md`, `Robustness_tests.md`, `State_flow.md` | 3 | Replaced by new docs |
| `docs/images/*` | 9 | Orphaned (only referenced by deleted docs) |

### Notes
- Add `src/pdf/.gitkeep` after deleting pdf_beamtime
- The pre-cleanup state of upstream/humble serves as the archive branch
- Consider creating the archive branch tag before this PR

---

## Important Warnings

### DO NOT include in any PR

1. **`.github/` changes** — humble-dev has REGRESSIONS here (removed `source /opt/ros/${ROS_DISTRO}/setup.bash` from lint script). Upstream's version is correct (fixed in commits `0fbc9ee`, `9faf015`, `230e6fa`, `f7078a0`).

2. **`setup.sh` changes** — humble-dev removed `--skip-keys "zivid_description"` from rosdep and removed a duplicate `rosdep update`. The `--skip-keys` may be needed for CI. Review carefully before including.

3. **Submodule entries** — Do NOT include `src/end_effectors/pipettor`, `src/vision/zivid-ros`, `src/vision/zivid-python-samples` as committed submodule references. They are cloned at build time via `vcs import`.

4. **Leftover dirs in pr_branch** — `src/apriltag_ros/` and `src/moveit_task_constructor/` exist as untracked dirs from previous vcs imports. Ignore them (they're not staged).

### PR branch workflow reminder

Always start from a fresh upstream/humble checkout:
```bash
cd /home/aditya/work/github_ws/pr_branch
git fetch upstream humble
git checkout -b pr/<branch-name> upstream/humble
# Copy files from dev/erobs, NOT from experimental
# Stage ONLY files in scope
# Verify nothing outside scope changed
```

---

## Dependency Graph

```
PR #80 (descriptions)  -->  independent, SUBMITTED
PR 2   (beambot)       -->  independent of #80
PR 3   (GUI + bluesky) -->  depends on PR 2 (needs beambot_interfaces)
PR 4   (tasks + docs)  -->  depends on PR 2 (README references beambot)
PR 5   (cleanup)       -->  depends on ALL above being merged
```

Sequential order: #80 -> PR 2 -> PR 3 & PR 4 (parallel) -> PR 5

---

## Last Updated

2026-03-09 — PR #80 submitted, waiting for merge before proceeding to PR 2.
