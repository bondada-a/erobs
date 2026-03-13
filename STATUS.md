# STATUS.md — Shared Development Context

> **Read this before starting development work. Update it after making code changes.**
> This file is for repo/code tasks only — not general BNL work.
> Last updated by: Roc (OpenClaw) — 2026-03-10

---

## 🔴 Current Focus

- MCP + camera/vision pipeline work — calibration, detection improvements

---

## ✅ Recently Completed

- ZED eye-to-hand calibration script + verified result
- Vision scan fixes (setGoal type error, grasp → Pilz LIN)
- Refactor: extract make_move_to_named_stage(), Pilz PTP for scan positions
- MCP logging fix (rosout QoS mismatch → file-based log reader)
- Repo cleanup: removed 33 stale task JSONs, old docker dirs, dead scripts
- Docs split: CLAUDE.md → ops-only ref + docs/development.md
- Pose registry with MCP tools (get/save/delete)
- Pilz LIN + PTP planning types added
- **MTC Fallbacks** — automatic planner fallback chains (CartesianPath → Pilz LIN → OMPL)
- **Path constraints** — optional constraints for MTC stages (moveto + pick/place)
- Renamed erobs-mcp-server → beambot-mcp-server
- Camera parameter added to detect_objects and get_point_3d
- ZED calibration consolidated into src/vision/zed_calibration/
- MCP crash logging added

---

## 📋 Up Next

_Prioritized. Pick from top unless you have a reason not to._

### High Priority
1. ~~**Review and update development.md**~~ — DONE. Updated deployment model, start_mcp.sh docs, ZED camera reference.
2. **MCP sample detection model** — current detection (circle/contour) is unreliable, needs redesign for MCP architecture. ArUco markers work but limited.
3. **Iterative visual correction (precision pick)** — coarse-fine positioning for small samples. Phase 1: current detect→move pipeline gets within ~5-10mm. Phase 2: iterative loop (capture → detect → compute offset → small Cartesian correction) until alignment < threshold. Uses existing Zivid + MTC, no new controllers needed. Could be a new task type (`precision_pick`) or enhancement to `vision_pick_place`.

### Medium Priority
2. **ePick suction cup z offset for new cups** — new suction cups require updated z_offset. Subtasks:
   - Architect config support for different suction cups (e.g. per-cup z_offset in `grippers.yaml` or `default_beamline.yaml`). Currently hardcoded at 0.1m in `vision_stages.py`.
   - Measure and set correct z_offset for each suction cup type
   - Test and verify accurate sample pickup with new offsets
3. **Octomap integration** — point cloud obstacle avoidance into beambot_bringup.launch.py
4. ~~**Dynamic speed profiles**~~ — deprioritized (P3). 20% speed cap is intentional safety choice, not a limitation.

### Lower Priority
5. **Hardware capabilities integration** — see `.planning/HARDWARE-CAPABILITIES.md` for full audit. Key subtasks:
   - ~~**ePick vacuum feedback**~~ — DONE. Two layers: (1) MCP `get_vacuum_status()` tool + `get_robot_state()` includes vacuum status; (2) Orchestrator watchdog auto-aborts with `VACUUM_LOST` if object drops between steps. Batching disabled for ePick to ensure per-step boundary checks. Tested on hardware.
   - **Hand-E position feedback** — finger position already in `/joint_states`. Subscribe and check after close command for grasp detection. (P0 in audit)
   - **MCP sensor tools (Hand-E)** — expose `get_gripper_state()`, `check_grasp()` in beambot-mcp-server (depends on Hand-E feedback above)
   - **Force mode controller** — for compliant insertion (pipette tips). Controller configured, needs integration (P1)
   - **Zivid depth ROI** — configured but disabled, toggle in scene_capture.yml
6. **Add camera housing to rviz**
7. **Minimal bsui container** — reduce from ~5GB to ~500MB
8. **New bluesky-ROS communication architecture** — replace JSON-based approach

### Dropped / Deferred
- ~~OMPL goal_bias tuning~~ — dropped. PTP is primary planner; OMPL needs randomness to find alternate paths, increasing goal_bias makes it too similar to PTP.
- ~~F/T sensor integration~~ — deferred. Samples at CMS are too light (gram-scale) for meaningful UR5e F/T readings. Revisit for force-controlled insertions later.

---

## 📝 Decisions & Context

- **MCP server setup**: `beambot-mcp-server` (custom, in beambot/mcp/) + `ros-mcp-server` (external). Both in `.claude/settings.local.json`
- **Gripper tracking**: Orchestrator tracks gripper state. Always use `/beambot_execution`, not individual action servers.
- **Zivid calibration**: Eye-in-hand, last calibrated 2026-01-15. Transform in `ur5e_robot_description/urdf/zivid_camera_mount.xacro`
- **ZED calibration**: Eye-to-hand, completed 2026-03-10
- **Planning strategy**: Pilz LIN/PTP preferred for short/deterministic moves. OMPL/RRTConnect as fallback via MTC Fallbacks container.
- **OMPL goal_bias**: Left at 0.15 intentionally — higher values make it redundant with PTP.

---

## 🔄 Update Log

| Date | Agent | What Changed |
|------|-------|-------------|
| 2026-03-10 | Roc (OpenClaw) | Created STATUS.md. Updated with actual git state — fallbacks/constraints done, OMPL tuning dropped. Added hardware audit subtasks. |
