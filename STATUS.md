# STATUS.md — Shared Development Context

> **Read this before starting development work. Update it after making code changes.**
> This file is for repo/code tasks only — not general BNL work.
> Last updated by: Roc (OpenClaw) — 2026-03-10

---

## 🔴 Current Focus

- MCP + camera/vision pipeline work — calibration, detection improvements
- ZED eye-to-hand calibration — just completed, verified

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

---

## 📋 Up Next

_Prioritized. Pick from top unless you have a reason not to._

### High Priority
1. **MCP sample detection model** — current detection (circle/contour) is unreliable, needs redesign for MCP architecture. ArUco markers work but limited.
2. **Motion planning improvements** — OMPL tuning (goal_bias → 0.3+), MTC Fallbacks container (CartesianPath → Pilz LIN → OMPL cascade)

### Medium Priority
3. **Edit epick suction cup z offset** — new suction cups installed
4. **Add constraints for motion** — moveto + pick/place stages
5. **Hardware capabilities research** — Zivid projector, Hand-E grip feedback, ePick vacuum feedback (doc started, incomplete)
6. **Octomap integration** — point cloud obstacle avoidance into beambot_bringup.launch.py

### Lower Priority
7. **Add camera housing to rviz**
8. **Minimal bsui container** — reduce from ~5GB to ~500MB
9. **New bluesky-ROS communication architecture** — replace JSON-based approach

---

## 📝 Decisions & Context

- **MCP server setup**: `erobs-mcp-server` (custom, in beambot/mcp/) + `ros-mcp-server` (external). Both configured in `.claude/settings.local.json`
- **Gripper tracking**: Orchestrator tracks gripper state. Always use `/beambot_execution`, never individual action servers directly.
- **Zivid calibration**: Eye-in-hand, last calibrated 2026-01-15. Transform in `ur5e_robot_description/urdf/zivid_camera_mount.xacro`
- **ZED calibration**: Eye-to-hand, just completed 2026-03-10
- **Planning**: Pilz LIN/PTP available alongside OMPL/RRTConnect. Pilz preferred for short moves.

---

## 🔄 Update Log

| Date | Agent | What Changed |
|------|-------|-------------|
| 2026-03-10 | Roc (OpenClaw) | Created STATUS.md, compiled tasks from Todoist + development.md + git history |
