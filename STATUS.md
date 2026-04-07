# STATUS.md — Shared Development Context

> **Task tracking has moved to [GitHub Issues](https://github.com/bondada-a/erobs/issues).**
> Check issues for current priorities: `gh issue list --label P0` / `gh issue list --label P1`
> When you complete work, close the relevant issue. When you discover new work, open an issue.
>
> This file retains decisions and context. Last updated: 2026-03-13.

---

## 📋 Task Tracking → GitHub Issues

All development tasks are tracked as GitHub Issues with priority labels:
- **P0** — Critical priority (do next)
- **P1** — High priority
- **P2** — Medium priority
- **P3** — Low priority

Category labels: `vision`, `gripper`, `mcp`, `infra`, `motion`

```bash
# See what to work on
gh issue list --label P0                    # Critical tasks
gh issue list --label P1                    # High priority
gh issue list                               # All open issues

# After completing work
gh issue close <number> --reason completed
```

---

## 📝 Decisions & Context

- **MCP server setup**: `beambot-mcp-server` (custom, in beambot/mcp/) + `ros-mcp-server` (external). Both in `.claude/settings.local.json`
- **Gripper tracking**: Orchestrator tracks gripper state. Always use `/beambot_execution`, not individual action servers.
- **Zivid calibration**: Eye-in-hand, last calibrated 2026-01-15. Transform in `ur5e_robot_description/urdf/zivid_camera_mount.xacro`
- **ZED calibration**: Eye-to-hand, completed 2026-03-10
- **Planning strategy**: Pilz LIN/PTP preferred for short/deterministic moves. OMPL/RRTConnect as fallback via MTC Fallbacks container.
- **OMPL goal_bias**: Left at 0.15 intentionally — higher values make it redundant with PTP.
- **Hand-E grasp detection**: Use `/joint_states` finger position (not C++ GPIO controller) — works in sim + real hardware. (Mar 13, ai-dev cafde57)

### Dropped / Deferred
- ~~OMPL goal_bias tuning~~ — dropped. PTP is primary planner; OMPL needs randomness to find alternate paths, increasing goal_bias makes it too similar to PTP.
- ~~F/T sensor integration~~ — deferred. Samples at CMS are too light (gram-scale) for meaningful UR5e F/T readings. Revisit for force-controlled insertions later.
- ~~Dynamic speed profiles~~ — deprioritized. 20% speed cap is intentional safety choice, not a limitation.

---

