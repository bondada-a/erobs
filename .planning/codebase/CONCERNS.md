# Codebase Concerns

**Analysis Date:** 2026-01-27

## Tech Debt

**Large Monolithic Files:**
- Files: `src/beambot/beambot/action_servers/orchestrator.py` (1108 lines), `src/beambot/beambot/stages/vision_stages.py` (1333 lines), `src/beambot/beambot/camera/zivid.py` (783 lines)
- Issue: Multiple responsibilities in single files
- Why: Rapid development, incremental feature additions
- Impact: Difficult to test, high cognitive load, increased bug likelihood
- Fix approach: Extract concerns into focused modules (batching, lifecycle, dispatch)

**Missing Input Validation:**
- File: `src/beambot/beambot/action_servers/orchestrator.py:105-106`
- Issue: YAML config loaded without schema validation, assumes keys exist
- Why: Quick prototyping without validation layer
- Impact: Confusing `KeyError` exceptions instead of meaningful errors
- Fix approach: Add Pydantic models or JSON Schema validation

**Hardcoded Magic Numbers:**
- File: `src/beambot/beambot/camera/zivid.py:275, 318`
- Issue: Timing constants like `for i in range(20)` and `max_wait_iterations = 200` without configuration
- Why: Tuned empirically, not parameterized
- Impact: Detection may timeout on slower hardware
- Fix approach: Move to configuration file or class constants with documentation

## Known Bugs

**Race Condition in Pause/Resume:**
- File: `src/beambot/beambot/action_servers/orchestrator.py:286-326`
- Symptoms: Robot could get stuck in PAUSED state
- Trigger: Pause/resume during state transitions
- Workaround: Restart orchestrator node
- Root cause: Inadequate synchronization on `_pause_event` and state flags
- Fix: Add proper mutex around all pause state accesses

**Detection Label Instability:**
- File: `src/beambot/beambot/camera/zivid.py` (contour detection)
- Symptoms: Sample labels shift between captures
- Trigger: Detection returning different number of objects
- Workaround: Use ArUco markers for critical operations
- Root cause: Contour detection order depends on OpenCV internals
- Blocked by: Needs stable spatial indexing implementation

## Security Considerations

**Raw Socket for Tool Voltage:**
- File: `src/beambot/beambot/core/moveit_lifecycle_manager.py:363-385`
- Risk: No verification that voltage command was received/executed
- Current mitigation: Robot hardware handles invalid commands safely
- Recommendations: Add acknowledgment verification, timeout handling

**Configuration File Permissions:**
- File: `src/beambot/config/default_beamline.yaml`
- Risk: Configuration loaded without permission checks
- Current mitigation: Running in controlled Docker environment
- Recommendations: Validate file ownership in production

## Performance Bottlenecks

**Point Cloud Transmission:**
- File: `src/beambot/beambot/camera/zivid.py:248, 315-316`
- Problem: Full point cloud (~40MB) transmitted for detection
- Measurement: 3-4 seconds transmission time
- Cause: No downsampling or ROI filtering
- Improvement path: Add downsampling node, implement ROI masking

**Busy-Wait Loops:**
- File: `src/beambot/beambot/core/moveit_lifecycle_manager.py:155-217`
- Problem: Polling with `time.sleep()` blocks ROS2 executor
- Measurement: Up to 45 iterations × 1s delay
- Cause: Waiting for MoveIt services without async pattern
- Improvement path: Use ROS2 async service clients with callbacks

**Client Recreation Overhead:**
- File: `src/beambot/beambot/core/moveit_lifecycle_manager.py:175-182`
- Problem: New service client created on each poll attempt
- Measurement: 45 potential client creations during startup
- Cause: Polling pattern creates client, waits, destroys
- Improvement path: Create client once, reuse across polls

## Fragile Areas

**MoveIt Lifecycle Management:**
- File: `src/beambot/beambot/core/moveit_lifecycle_manager.py`
- Why fragile: Subprocess spawning, socket commands, service polling
- Common failures: Timeout waiting for planning service, voltage command fails silently
- Safe modification: Add comprehensive logging before changes
- Test coverage: Manual testing only

**Vision Detection Pipeline:**
- File: `src/beambot/beambot/stages/vision_stages.py`
- Why fragile: Multiple detection methods, coordinate transforms, retry logic
- Common failures: Marker not found, transform stale, point cloud empty
- Safe modification: Test with all detection types after changes
- Test coverage: Manual scripts, no automated tests

**Pause/Resume State Machine:**
- File: `src/beambot/beambot/action_servers/orchestrator.py:265-345`
- Why fragile: Threading with events and locks
- Common failures: Deadlock, stuck in paused state
- Safe modification: Add state transition logging
- Test coverage: None

## Scaling Limits

**Single Orchestrator:**
- Current capacity: One task script at a time
- Limit: Concurrent goal requests queued, not parallelized
- Symptoms at limit: Goals timeout waiting for previous to complete
- Scaling path: Would need multi-robot coordination layer

## Dependencies at Risk

**numpy<2 Pinning:**
- File: `docker/erobs-common-img/Dockerfile:75`
- Risk: Pinned to avoid ROS 2 Humble compatibility issues
- Impact: Cannot use numpy 2.x features, security updates delayed
- Migration plan: Wait for ROS 2 Jazzy/Rolling with numpy 2 support

**MTC Python Bindings:**
- Package: `moveit_task_constructor_core` pybind11 bindings
- Risk: Unofficial bindings, may break with MoveIt updates
- Impact: Core functionality depends on these
- Migration plan: Monitor upstream for official Python support

## Missing Critical Features

**Automated Test Suite:**
- Problem: No unit tests for orchestrator, stages, vision
- Current workaround: Manual testing, test scripts
- Blocks: Safe refactoring, CI/CD pipeline
- Implementation complexity: Medium (need mock MTC, service mocks)

**Watchdog for Stuck Operations:**
- Problem: No timeout on MTC execution phase
- Current workaround: Manual intervention if robot hangs
- Blocks: Unattended 24/7 operation
- Implementation complexity: Medium (need execution monitoring thread)

**Configuration Validation:**
- Problem: No startup validation of beamline config
- Current workaround: Fail at runtime with cryptic errors
- Blocks: Easy deployment troubleshooting
- Implementation complexity: Low (add validation function)

## Test Coverage Gaps

**Orchestrator (1108 lines):**
- What's not tested: Batch grouping, goal parsing, dispatch logic
- Risk: Regressions in core coordination
- Priority: High
- Difficulty: Medium (need to mock action clients)

**Vision Stages (1333 lines):**
- What's not tested: Detection result handling, transform chains
- Risk: Vision failures in production
- Priority: High
- Difficulty: Medium (need mock camera service)

**MoveIt Lifecycle Manager:**
- What's not tested: Service polling, subprocess management
- Risk: Startup failures
- Priority: Medium
- Difficulty: High (subprocess mocking complex)

---

*Concerns audit: 2026-01-27*
*Update as issues are fixed or new ones discovered*
