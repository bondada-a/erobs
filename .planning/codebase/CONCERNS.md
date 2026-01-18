# Codebase Concerns

**Analysis Date:** 2026-01-17

## Tech Debt

**Bare and Overly Broad Exception Handlers:**
- Issue: Multiple locations use bare `except:` or broad `except Exception:`
- Files:
  - `src/beambot/beambot/core/moveit_lifecycle_manager.py:211-212` - `except: pass` (bare)
  - `src/beambot/beambot/stages/base_stages.py:72` - Silently ignores parameter errors
  - `src/beambot/beambot/action_servers/orchestrator.py:556, 560` - Broad exception catching
  - `src/beambot/beambot/camera/zivid.py:252, 529` - Broad exception catching
- Why: Rapid development, defensive programming
- Impact: Masks unexpected errors, makes debugging difficult
- Fix approach: Catch specific exceptions (ProcessLookupError, OSError, etc.)

**Hardcoded Scaling Factors:**
- Issue: 20% velocity/acceleration is very conservative
- Files: `src/beambot/beambot/stages/base_stages.py:43-45`
  ```python
  VELOCITY_SCALING = 0.2
  ACCELERATION_SCALING = 0.2
  ```
- Why: Safety-first defaults from initial development
- Impact: Slow, jerky movements; operations take longer than necessary
- Fix approach: Move to beamline YAML config, make per-gripper configurable

**Hardcoded Joint Names:**
- Issue: UR5e joint names hardcoded, breaks for other robot models
- Files: `src/beambot/beambot/stages/base_stages.py:34-41`
- Why: Single robot model during development
- Impact: Silent failures or cryptic errors with UR3/UR10
- Fix approach: Load from MoveIt config dynamically

## Known Bugs

**Vision Detection Inconsistency (Documented in CLAUDE.md):**
- Symptoms: Contours don't always get detected, labels shift between captures
- Trigger: Variable lighting conditions, object movement between captures
- Files: `src/beambot/beambot/camera/zivid.py`, `src/beambot/beambot/stages/vision_stages.py`
- Workaround: Retry logic (3 attempts, 0.5s delay)
- Root cause: Lighting/contrast dependent detection, no stable object tracking
- Status: IN PROGRESS - 5-phase remediation plan in CLAUDE.md

**Centroid Accuracy:**
- Symptoms: Robot doesn't move to exact center of detected objects
- Trigger: Vision-guided pick operations
- Files: `src/beambot/beambot/camera/zivid.py` (detection), `src/beambot/beambot/stages/vision_stages.py` (grasp offset)
- Workaround: Manual z_offset tuning
- Root cause: Grasp point calculation not accounting for object geometry

## Security Considerations

**Subprocess Spawning Without Input Validation:**
- Risk: If config values come from untrusted source, code injection possible
- Files: `src/beambot/beambot/core/moveit_lifecycle_manager.py:100-106`
  ```python
  cmd = ["ros2", "launch", config["moveit_package"], ...]
  subprocess.Popen(cmd, start_new_session=True)
  ```
- Current mitigation: Config loaded from local YAML (trusted)
- Recommendations: Document assumption that config is trusted; add validation if loading from network

**Raw Socket Commands to Robot:**
- Risk: Unencrypted commands on UR secondary interface (port 30002)
- Files: `src/beambot/beambot/core/moveit_lifecycle_manager.py:364-371`
- Current mitigation: Private network between robot and control PC
- Recommendations: Document network security requirements; consider authenticated interface

**Unsanitized Logging:**
- Risk: User input (JSON task scripts) logged verbatim
- Files: `src/beambot/beambot/action_servers/orchestrator.py` (multiple logging calls)
- Current mitigation: Low risk (no sensitive data in typical tasks)
- Recommendations: Sanitize before logging if sensitive data possible

## Performance Bottlenecks

**Point Cloud Waiting Loop:**
- Problem: Inefficient spin_once loop waiting for Zivid data
- Files: `src/beambot/beambot/camera/zivid.py:260-272`
  ```python
  for i in range(200):  # Up to 20 seconds
      rclpy.spin_once(node, timeout_sec=0.1)
  ```
- Measurement: Logs every 1s with expensive string formatting
- Cause: Zivid point cloud takes 3-4s longer than RGB image
- Improvement path: Use async Future pattern, reduce logging verbosity

**Polling Instead of Async Waiting:**
- Problem: Manual polling for action completion
- Files: `src/beambot/beambot/action_servers/orchestrator.py:779-801`
  ```python
  while not send_future.done():
      time.sleep(0.01)  # Polling every 10ms
  ```
- Cause: Executor conflicts with ReentrantCallbackGroup (documented in code)
- Improvement path: Refactor callback group architecture

**Excessive Logging Volume:**
- Problem: 55+ logging calls in orchestrator for continuous operation
- Files: `src/beambot/beambot/action_servers/orchestrator.py`
- Cause: Development debugging left in place
- Improvement path: Move verbose logs to debug level

## Fragile Areas

**MoveIt Lifecycle Management:**
- Files: `src/beambot/beambot/core/moveit_lifecycle_manager.py`
- Why fragile: Complex subprocess management, socket communication, controller activation
- Common failures: MoveIt doesn't start, controllers not activated, timeout waiting for services
- Safe modification: Add comprehensive logging, test with multiple gripper switches
- Test coverage: No unit tests, manual testing only

**Task Batching Logic:**
- Files: `src/beambot/beambot/action_servers/orchestrator.py` (`_group_into_batches()`)
- Why fragile: State management across task types, batch-breaking conditions
- Common failures: Wrong gripper state after batch, incomplete batch execution
- Safe modification: Add unit tests for edge cases, document batch-breaking rules
- Test coverage: None

**Vision-Guided Pick Sequence:**
- Files: `src/beambot/beambot/stages/vision_pick_place_stages.py`
- Why fragile: Two MTC tasks, camera timing, TF transform timing
- Common failures: Detection fails between tasks, transform not ready
- Safe modification: Add retry logic, validate TF availability
- Test coverage: Manual scripts only

## Scaling Limits

**Single Orchestrator:**
- Current capacity: One task script at a time
- Limit: Concurrent task execution not supported
- Symptoms at limit: Goal rejection, "server busy" message
- Scaling path: Would need multi-orchestrator architecture (not planned)

**Vision Processing:**
- Current capacity: ~3-4s per detection cycle (Zivid capture + processing)
- Limit: High-throughput sample processing limited by camera speed
- Symptoms at limit: Workflow bottleneck during vision operations
- Scaling path: Pre-capture during arm motion (pipeline optimization)

## Dependencies at Risk

**numpy<2 Constraint:**
- Risk: Pinned for ROS2 Humble compatibility (cv_bridge, tf_transformations)
- Files: `docker/erobs-common-img/Dockerfile`
- Impact: Can't use NumPy 2.x features, potential security patches missed
- Migration plan: Wait for ROS2 packages to update for NumPy 2.x compatibility

**External Gripper Drivers:**
- Risk: External GitHub repos may become unmaintained
- Files: `src/end_effectors/end_effectors.repos`
  - robotiq_hande_driver (AGH-CEAI)
  - ros2_epick_gripper (PickNikRobotics)
- Impact: Bug fixes and ROS2 updates may lag
- Migration plan: Fork and maintain internally if upstream goes stale

## Missing Critical Features

**Octomap Integration Incomplete:**
- Problem: Point cloud obstacle avoidance tested but not integrated into main launch
- Files: `src/beambot/launch/octomap_test.launch.py` (standalone), `src/beambot/launch/beambot_bringup.launch.py` (missing integration)
- Current workaround: Run octomap_test.launch.py separately
- Blocks: Production obstacle avoidance, safety in cluttered environments
- Implementation: Add `use_octomap:=true` argument to bringup launch

**Cartesian Path Fallback:**
- Problem: Cartesian planning can fail, no automatic OMPL fallback
- Files: `src/beambot/beambot/stages/base_stages.py`, `pick_place_stages.py`
- Current workaround: `min_fraction: 0.95` allows partial paths
- Blocks: Reliable execution in constrained spaces
- Implementation: MTC Fallbacks container (documented in CLAUDE.md)

**Input Validation:**
- Problem: Type conversions on user JSON without validation
- Files: `src/beambot/beambot/action_servers/orchestrator.py:514, 833, 888-893`
  ```python
  goal.distance = float(step.get("distance", 0.0))  # No try-catch
  ```
- Blocks: Robust error messages for invalid user input
- Implementation: Wrap in try-except with informative errors

## Test Coverage Gaps

**No Unit Tests for Core Logic:**
- What's not tested: JSON parsing, task batching, gripper state tracking
- Files: `src/beambot/beambot/action_servers/orchestrator.py`
- Risk: Regressions in critical orchestration logic go undetected
- Priority: HIGH
- Difficulty to test: Needs mock infrastructure for ROS2 actions

**No Integration Tests:**
- What's not tested: Tool exchange → motion sequence, pause/resume, batch combinations
- Risk: Multi-step workflows may break without detection
- Priority: HIGH
- Difficulty to test: Needs simulated hardware setup

**Vision Tests Are Manual:**
- What's not tested: Automated detection validation
- Files: `src/beambot/scripts/test_contour_detection.py`
- Risk: Vision regressions require manual verification
- Priority: MEDIUM
- Difficulty to test: Needs captured test images/point clouds

---

*Concerns audit: 2026-01-17*
*Update as issues are fixed or new ones discovered*
