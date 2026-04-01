# Beambot Code Review Findings

> Generated 2026-03-31 from 4-agent deep analysis + manual verification.
> Ranked from highest to lowest impact.

---

## BUGS

### ~~B1. TF frame "base" vs "base_link" in vision_stages.py~~ — NOT A BUG
- **File:** `stages/vision_stages.py` lines 1280, 1288, 1296
- **Status:** `"base"` is a valid UR TF frame (identity transform to `base_link`). `can_transform` only checks existence, not values. Actual transforms always use `"base_link"` in `_transform_to_base_link()`. Consistency issue only — change to `"base_link"` if desired but not functional.

### B2. Uninitialized `client` in exception handler — moveit_lifecycle_manager.py
- **File:** `core/moveit_lifecycle_manager.py` line 275
- **Bug:** If `create_client()` at line 240 throws, the `except` block at line 274 tries `destroy_client(client)` but `client` was never assigned. Causes `NameError`.
- **Impact:** MEDIUM — masks the real startup error. The outer `except` catches it, but error message is misleading.
- **Fix:** Initialize `client = None` before the try block, check `if client is not None` before destroying.

### B3. Resource leak in `_restart_external_control`
- **File:** `core/moveit_lifecycle_manager.py` lines 461-484
- **Bug:** If `call_async()` at line 473 throws, the `except` at line 482 doesn't destroy the client.
- **Impact:** LOW — client leaks. Only called during tool exchange, so leaks are rare.
- **Fix:** Use try/finally for `destroy_client`.

### B4. Division by zero in marker normalization — MCP server
- **File:** `mcp/beambot_mcp_server.py` lines 1459-1461
- **Bug:** `marker_x / np.linalg.norm(marker_x)` divides by zero if marker corners are degenerate (same point or collinear).
- **Impact:** LOW — only in `detect_sample()`, only with badly detected markers.
- **Fix:** Check `norm > 0` before dividing.

### B5. `settle_time` parameter is dead code in `detect_markers`
- **File:** `camera/zivid.py` line 74, 91
- **Bug:** `settle_time` is accepted as a parameter but never actually used — no `time.sleep(settle_time)` before the pre-capture timestamp or service call. The parameter has zero effect.
- **Impact:** HIGH — This is the most likely cause of intermittent ~3mm z-offset errors during experiments. The orchestrator's 1.0s settle happens before the goal is sent, but 50-200ms of variable network/processing delay occurs before `pre_capture_stamp` is recorded. If robot vibration hasn't fully damped in that variable window, the TF at `pre_capture_stamp` is slightly wrong. The 30% failure rate matches the variability of this delay.
- **Fix:** Add `if settle_time > 0: time.sleep(settle_time)` before line 106 (`pre_capture_stamp = node.get_clock().now()`), so settling happens immediately before the timestamp is captured — not 50-200ms earlier in the orchestrator.
- **Related:** `DEFAULT_SETTLE_TIME = 0.0` in `vision_stages.py` line 70 means this parameter is also zeroed at the class level. Once the sleep is implemented, re-enable with a reasonable default (e.g., 0.3s).

### B6. Unvalidated marker_ids input crashes MCP server
- **File:** `mcp/beambot_mcp_server.py` line 1196
- **Bug:** `[int(x.strip()) for x in marker_ids.split(",")]` — no try/catch. Non-numeric input crashes the tool.
- **Impact:** LOW — MCP input comes from Claude which sends valid IDs, but defensive coding is better.
- **Fix:** Wrap in try/except, return error JSON.

---

## STALE / REDUNDANT CODE

### S1. `grippers.yaml` is stale — superseded by `default_beamline.yaml`
- **File:** `config/grippers.yaml`
- **What:** Only has 4 grippers with 2 fields each (moveit_package, tool_voltage). Missing: dock_number, controller_name, gripper_group, states, cup_profile.
- **Reality:** `default_beamline.yaml` is the actual source of truth used by the orchestrator. `grippers.yaml` is never loaded by any code.
- **Action:** Delete or add deprecation comment pointing to `default_beamline.yaml`.

### S2. `ur3e_beamline.yaml` is incomplete and likely broken
- **File:** `config/ur3e_beamline.yaml`
- **What:** Missing: cameras section, vision_targets, controller_name, dock_number. References `ur3e_hande_moveit_config` which doesn't exist in this workspace.
- **Action:** Either update to match `default_beamline.yaml` structure or delete if UR3e is not currently supported.

### S3. Dead functions in MCP server
- **File:** `mcp/beambot_mcp_server.py` lines 2101-2108
- **What:** `_load_grippers_config()` and `_load_tip_rack_config()` are defined but never called. Leftover from before beamline config centralization.
- **Action:** Delete.

### S4. Unused imports in stage files
- **Files:**
  - `pick_place_stages.py` line 27: `joints_from_degrees` imported but never used
  - `vision_pick_place_stages.py` line 34: `joints_from_degrees` imported but never used
  - `end_effector_stages.py` line 6: `core` imported but never used
- **Action:** Remove unused imports.

### S5. Z-offset hardcoding mismatch in `_detect_current_gripper`
- **File:** `stages/vision_stages.py` line 1284
- **What:** Returns hardcoded `0.003` for epick_tip, but `_Z_OFFSETS` dict at line 1030 defines it as `0.0`. Stale value from before URDF calibration fix (#28).
- **Action:** Change to `0.0` to match `_Z_OFFSETS`, or use `self._z_offset_for_frame()`.

---

## THREAD SAFETY

### T1. VacuumMonitor state accessed from multiple threads without locking
- **File:** `core/vacuum_monitor.py`
- **What:** `_on_status()` runs on subscription thread, `update_after_tasks()` and `check_lost()` run from orchestrator thread. Both read/write `self.armed`, `self.lost`, `self.status` without synchronization.
- **Practical impact:** LOW — Python's GIL makes simple attribute assignment atomic. The worst case is a missed detection on one cycle, caught on the next. But it's technically a race condition.
- **Fix:** Add `threading.Lock()` around state modifications if we want to be correct.

### T2. base_action_server goal acceptance race window
- **File:** `action_servers/base_action_server.py` lines 49-61
- **What:** Lock is released after `_goal_callback` returns ACCEPT but before `_execute_callback` sets `_executing = True`. Two goals could both pass the check.
- **Practical impact:** VERY LOW — ROS2 action servers serialize execute callbacks by default with `ReentrantCallbackGroup` only allowing one at a time in practice. And the orchestrator (which uses its own lock) is the only caller.
- **Note:** The individual action servers use `MutuallyExclusiveCallbackGroup` (default), which prevents this race entirely.

---

## SIMPLIFICATION OPPORTUNITIES

### P1. Duplicate camera config loading in vision servers
- **Files:** `action_servers/vision_server.py` lines 54-80, `action_servers/vision_pick_place_server.py` lines 29-55
- **What:** Identical 25-line config loading block.
- **Note:** Depends on #47 (whether vision_pick_place gets consolidated). If it stays, extract to shared helper.

### P2. HSV detection only in MCP server, not in camera abstraction
- **File:** `mcp/beambot_mcp_server.py` line 170
- **What:** HSV color detection is implemented only in MCP server. Circle, contour, and marker detection exist in both MCP server and `beambot/detection/algorithms.py`. Inconsistent.
- **Action:** Move HSV to `detection/algorithms.py` for consistency.

### P3. MoveToStages/EndEffectorStages created per batch execution
- **File:** `action_servers/orchestrator.py` lines 451-452
- **What:** `_execute_batch()` creates new `MoveToStages(self, self._arm_group)` and `EndEffectorStages(self, self._arm_group)` every batch. MoveToStages constructor creates a TF Buffer + TransformListener each time.
- **Impact:** Creates unnecessary TF subscriptions. New buffer starts empty.
- **Fix:** Cache stage instances on the orchestrator, create once.

### P4. Duplicate scan position parsing in orchestrator
- **File:** `action_servers/orchestrator.py` — `_call_vision_moveto` and `_call_vision_scan`
- **What:** ~25 lines of identical pose-key → radians → flat list conversion.
- **Fix:** Extract `_parse_scan_positions(step, poses_json)` helper.

### P5. Simple action servers are pure boilerplate
- **Files:** `move_to_server.py`, `pick_place_server.py`, `end_effector_server.py`, `tool_exchange_server.py`, `pipettor_server.py` — each ~35 lines
- **What:** Each just calls `super().__init__()` and sets `self._stages = XyzStages(self)`. No added behavior.
- **Note:** Not worth changing — they're small, explicit, and easy to find. A factory pattern would save lines but reduce clarity.

---

## ROBUSTNESS

### R1. Image conversion errors not caught in camera modules
- **Files:** `camera/zed.py` line 220, `camera/zivid.py` line 286
- **What:** No try/catch around `bridge.imgmsg_to_cv2()`. If encoding mismatch occurs, detection pipeline crashes.
- **Fix:** Wrap in try/except, return None/empty result.

### R2. Race condition on Zivid capture state in MCP server
- **File:** `mcp/beambot_mcp_server.py` lines 389-392
- **What:** `last_rgb`, `last_cloud`, `_waiting_for_capture` accessed from MCP thread and ROS executor thread without locking.
- **Practical impact:** LOW — `threading.Event` handles the main synchronization. The boolean flag race is benign in practice.

### R3. `_execute_scan` bypasses base class error handling
- **File:** `action_servers/vision_server.py` lines 111-171
- **What:** If `scan_all_tags()` throws an uncaught exception, the action hangs (no result returned). Other servers go through base class try/except.
- **Fix:** Add try/except wrapper or route through base class.

### R4. No error handling for missing Zivid config files in launch
- **File:** `launch/beambot_bringup.launch.py` lines 171-180
- **What:** References `config/zivid_settings.yml` and `config/scene_capture_noproj.yml` which are Zivid-specific and may not be in the repo (generated per-camera).
- **Impact:** Vision node fails to start if files missing.

---

## CONFIG / DOCS

### C1. `controller_name` field added but not reflected in ur3e config or grippers.yaml
- **What:** We just added `controller_name` to `default_beamline.yaml` but `ur3e_beamline.yaml` and `grippers.yaml` don't have it.
- **Action:** Update ur3e config if maintained, or delete stale configs.

### C2. `test_sample_detection.py` not installed in CMakeLists.txt
- **File:** `CMakeLists.txt`
- **What:** Script exists but won't be available via `ros2 run beambot`.
- **Fix:** Add to install list.

### C3. Optional type hint used as string literal in multiple files
- **Files:** `pipettor_stages.py`, `end_effector_stages.py`, `pick_place_stages.py`, `tool_exchange_stages.py`
- **What:** Use `'Optional[str]'` (string) instead of importing `Optional` from typing. Works at runtime but breaks static type checking.
- **Note:** Cosmetic — Python evaluates string annotations lazily. Not a runtime issue.
