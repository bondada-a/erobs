# P0: Hand-E Grasp Detection — Implementation Plan

**Status**: Draft
**Priority**: P0
**Date**: 2026-03-13
**Branch**: `ai-dev`

## Problem Statement

The Hand-E gripper has no grasp detection in the EROBS system. Unlike the ePick vacuum gripper (which has real-time vacuum seal monitoring via `/object_detection_status`), the Hand-E closes blindly — if it misses the object or the object slips during transport, the system doesn't know.

The Robotiq Hand-E hardware **already computes `object_detected`** from Modbus registers (via `gObj` status bits), but this data is **never exposed to ROS 2**. The hardware interface only exports `position` and `velocity` state interfaces.

## Architecture Overview

Mirror the ePick vacuum monitoring pattern:

```
Hardware (Modbus) → GPIO State Interface → Status Publisher Controller → ROS Topic
                                                                           ↓
                                              Orchestrator Watchdog ←──────┘
                                              MCP Tools ←──────────────────┘
```

### ePick Pattern (existing, to mirror)
```
ePick HW Interface → gpio:object_detection_status → EpickStatusPublisherController
    → /object_detection_status (epick_msgs/ObjectDetectionStatus)
    → orchestrator._on_epick_status() callback
    → _vacuum_armed / _vacuum_lost flags
    → abort on VACUUM_LOST
```

### Hand-E Pattern (to implement)
```
Hand-E HW Interface → gpio:object_detection_status → HandeStatusPublisherController
    → /hande_object_detection_status (std_msgs/Int8)
    → orchestrator._on_hande_status() callback
    → _hande_grasp_armed / _hande_grasp_lost flags
    → abort on GRASP_LOST
```

---

## Phase 1: Config Fix (`allow_stalling: true`)

### Context

The `GripperActionController` (`position_controllers/GripperActionController`) has stall detection built in. With `allow_stalling: false`, the controller **aborts** when the gripper stalls (i.e., closes on an object and can't reach the goal position). With `allow_stalling: true`, it **succeeds** with `stalled=true` in the result — which is the correct behavior for grasping.

Two configs exist with different settings:

| Config | Location | `allow_stalling` | Used When |
|--------|----------|-------------------|-----------|
| Bringup | `src/end_effectors/robotiq_hande_driver/.../hande_controller.yaml` | `false` | Standalone Hand-E launch |
| MoveIt | `src/custom-ur-descriptions/.../ur_hande_controllers.yaml` | `true` | MoveIt launch (beambot flow) |

The MoveIt config is correct. The bringup config needs fixing for consistency and standalone usage.

### Changes

#### 1a. Fix bringup config

**File**: `src/end_effectors/robotiq_hande_driver/robotiq_hande_driver/bringup/config/hande_controller.yaml`
**Line 20**: Change `allow_stalling: false` → `allow_stalling: true`

```yaml
# Before (line 20)
    allow_stalling: false

# After
    allow_stalling: true
```

**Why**: Without this, a successful grasp (object between fingers) causes the GripperActionController to abort the action because it can't reach the fully-closed position. With `allow_stalling: true`, the controller succeeds and reports `stalled: true` — indicating an object was detected.

#### 1b. Velocity reporting bug (noted, not blocking)

**File**: `src/end_effectors/robotiq_hande_driver/robotiq_hande_driver/hardware/src/hande_hardware_interface.cpp`
**Line 209-235** (`gripper_communication()`): Only `read_position_` is updated from the driver. `read_velocity_` is **never updated** — it stays at the initial value of `0.0` forever.

```cpp
// Line 215 — position IS updated
read_position_ = gripper_driver_.get_position();

// read_velocity_ is NEVER updated — always 0.0
```

**Impact**: The `velocity` state interface always reports 0.0. The `GripperActionController` sees velocity=0 < stall_threshold=0.001 from the moment a command is sent, so the stall timer starts immediately. This means:
- **Empty close** (no object): Position reaches within `goal_tolerance` (~0.2s for 25mm at 150mm/s) → `reached_goal=true` before stall timeout (1.0s). **Works correctly.**
- **Object grasp**: Position stalls outside tolerance → after `stall_timeout` (1.0s) → `stalled=true`. **Works correctly, but for the wrong reason** (velocity bug, not actual velocity monitoring).

**Not blocking Phase 1** because the end behavior is correct. Fix later:
```cpp
// In gripper_communication(), after line 215, add:
read_velocity_ = gripper_driver_.get_position();  // Store for velocity calc
// Then compute velocity from position delta / time delta
```

### Verification

1. Build: `colcon build --packages-select robotiq_hande_driver`
2. Launch Hand-E standalone: verify controller starts without errors
3. Close gripper on object: should succeed (not abort)
4. Close gripper on empty air: should succeed with `reached_goal=true`
5. Check via `ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0}}"` — observe `stalled` field in result

### Gotchas

- The bringup config is for standalone Hand-E testing only. The beambot flow uses the MoveIt config (`ur_hande_controllers.yaml` line 71: `allow_stalling: true`), which is already correct.
- Changing `allow_stalling` does NOT add grasp detection — it just prevents false failures. Grasp detection requires Phase 2.

---

## Phase 2: MCP Tools (`get_gripper_state`, `check_grasp`, update `get_robot_state`)

### Dependencies
- Phase 1 (config fix) should be done but is not strictly required for MCP tool development
- Phase 2a (C++ hardware changes) must be done before 2b (MCP tools can subscribe to topic)

### Phase 2a: Expose `object_detected` from Hardware Interface

Mirror the ePick pattern: GPIO state interface → status publisher controller → ROS topic.

#### 2a-i. Add GPIO state interface to Hand-E hardware interface

**File**: `src/end_effectors/robotiq_hande_driver/robotiq_hande_driver/hardware/include/robotiq_hande_driver/hande_hardware_interface.hpp`

Add new member variables (after line 66):
```cpp
// Grasp detection state (exposed via GPIO state interface)
double state_object_detection_;  // 0.0=MOTION_NO_OBJECT, 1.0=STOPPED_OPENING, 2.0=STOPPED_CLOSING, 3.0=REQ_POS_NO_OBJECT
```

**File**: `src/end_effectors/robotiq_hande_driver/robotiq_hande_driver/hardware/src/hande_hardware_interface.cpp`

**In `on_init()` (after line 66)**: Initialize the new state variable:
```cpp
state_object_detection_ = 0.0;
```

**In `export_state_interfaces()` (after line 161)**: Add GPIO state interface:
```cpp
// GPIO state interface for object detection status
state_interfaces.emplace_back(hardware_interface::StateInterface(
    "hande_status", "object_detection_status", &state_object_detection_));
```

**In `gripper_communication()` (after line 215)**: Update object detection from driver:
```cpp
// Update object detection status from Modbus registers
auto status = gripper_driver_.get_status();
{
    std::lock_guard<std::mutex> lock(mtx_read_);
    read_position_ = gripper_driver_.get_position();
    read_object_detection_ = status.object_detected ? 2.0 : 3.0;
    // 2.0 = STOPPED_CLOSING_DETECTED (object grasped)
    // 3.0 = NO_OBJECT_DETECTED
    // For finer granularity, map from ObjectDetectionStatus enum directly
}
```

Actually, for a more precise mapping, read the raw ObjectDetectionStatus:

**File**: `src/end_effectors/robotiq_hande_driver/robotiq_hande_driver/hardware/include/robotiq_hande_driver/hande_gripper.hpp`

Add a method to get raw object detection status (after line 104):
```cpp
/**
 * @brief Retrieves the object detection status enum value.
 * @return 0=MOTION_NO_OBJECT, 1=STOPPED_OPENING_DETECTED, 2=STOPPED_CLOSING_DETECTED, 3=REQ_POS_NO_OBJECT
 */
uint8_t get_object_detection_status() const;
```

**File**: `src/end_effectors/robotiq_hande_driver/robotiq_hande_driver/hardware/src/hande_gripper.cpp`

Add implementation (after line 69):
```cpp
uint8_t HandeGripper::get_object_detection_status() const {
    return static_cast<uint8_t>(prot_.get_object_detection_status());
}
```

**File**: `src/end_effectors/robotiq_hande_driver/robotiq_hande_driver/hardware/include/robotiq_hande_driver/protocol_logic.hpp`

Add public accessor for object detection status (after line 178):
```cpp
/**
 * @brief Retrieves the object detection status.
 * @return The current ObjectDetectionStatus enum value.
 */
ObjectDetectionStatus get_object_detection_status() const;
```

**File**: `src/end_effectors/robotiq_hande_driver/robotiq_hande_driver/hardware/src/protocol_logic.cpp`

Add implementation:
```cpp
ObjectDetectionStatus ProtocolLogic::get_object_detection_status() const {
    return object_detection_status_;
}
```

Then in `gripper_communication()`:
```cpp
read_object_detection_ = static_cast<double>(gripper_driver_.get_object_detection_status());
```

And in `read()` (after line 283):
```cpp
state_object_detection_ = read_object_detection_;
```

Add to header (hpp, after `read_velocity_` on line 66):
```cpp
double read_object_detection_;
```

#### 2a-ii. Declare GPIO in XACRO

**File**: `src/end_effectors/robotiq_hande_description/urdf/robotiq_hande_gripper.ros2_control.xacro`

Add GPIO block (after the `</joint>` closing tag, before `</ros2_control>`):

```xml
<gpio name="hande_status">
  <state_interface name="object_detection_status"/>
</gpio>
```

#### 2a-iii. Create Hand-E status publisher controller

Create a new controller that reads the GPIO state interface and publishes to a ROS topic. Two options:

**Option A (Recommended): Minimal Python node that subscribes to `/joint_states` extended data**
Too complex — GPIO data doesn't go through joint_states.

**Option B (Recommended): C++ controller mirroring ePick pattern**

Create a new package or add to `robotiq_hande_driver`:

**New files**:
- `src/end_effectors/robotiq_hande_driver/robotiq_hande_driver/controllers/include/hande_status_publisher_controller.hpp`
- `src/end_effectors/robotiq_hande_driver/robotiq_hande_driver/controllers/src/hande_status_publisher_controller.cpp`

**Alternative (simpler): Publish directly from hardware interface**

The cleanest minimal approach: since the hardware interface already has a background communication thread, add a ROS publisher there. However, hardware interfaces in ros2_control don't have access to node handles for creating publishers.

**Recommended approach**: Follow the ePick pattern exactly. Create `HandeStatusPublisherController` as a controller plugin.

Reference implementation: `src/end_effectors/ros2_epick_gripper/epick_controllers/src/epick_status_publisher_controller.cpp`

Key differences from ePick:
- State interface name: `"hande_status/object_detection_status"` (not `"gripper/object_detection_status"`)
- Topic name: `"/hande_object_detection_status"` (not `/object_detection_status`)
- Message type: Use `std_msgs/msg/Int8` (avoid creating a new message package, or reuse/reference ePick enum values)
- Status mapping: `0=MOTION_NO_OBJECT, 1=STOPPED_OPENING_DETECTED, 2=STOPPED_CLOSING_DETECTED, 3=REQ_POS_NO_OBJECT`

**Alternative simpler message**: Since we only need a boolean for grasp detection, we could publish `std_msgs/msg/Bool` on `/hande_object_detected`. But using Int8 with the full status enum is more informative and mirrors the ePick pattern better.

#### 2a-iv. Register controller in config files

**File**: `src/end_effectors/robotiq_hande_driver/robotiq_hande_driver/bringup/config/hande_controller.yaml`

Add after `gripper_action_controller` type declaration (line 12):
```yaml
      hande_status_publisher_controller:
        type: robotiq_hande_driver/HandeStatusPublisherController
```

**File**: `src/custom-ur-descriptions/ur5e_moveit_configs/ur_zivid_hande_moveit_config/config/ur_hande_controllers.yaml`

Add the controller declaration alongside gripper_action_controller (after line 52):
```yaml
      hande_status_publisher_controller:
        type: robotiq_hande_driver/HandeStatusPublisherController
```

### Phase 2b: MCP Tool Implementation

#### 2b-i. Add Hand-E status subscription to beambot_mcp_server.py

**File**: `src/beambot/mcp/beambot_mcp_server.py`

**Add constant** (after line 101, near `EPICK_STATUS_TOPIC`):
```python
HANDE_STATUS_TOPIC = "/hande_object_detection_status"
```

**Add status name mapping** (after `EPICK_STATUS_NAMES` at line 109):
```python
HANDE_OBJECT_DETECTION_NAMES = {
    0: "MOTION_NO_OBJECT",
    1: "STOPPED_OPENING_DETECTED",
    2: "STOPPED_CLOSING_DETECTED",
    3: "REQ_POS_NO_OBJECT",
}
```

**Add state variable** to `ROS2BridgeNode.__init__` (after `self.epick_status` around line 345):
```python
self.hande_object_detection: Optional[int] = None
```

**Add subscription** to `ROS2BridgeNode.__init__` (after ePick subscription, around line 340):
```python
# Hand-E object detection status (uses std_msgs/Int8)
from std_msgs.msg import Int8
self._hande_status_sub = self.create_subscription(
    Int8, HANDE_STATUS_TOPIC, self._on_hande_status, 10,
    callback_group=self._cb_group,
)
```

**Add callback** (after `_on_epick_status`, around line 405):
```python
def _on_hande_status(self, msg: Int8):
    self.hande_object_detection = int(msg.data)
```

#### 2b-ii. Create `get_gripper_state()` MCP tool

**File**: `src/beambot/mcp/beambot_mcp_server.py`

Add new tool (after `get_vacuum_status` around line 685):

```python
@mcp.tool()
async def get_gripper_state() -> str:
    """Get the current gripper's grasp detection status.

    Returns a JSON object with:
    - gripper: currently attached gripper name
    - available: whether grasp detection data is being published
    - For Hand-E:
      - object_detection_status: one of "MOTION_NO_OBJECT",
        "STOPPED_OPENING_DETECTED", "STOPPED_CLOSING_DETECTED", "REQ_POS_NO_OBJECT"
      - object_detected: boolean — true if gripper stopped on an object while closing
      - position_mm: current finger position in millimeters (0 = closed, 25 = open)
    - For ePick: delegates to get_vacuum_status() behavior

    Use this AFTER a close/grasp operation to verify the object was grasped.
    If object_detected is false after closing the gripper, the pick failed.
    """
    node = bridge.node
    gripper = node.current_gripper or "unknown"

    if gripper == "epick":
        # Delegate to existing vacuum status logic
        return await get_vacuum_status()

    if gripper == "hande":
        if node.hande_object_detection is None:
            return json.dumps({
                "available": False,
                "gripper": gripper,
                "note": "No data received on /hande_object_detection_status yet. "
                        "The Hand-E status controller may not be running.",
            })

        status_int = node.hande_object_detection
        # Get finger position from joint_states
        position_mm = None
        if node.joint_names and node.joint_positions:
            for name, pos in zip(node.joint_names, node.joint_positions):
                if "hande_left_finger" in name:
                    position_mm = round(pos * 1000, 2)  # meters → mm
                    break

        return json.dumps({
            "available": True,
            "gripper": gripper,
            "object_detection_status": HANDE_OBJECT_DETECTION_NAMES.get(
                status_int, f"UNKNOWN({status_int})"
            ),
            "object_detected": status_int in (1, 2),  # STOPPED_OPENING or STOPPED_CLOSING
            "position_mm": position_mm,
        }, indent=2)

    return json.dumps({
        "available": False,
        "gripper": gripper,
        "note": f"Grasp detection not available for gripper '{gripper}'",
    })
```

#### 2b-iii. Update `get_robot_state()` to include Hand-E grasp info

**File**: `src/beambot/mcp/beambot_mcp_server.py`
**Location**: `get_robot_state()` function, around line 621-638

Add Hand-E status alongside existing ePick status (after the `vacuum_status` block):
```python
    # Include Hand-E grasp status when Hand-E is the active gripper
    hande_status = None
    if gripper == "hande" and node.hande_object_detection is not None:
        status_int = node.hande_object_detection
        hande_status = {
            "object_detection_status": HANDE_OBJECT_DETECTION_NAMES.get(
                status_int, f"UNKNOWN({status_int})"
            ),
            "object_detected": status_int in (1, 2),
        }

    result = {
        "system_running": system_running,
        "gripper": gripper,
        "execution_state": exec_state,
        "joints_deg": joints_deg,
    }
    if vacuum_status is not None:
        result["vacuum_status"] = vacuum_status
    if hande_status is not None:
        result["hande_grasp_status"] = hande_status
    return json.dumps(result, indent=2)
```

### Verification

1. Build C++ packages: `colcon build --packages-select robotiq_hande_driver`
2. Build beambot: `colcon build --packages-select beambot`
3. Launch system with Hand-E gripper
4. Verify topic exists: `ros2 topic echo /hande_object_detection_status`
5. Close gripper on object → verify status changes to `STOPPED_CLOSING_DETECTED` (2)
6. Open gripper → verify status changes to `MOTION_NO_OBJECT` (0) then `REQ_POS_NO_OBJECT` (3)
7. Call `get_robot_state()` via MCP → verify `hande_grasp_status` field present
8. Call `get_gripper_state()` via MCP → verify `object_detected` field

### Gotchas

- **GPIO name must match**: The XACRO GPIO name (`hande_status`) must match the StateInterface name in `export_state_interfaces()` exactly. Mismatch → controller can't find the interface.
- **Controller must be activated**: The `hande_status_publisher_controller` must be spawned and activated by the launch file. Check MoveIt launch files to ensure it's included.
- **No `epick_msgs` dependency**: Use `std_msgs/Int8` to avoid coupling Hand-E driver to ePick packages.
- **Thread safety**: The `state_object_detection_` double is written in `read()` (controller_manager thread) and read by the status publisher controller. The ros2_control framework handles this synchronization — no extra mutex needed for state interfaces.
- **Subscription QoS**: Use QoS depth 10 (matching ePick pattern), not latched/transient_local — the status is continuously published at the controller update rate (10-20 Hz).

---

## Phase 3: Orchestrator Grasp Watchdog (Mirror ePick Pattern)

### Dependencies
- Phase 2a (topic `/hande_object_detection_status` must be available)
- Phase 2b (MCP tools are independent but should be done first for testing)

### Changes

**File**: `src/beambot/beambot/action_servers/orchestrator.py`

#### 3a. Add Hand-E status subscription and state variables

**Near line 218** (after ePick vacuum state variables):
```python
# Hand-E grasp monitoring (mirrors ePick vacuum pattern)
self._hande_grasp_armed = False      # True: gripper closed, watch for object loss
self._hande_grasp_lost = False       # True: object detection lost during transport
self._hande_status: 'int | None' = None  # Current ObjectDetectionStatus value

# Subscribe to Hand-E object detection topic
from std_msgs.msg import Int8
self._hande_sub = self.create_subscription(
    Int8, '/hande_object_detection_status',
    self._on_hande_status, 10,
    callback_group=self._callback_group,
)
```

#### 3b. Add Hand-E status callback

**After `_on_epick_status` (around line 755)**:
```python
def _on_hande_status(self, msg):
    """Callback for /hande_object_detection_status — fires at controller rate."""
    self._hande_status = int(msg.data)
    # Status 0 = MOTION_NO_OBJECT (fingers moving, no contact)
    # Status 1 = STOPPED_OPENING_DETECTED (object detected while opening — unusual)
    # Status 2 = STOPPED_CLOSING_DETECTED (object grasped)
    # Status 3 = REQ_POS_NO_OBJECT (reached target position, no object)
    if self._hande_grasp_armed and self._hande_status in (0, 3):
        # Object was held but now fingers are moving freely or reached target
        # This means the object slipped or was lost
        self._hande_grasp_lost = True
        self.get_logger().warn(
            "GRASP_LOST: Hand-E object detection changed to "
            f"{self._hande_status} while grasp was active"
        )
```

#### 3c. Generalize `_update_vacuum_state` → `_update_grasp_state`

Rename and extend to handle both ePick and Hand-E:

**Replace `_update_vacuum_state` (lines 757-788)**:
```python
def _update_grasp_state(self, executed_tasks: List[Dict[str, Any]]):
    """Update grasp monitor after tasks execute.

    For ePick: arms/disarms vacuum watchdog on vacuum_on/vacuum_off.
    For Hand-E: arms/disarms grasp watchdog on hande_closed/hande_open.
    """
    gripper = self._current_gripper

    if gripper == "epick" and _EPICK_MSGS_AVAILABLE:
        self._update_epick_vacuum_state(executed_tasks)
    elif gripper == "hande":
        self._update_hande_grasp_state(executed_tasks)


def _update_epick_vacuum_state(self, executed_tasks):
    """Existing ePick vacuum logic — extracted from _update_vacuum_state."""
    # ... (move existing ePick code here unchanged) ...


def _update_hande_grasp_state(self, executed_tasks):
    """Hand-E grasp monitoring — mirror of ePick pattern."""
    grasp_state = self._grippers.get("hande", {}).get("states", {}).get("grasp", "hande_closed")
    release_state = self._grippers.get("hande", {}).get("states", {}).get("release", "hande_open")

    for task in executed_tasks:
        if task.get("task_type") != "end_effector":
            continue
        action = task.get("end_effector_action", "")
        if action == grasp_state:
            # ARM the monitor — gripper just closed
            self._hande_grasp_lost = False
            self._hande_grasp_armed = True
            self.get_logger().info("Hand-E grasp monitor ARMED (close detected)")
            # Immediate check: did we detect an object?
            if self._hande_status is not None and self._hande_status not in (1, 2):
                self._hande_grasp_lost = True
                self.get_logger().warn(
                    "GRASP_LOST: no object detected immediately after Hand-E close"
                )
        elif action == release_state:
            # DISARM — intentional open
            self._hande_grasp_armed = False
            self._hande_grasp_lost = False
            self.get_logger().info("Hand-E grasp monitor DISARMED (open detected)")
```

#### 3d. Generalize `_check_vacuum_lost` → `_check_grasp_lost`

**Replace `_check_vacuum_lost` (lines 790-805)**:
```python
def _check_grasp_lost(self) -> bool:
    """Check if grasp was lost since last armed (ePick or Hand-E).

    Returns True if grasp lost (caller should abort), False if OK.
    Sets self._last_error on failure.
    """
    # ePick vacuum check (existing logic)
    if self._vacuum_armed and self._vacuum_lost:
        self._last_error = (
            "VACUUM_LOST: object dropped — ePick reports NO_OBJECT_DETECTED "
            "while vacuum was active. Send vacuum_off then vacuum_on to retry."
        )
        self.get_logger().error(self._last_error)
        self._vacuum_armed = False
        return True

    # Hand-E grasp check (new)
    if self._hande_grasp_armed and self._hande_grasp_lost:
        self._last_error = (
            "GRASP_LOST: object dropped — Hand-E reports no object detected "
            "while gripper was closed. Re-close gripper to retry pick."
        )
        self.get_logger().error(self._last_error)
        self._hande_grasp_armed = False
        return True

    return False
```

#### 3e. Update call sites in `_execute()`

**Line 548-550** (reset at start of execution):
```python
# Reset grasp state for new goal
self._vacuum_armed = False
self._vacuum_lost = False
self._hande_grasp_armed = False
self._hande_grasp_lost = False
```

**Line 586** (disable batching for Hand-E too):
```python
batching_enabled = self._enable_batching and start_gripper not in ("epick", "hande")
if self._enable_batching and not batching_enabled:
    self.get_logger().info(
        f"Batching disabled for {start_gripper} — grasp watchdog needs per-step boundaries"
    )
```

**Lines 624, 655, 681, 687** — Replace all calls:
- `_check_vacuum_lost()` → `_check_grasp_lost()`
- `_update_vacuum_state(...)` → `_update_grasp_state(...)`

#### 3f. Update CLAUDE.md error taxonomy

Add to the error handling table:

```markdown
| `GRASP_LOST` | Grasp | Object dropped during Hand-E transport. Re-close gripper to retry pick. Do NOT proceed to place. |
```

Add to recovery policy:

```markdown
- **After Hand-E GRASP_LOST**: Object was detected after close but lost during transport. Report to user and ask whether to retry the pick or abort. Unlike ePick, Hand-E does not need an off→on cycle — just re-close the gripper.
```

### Verification

1. Build: `colcon build --packages-select beambot`
2. Launch system with Hand-E gripper
3. **Grasp detection test**:
   - Send pick_and_place task via `/beambot_execution`
   - Verify logs show "Hand-E grasp monitor ARMED" after close stage
   - Verify logs show "Hand-E grasp monitor DISARMED" after open stage
4. **Grasp failure test**:
   - Send close command on empty air
   - Verify `GRASP_LOST` error in result
5. **Transport loss test** (manual):
   - Close gripper on object
   - During transport, physically remove object
   - Verify `GRASP_LOST` abort
6. **ePick regression test**:
   - Run existing ePick pick_and_place tasks
   - Verify vacuum monitoring still works unchanged
7. **Batching test**:
   - Verify Hand-E tasks are not batched (per-step boundaries preserved)

### Gotchas

- **Batching must be disabled for Hand-E**: Same reason as ePick — the grasp watchdog needs per-step boundaries to arm/disarm correctly. Without this, all stages execute in a single MTC plan and the watchdog can't distinguish grasp from release.
- **Hand-E status topic availability**: If the `hande_status_publisher_controller` isn't running (e.g., different gripper loaded), `_hande_status` will be `None`. The arm/disarm logic should handle this gracefully — if status is None after arming, log a warning but don't treat as lost.
- **Immediate check timing**: After closing the gripper, there may be a brief delay before the Hand-E hardware updates `object_detection_status`. The `stall_timeout` (1.0s) provides a natural delay, but the immediate check in `_update_hande_grasp_state` happens after MTC returns. By that time, the status should be updated.
- **STOPPED_CLOSING_DETECTED vs REQ_POS_NO_OBJECT**: After a successful close command:
  - With object: status = 2 (`STOPPED_CLOSING_DETECTED`) — object detected
  - Without object: status = 3 (`REQ_POS_NO_OBJECT`) — reached closed position, no object
  - Status = 0 (`MOTION_NO_OBJECT`) — gripper still moving (unlikely after MTC returns)
- **Don't confuse open with loss**: Status 1 (`STOPPED_OPENING_DETECTED`) means the gripper hit something while opening — this is an edge case but still indicates an object. Only 0 and 3 indicate loss.
- **Refactor scope**: The rename from `_check_vacuum_lost` / `_update_vacuum_state` to `_check_grasp_lost` / `_update_grasp_state` touches multiple call sites. Keep the ePick internal logic (`_update_epick_vacuum_state`) completely unchanged to minimize regression risk.

---

## Phase 4: Testing Strategy

### Unit Tests

#### 4a. Orchestrator grasp state logic

**File**: `src/beambot/tests/test_grasp_watchdog.py` (new)

Test cases:
1. **Hand-E arm on close**: Simulate `end_effector` task with `hande_closed` → verify `_hande_grasp_armed = True`
2. **Hand-E disarm on open**: Simulate `end_effector` task with `hande_open` → verify `_hande_grasp_armed = False`
3. **Hand-E immediate loss detection**: Arm with `_hande_status = 3` → verify `_hande_grasp_lost = True`
4. **Hand-E transport loss**: Arm → set `_hande_status = 2` → then set `_hande_status = 3` → verify `_check_grasp_lost() = True`
5. **ePick unchanged**: Verify ePick arm/disarm/loss logic still passes all existing patterns
6. **Cross-gripper isolation**: Arm Hand-E → ePick status change should NOT trigger Hand-E loss (and vice versa)
7. **None gripper**: With `_current_gripper = "none"`, verify no monitoring is armed

#### 4b. MCP tool tests

**File**: `src/beambot/tests/test_mcp_gripper_tools.py` (new)

Test cases:
1. **get_gripper_state with Hand-E**: Mock `hande_object_detection = 2` → verify `object_detected = True`
2. **get_gripper_state with no data**: Mock `hande_object_detection = None` → verify `available = False`
3. **get_gripper_state with ePick**: Verify delegates to vacuum status
4. **get_robot_state includes hande_grasp_status**: Mock Hand-E as current gripper → verify field present
5. **get_robot_state excludes hande_grasp_status for ePick**: Verify field NOT present when ePick is active

### Integration Tests (require hardware or simulation)

#### 4c. Hardware integration

Test matrix (run with real Hand-E on UR5e):

| Test | Command | Expected Status | Expected `object_detected` |
|------|---------|----------------|---------------------------|
| Close on thin object (5mm) | `hande_closed` | `STOPPED_CLOSING_DETECTED` (2) | `true` |
| Close on thick object (20mm) | `hande_closed` | `STOPPED_CLOSING_DETECTED` (2) | `true` |
| Close on empty air | `hande_closed` | `REQ_POS_NO_OBJECT` (3) | `false` |
| Open after grasp | `hande_open` | `REQ_POS_NO_OBJECT` (3) | `false` |
| Pick and place (success) | `pick_and_place` | Completes without error | N/A |
| Pick and place (miss) | `pick_and_place` (no object) | `GRASP_LOST` error | N/A |

#### 4d. Regression tests

1. **ePick full workflow**: Run existing ePick pick_and_place → verify no regressions
2. **Tool exchange**: Switch from Hand-E to ePick → verify vacuum monitoring activates, Hand-E monitoring deactivates
3. **Orchestrator batching**: Verify moveto-only tasks (no gripper) still batch correctly
4. **MoveIt restart**: After tool exchange, verify Hand-E status publisher controller restarts

### Manual Smoke Tests

1. **MCP end-to-end**: Connect via MCP, call `get_robot_state()`, send pick_and_place, call `get_gripper_state()` after grasp
2. **Object slip simulation**: Grasp object, during transport manually pull object out, verify `GRASP_LOST` error
3. **Thin object edge case**: Test with objects near the `goal_tolerance` (0.02m) boundary — verify detection works

---

## Summary of Files Changed

| Phase | File | Change Type |
|-------|------|-------------|
| 1 | `src/end_effectors/robotiq_hande_driver/.../bringup/config/hande_controller.yaml` | Config fix |
| 2a | `src/end_effectors/robotiq_hande_driver/.../hardware/include/.../hande_hardware_interface.hpp` | Add state variable |
| 2a | `src/end_effectors/robotiq_hande_driver/.../hardware/src/hande_hardware_interface.cpp` | Add GPIO state interface + update in read loop |
| 2a | `src/end_effectors/robotiq_hande_driver/.../hardware/include/.../hande_gripper.hpp` | Add `get_object_detection_status()` |
| 2a | `src/end_effectors/robotiq_hande_driver/.../hardware/src/hande_gripper.cpp` | Implement `get_object_detection_status()` |
| 2a | `src/end_effectors/robotiq_hande_driver/.../hardware/include/.../protocol_logic.hpp` | Add `get_object_detection_status()` |
| 2a | `src/end_effectors/robotiq_hande_driver/.../hardware/src/protocol_logic.cpp` | Implement `get_object_detection_status()` |
| 2a | `src/end_effectors/robotiq_hande_description/urdf/robotiq_hande_gripper.ros2_control.xacro` | Add GPIO declaration |
| 2a | New: `hande_status_publisher_controller.hpp` + `.cpp` | New controller (mirror ePick pattern) |
| 2a | `hande_controller.yaml` + `ur_hande_controllers.yaml` | Register new controller |
| 2b | `src/beambot/mcp/beambot_mcp_server.py` | Add subscription + `get_gripper_state()` + update `get_robot_state()` |
| 3 | `src/beambot/beambot/action_servers/orchestrator.py` | Generalize grasp watchdog for Hand-E |
| 3 | `CLAUDE.md` | Update error taxonomy |
| 4 | New: `src/beambot/tests/test_grasp_watchdog.py` | Unit tests |
| 4 | New: `src/beambot/tests/test_mcp_gripper_tools.py` | MCP tool tests |

## Build Order

```bash
# Phase 1
colcon build --packages-select robotiq_hande_driver

# Phase 2a (C++ changes)
colcon build --packages-select robotiq_hande_driver robotiq_hande_description

# Phase 2b (Python changes)
colcon build --packages-select beambot

# Phase 3
colcon build --packages-select beambot

# Phase 4
colcon test --packages-select beambot
```

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| GPIO state interface name mismatch | Medium | Controller fails to start | Test in isolation before integration |
| Hand-E status not published fast enough after close | Low | False GRASP_LOST on immediate check | Add configurable delay or rely on MTC completion timing |
| Batching regression for non-gripper tasks | Low | Performance loss | Test moveto-only tasks still batch |
| ePick regression from refactored names | Medium | Broken vacuum monitoring | Keep ePick internal logic unchanged, only rename wrappers |
| Velocity bug causes unexpected stall behavior | Low | Premature stall detection | Not blocking — stall timeout (1.0s) > close time (~0.2s) |
