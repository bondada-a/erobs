# Bluesky-ROS Integration - Complete Control Flow Documentation

**Repository**: erobs (Experimental Robotics Beamline System)
**Module**: `/home/aditya/work/github_ws/erobs/src/bluesky_ros/`
**Purpose**: Bridge between Bluesky data acquisition framework (NSLS-II) and ROS 2 robotic control
**Language**: Python 3
**Key Technologies**: Bluesky, Ophyd, ROS 2, MoveIt Task Constructor (MTC)

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Class Hierarchies](#class-hierarchies)
3. [Core Control Flows](#core-control-flows)
4. [State Management](#state-management)
5. [Implementation Approaches](#implementation-approaches)
6. [Sequence Diagrams](#sequence-diagrams)
7. [Error Handling](#error-handling)
8. [Design Insights](#design-insights)

---

## Architecture Overview

### System Context

This integration solves a critical problem: **enabling Bluesky experimental plans to control ROS 2 robotic systems** at synchrotron beamlines. Bluesky is a Python framework for orchestrating data acquisition at NSLS-II (National Synchrotron Light Source II), while ROS 2 provides real-time robotic control.

### Three-Layer Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    BLUESKY LAYER                            │
│  ┌──────────────┐         ┌─────────────────┐             │
│  │ RunEngine    │────────▶│ Experimental    │             │
│  │              │         │ Plans           │             │
│  └──────────────┘         └─────────────────┘             │
│         │                         │                         │
│         │ executes               │ yield from               │
│         ▼                         ▼                         │
│  ┌──────────────────────────────────────────┐             │
│  │  bps.abs_set(device, value, wait=True)   │             │
│  └──────────────────────────────────────────┘             │
└─────────────────────────────────────────────────────────────┘
                        │
                        │ calls device.set(value)
                        ▼
┌─────────────────────────────────────────────────────────────┐
│                    OPHYD LAYER                              │
│  ┌──────────────────────────────────────────┐             │
│  │ Movable Protocol                          │             │
│  │   - set(value) → Status                  │             │
│  │   - Provides hardware abstraction         │             │
│  └──────────────────────────────────────────┘             │
│         │                                                   │
│         │ implements                                        │
│         ▼                                                   │
│  ┌──────────────────────────────────────────┐             │
│  │ ActionMovable / MTCExecutionDevice        │             │
│  │   - Wraps ROS 2 action as Ophyd device   │             │
│  │   - Manages ActionStatus lifecycle        │             │
│  └──────────────────────────────────────────┘             │
└─────────────────────────────────────────────────────────────┘
                        │
                        │ sends goals via ActionClient
                        ▼
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 LAYER                              │
│  ┌──────────────────────────────────────────┐             │
│  │ Action Server (mtc_execution)            │             │
│  │   - Executes MTC tasks                   │             │
│  │   - Provides feedback & results          │             │
│  │   - Controls UR5e robot via MoveIt       │             │
│  └──────────────────────────────────────────┘             │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow Pipeline

```
JSON Task → Bluesky Plan → Ophyd Device → ROS 2 Action → MoveIt → Robot
    ↓           ↓              ↓              ↓            ↓        ↓
 Config     RunEngine    ActionMovable   ActionServer  MTC Task  UR5e
```

### Key Concepts

**Bluesky RunEngine**: Event-driven execution engine that processes generator-based plans. It coordinates device movements and data collection while emitting documents for metadata capture.

**Ophyd Device**: Hardware abstraction layer that wraps physical devices (motors, detectors) or in this case, robotic systems. Must implement the `Movable` protocol.

**Movable Protocol**: Bluesky interface requiring `set(value)` method that returns a `Status` object tracking operation completion.

**ROS 2 Action**: Three-part communication pattern providing goal submission, feedback during execution, and final result. Suitable for long-running tasks like robotic manipulation.

**MTC (MoveIt Task Constructor)**: High-level motion planning framework that chains primitive stages (move, grasp, place) into complex manipulation tasks.

---

## Class Hierarchies

### Overview

```
┌──────────────────────────────────────────────────────────────┐
│                    CLASS HIERARCHY                           │
└──────────────────────────────────────────────────────────────┘

Python ABC Protocols          ROS 2 Classes
─────────────────            ──────────────
┌──────────────┐             ┌──────────────┐
│   Movable    │             │     Node     │
│  (Protocol)  │             │  (rclpy)     │
└──────────────┘             └──────────────┘
       ▲                            ▲
       │                            │
       │ implements                 │ inherits
       │                            │
       └────────────┬───────────────┘
                    │
         ┌──────────────────────┐
         │  ActionMovable       │ ◄────── Base Implementation
         │  (ophyd_ros.py)      │         (Abstract)
         └──────────────────────┘
                    │
                    │ extends
                    ▼
         ┌──────────────────────┐
         │ MTCExecutionDevice   │ ◄────── Concrete Implementation
         │ (mtc_ophyd_device.py)│         (MTC-specific)
         └──────────────────────┘


Status Tracking Hierarchy
──────────────────────────
┌──────────────┐
│ DeviceStatus │  (ophyd.status)
│  (Base)      │
└──────────────┘
       ▲
       │ inherits
       │
┌──────────────┐
│ ActionStatus │  (Custom - ophyd_ros.py)
│              │  Adds ROS-specific failure handling
└──────────────┘
```

### ActionMovable Base Class (ophyd_ros.py)

**File**: `/home/aditya/work/github_ws/erobs/src/bluesky_ros/ophyd_ros.py`
**Lines**: 28-177
**Purpose**: Abstract base class providing reusable ROS 2 Action → Bluesky integration infrastructure

**Inheritance**:
- Inherits from: `rclpy.node.Node` (ROS 2 functionality)
- Implements: `bluesky.protocols.Movable` (Bluesky device interface)

**Key Responsibilities**:
1. **ROS 2 Node Management**: Initializes and manages ROS 2 node lifecycle
2. **Action Client Setup**: Creates and manages ActionClient for server communication
3. **Future Coordination**: Manages multiple Future objects for async operations
4. **Status Synchronization**: Bridges ROS 2 action status to Bluesky ActionStatus
5. **Callback Orchestration**: Coordinates goal response, feedback, and result callbacks

**Abstract Methods** (must be implemented by subclasses):

| Method | Purpose | Line |
|--------|---------|------|
| `action_type` (property) | Define the ROS 2 action message type | 73-77 |
| `construct_goal_message()` | Build action goal from user input | 79-82 |
| `get_result_callback()` | Process final action result | 84-87 |
| `feedback_callback()` | Handle feedback during execution | 100-103 |

**Member Variables**:

| Variable | Type | Purpose | Init Line |
|----------|------|---------|-----------|
| `_action_client` | `ActionClient` | ROS 2 action client | 59 |
| `_goal_handle` | `ClientGoalHandle` | Handle to track submitted goal | 60 |
| `_send_goal_future` | `Future` | Tracks goal submission | 63 |
| `_get_result_future` | `Future` | Tracks result retrieval | 64 |
| `_bluesky_status` | `ActionStatus` | Bluesky status object | 65 |
| `_finalize_future` | `Future` | Controls spin loop termination | 66 |

### MTCExecutionDevice Concrete Class (mtc_ophyd_device.py)

**File**: `/home/aditya/work/github_ws/erobs/src/bluesky_ros/mtc_ophyd_device.py`
**Lines**: 21-125
**Purpose**: Concrete implementation for MoveIt Task Constructor execution

**Inheritance**:
- Inherits from: `rclpy.node.Node` + `bluesky.protocols.Movable`
- Does NOT inherit from ActionMovable (alternative implementation)

**Key Differences from ActionMovable**:
1. Uses `spin_once()` loop instead of `spin_until_future_complete()`
2. Directly implements all methods (not using base class abstraction)
3. Simpler structure for single-purpose use case
4. Hardcoded action type loading via `get_action()`

**Configuration**:
- **Action Server**: `'mtc_execution'`
- **Action Type**: `'mtc_pipeline/MTCExecution'`
- **Default Robot IP**: `'192.168.56.101'`

**Concrete Implementations**:

| Method | Purpose | Lines |
|--------|---------|-------|
| `construct_goal_message()` | Parse JSON file/string into MTCExecution.Goal | 41-54 |
| `_feedback_callback()` | Log task progress (step, action, percentage) | 79-85 |
| `_result_callback()` | Handle completion/abort/cancel outcomes | 100-117 |

---

## Core Control Flows

### Flow 1: Complete Execution Pipeline (ActionMovable)

**Entry Point**: User calls `device.set(value)` from Bluesky plan
**File**: `ophyd_ros.py`

```
COMPLETE EXECUTION FLOW (ActionMovable)
════════════════════════════════════════════════════════════════
    [Bluesky Plan Execution]
       │
       │ yield from bps.abs_set(device, value)
       ▼
    set(value) METHOD (line 157)
       │
       ├─> Create ActionStatus object (line 160)
       ├─> Call _send_goal(value) (line 159)
       └─> Start spin loop (line 161)
       │
       ▼
    ┌─────────────────────────────────────────┐
    │ PHASE 1: GOAL CONSTRUCTION & SUBMISSION │
    └─────────────────────────────────────────┘
       │
    _send_goal(goal) (line 134)
       │
       ├─> Parse goal type (dict/iterable/object) (lines 135-140)
       ├─> Call construct_goal_message(**goal) [ABSTRACT]
       ├─> Validate goal type (lines 142-143)
       └─> Wait for action server (line 145, timeout=10s)
       │
       ▼
    _action_client.send_goal_async() (line 147)
       │
       ├─> Send goal message to server
       ├─> Register feedback_callback [ABSTRACT]
       └─> Add done callback → _goal_response_callback
       │
       ▼
    ┌─────────────────────────────────────────┐
    │ PHASE 2: GOAL RESPONSE HANDLING         │
    └─────────────────────────────────────────┘
       │
    _goal_response_callback(future) (line 121)
       │
       ├─> Extract goal_handle from future (line 123)
       │
       ▼
    ┌─────────────────────────┐
    │ Goal accepted?          │ (line 124)
    └─────────────────────────┘
       │
       ├───[NO]──> Log "Goal rejected" (line 125)
       │           └─> RETURN (no status update)
       │
       └───[YES]─> Log "Goal accepted" (line 128)
                   │
                   ├─> Store goal_handle (line 130)
                   ├─> Call get_result_async() (line 131)
                   └─> Add callback → _stop_spin_callback (line 132)
       │
       ▼
    ┌─────────────────────────────────────────┐
    │ PHASE 3: EXECUTION & FEEDBACK           │
    └─────────────────────────────────────────┘
       │
    [Parallel Execution]
       │
       ├──> ROS 2 Action Server executing
       │    └──> Sends feedback messages periodically
       │         └──> feedback_callback() invoked [ABSTRACT]
       │
       └──> rclpy.spin_until_future_complete() (line 161)
            └──> Blocks until _finalize_future is set
       │
       ▼
    [Action completes on server side]
       │
       ▼
    ┌─────────────────────────────────────────┐
    │ PHASE 4: RESULT PROCESSING              │
    └─────────────────────────────────────────┘
       │
    _stop_spin_callback(future) (line 89)
       │
       ├─> Check future.done() (line 91)
       ├─> Call get_result_callback(future) [ABSTRACT] (line 94)
       ├─> Call _update_bluesky_status(future) (line 96)
       │   │
       │   └──> _update_bluesky_status() (line 164)
       │        │
       │        ├─> Check for exception (line 169)
       │        │   └─[YES]─> status.set_exception()
       │        │
       │        └─> Check future.done() (line 171)
       │            └─[YES]─> status.set_finished()
       │
       └─> Set _finalize_future.set_result(True) (line 98)
       │
       ▼
    [spin_until_future_complete exits]
       │
       ▼
    set() returns ActionStatus (line 162)
       │
       ▼
    [Bluesky Plan Continues]
       │
       ▼
    [EXIT]
```

### Flow 2: MTCExecutionDevice Execution

**Entry Point**: `MTCExecutionDevice.set(json_path_or_string)`
**File**: `mtc_ophyd_device.py`

```
MTC EXECUTION FLOW (MTCExecutionDevice)
════════════════════════════════════════════════════════════════
    [Bluesky Plan Execution]
       │
       │ yield from bps.abs_set(mtc_device, json_file)
       ▼
    set(json_path_or_string) METHOD (line 56)
       │
       ├─> Create ActionStatus (line 58)
       └─> Construct goal message (line 61)
       │
       ▼
    construct_goal_message() (line 41)
       │
       ├─> Create MTCExecution.Goal object (line 43)
       │
       ▼
    ┌─────────────────────────┐
    │ Is input a .json file?  │ (line 46)
    └─────────────────────────┘
       │
       ├───[YES]─> Open and read file (lines 47-48)
       │           └─> goal.full_json = file_contents
       │
       └───[NO]──> Treat as JSON string (line 51)
                   └─> goal.full_json = json_path_or_string
       │
       ├─> Set goal.robot_ip (line 53)
       │
       ▼
    [Return to set() method]
       │
       ├─> Wait for action server (lines 63-64, timeout=10s)
       ├─> Log "Sending goal..." (line 66)
       │
       ▼
    _action_client.send_goal_async() (line 67)
       │
       ├─> Register _feedback_callback (line 69)
       └─> Add done callback → _goal_response_callback (line 71)
       │
       ▼
    ┌─────────────────────────────────────────┐
    │ SPIN LOOP (Poll-based)                  │
    └─────────────────────────────────────────┘
       │
    while not status.done: (line 74)
       │
       ├─> rclpy.spin_once(self, timeout=0.1) (line 75)
       │   └─> Process callbacks for 100ms
       │
       └─> Loop continues until status.done = True
       │
       ▼
    ┌─────────────────────────────────────────┐
    │ GOAL RESPONSE HANDLING                  │
    └─────────────────────────────────────────┘
       │
    _goal_response_callback(future) (line 87)
       │
       ├─> Extract goal_handle (line 89)
       │
       ▼
    ┌─────────────────────────┐
    │ Goal accepted?          │ (line 90)
    └─────────────────────────┘
       │
       ├───[NO]──> Log error (line 91)
       │           └─> status.set_exception() (line 92)
       │               └─> status.done = True → exits spin loop
       │
       └───[YES]─> Log "Goal accepted" (line 95)
                   │
                   ├─> Store goal_handle (line 96)
                   ├─> Call get_result_async() (line 97)
                   └─> Add callback → _result_callback (line 98)
       │
       ▼
    ┌─────────────────────────────────────────┐
    │ FEEDBACK PROCESSING (during execution)  │
    └─────────────────────────────────────────┘
       │
    _feedback_callback(feedback_msg) (line 79)
       │
       ├─> Extract feedback fields (line 81)
       │   ├─> current_step
       │   ├─> current_action
       │   ├─> progress_percentage
       │   └─> status_message
       │
       └─> Log formatted feedback (lines 82-85)
       │
       ▼
    [Continue spinning until result arrives]
       │
       ▼
    ┌─────────────────────────────────────────┐
    │ RESULT HANDLING                         │
    └─────────────────────────────────────────┘
       │
    _result_callback(future) (line 100)
       │
       ├─> Extract result object (line 102)
       │
       ▼
    ┌─────────────────────────┐
    │ Switch on result.status │ (line 104)
    └─────────────────────────┘
       │
       ├───[status == 4 (SUCCEEDED)]
       │   ├─> Log completion info (lines 105-107)
       │   │   └─> "completed_steps/total_steps"
       │   └─> status.set_finished() (line 108)
       │       └─> status.done = True
       │
       ├───[status == 5 (ABORTED)]
       │   ├─> Log error message (line 110)
       │   └─> status.set_exception() (line 111)
       │       └─> status.done = True
       │
       ├───[status == 6 (CANCELED)]
       │   ├─> Log warning (line 113)
       │   └─> status.set_exception() (line 114)
       │       └─> status.done = True
       │
       └───[other status]
           ├─> Log unknown status (line 116)
           └─> status.set_exception() (line 117)
               └─> status.done = True
       │
       ▼
    [Spin loop detects status.done = True]
       │
       ▼
    [Exit while loop] (line 74)
       │
       ▼
    set() returns ActionStatus (line 77)
       │
       ▼
    [Bluesky Plan Continues]
       │
       ▼
    [EXIT]
```

### Flow 3: Subprocess-Based Execution (simple_mtc_bluesky.py)

**Entry Point**: `MTCDevice.set(task_params)`
**File**: `simple_mtc_bluesky.py`

```
SUBPROCESS-BASED EXECUTION FLOW
════════════════════════════════════════════════════════════════
    [Bluesky Plan]
       │
       │ yield from bps.abs_set(mtc_device, task_params)
       ▼
    set(task_params) METHOD (line 26)
       │
       ├─> Extract json_file_path (line 32)
       ├─> Extract robot_ip (line 33)
       └─> Create Status object (line 36)
       │
       ▼
    try: (line 38)
       │
       ├─> Construct subprocess command (lines 40-41)
       │   │
       │   └─> ['ros2', 'run', 'mtc_pipeline',
       │        'mtc_action_client_example',
       │        json_file_path, robot_ip, '300']
       │
       ├─> Log execution message (line 43)
       │
       ▼
    subprocess.run() (line 45)
       │
       ├─> capture_output=True (capture stdout/stderr)
       ├─> text=True (decode as UTF-8 strings)
       └─> timeout=300 (5 minute timeout)
       │
       ▼
    [External C++ process executes]
       │
       ├─> Loads JSON file
       ├─> Connects to ROS 2 action server
       ├─> Sends MTCExecution goal
       ├─> Waits for completion
       └─> Returns exit code
       │
       ▼
    [subprocess.run() returns]
       │
       ▼
    ┌─────────────────────────┐
    │ returncode == 0?        │ (line 47)
    └─────────────────────────┘
       │
       ├───[YES (SUCCESS)]
       │   ├─> Print success message (line 48)
       │   └─> status.set_finished() (line 49)
       │
       └───[NO (FAILURE)]
           ├─> Print error with stderr (line 51)
           └─> status.set_exception() (line 52)
       │
       ▼
    [EXIT try block]
       │
       ▼
except subprocess.TimeoutExpired: (line 54)
       │
       ├─> Print timeout message (line 55)
       └─> status.set_exception() (line 56)
       │
       ▼
except Exception as e: (line 57)
       │
       ├─> Print error message (line 58)
       └─> status.set_exception() (line 59)
       │
       ▼
    return status (line 61)
       │
       ▼
    [Bluesky Plan Continues]
       │
       ▼
    [EXIT]


COMPARISON: Native vs Subprocess
════════════════════════════════════════════════════════════════

Native Python Approach (MTCExecutionDevice):
    ✓ Real-time feedback during execution
    ✓ Fine-grained control (can cancel mid-execution)
    ✓ Direct ROS 2 integration (same process)
    ✗ More complex code (callbacks, futures)
    ✗ Requires rclpy in same Python environment

Subprocess Approach (MTCDevice):
    ✓ Simple implementation (blocking call)
    ✓ Isolated execution (separate process)
    ✓ No callback complexity
    ✗ No real-time feedback (waits for completion)
    ✗ Cannot cancel mid-execution
    ✗ Depends on external C++ client binary
```

### Flow 4: Goal Cancellation

**Trigger**: ActionStatus failure handling or user interruption
**Files**: Both `ophyd_ros.py` and `mtc_ophyd_device.py`

```
GOAL CANCELLATION FLOW
════════════════════════════════════════════════════════════════
    [User Interrupt or Status Failure]
       │
       │ Ctrl+C or status._handle_failure()
       ▼
    ActionStatus._handle_failure() (ophyd_ros.py line 23)
       │
       └─> device.cancel_goal() (line 25)
       │
       ▼
    ┌──────────────────────────────────────────┐
    │ OPHYD_ROS.PY CANCELLATION (line 105)     │
    └──────────────────────────────────────────┘
       │
    ┌─────────────────────────┐
    │ _goal_handle exists?    │ (line 107)
    └─────────────────────────┘
       │
       ├───[NO]──> RETURN (nothing to cancel)
       │
       └───[YES]─> call cancel_goal_async() (line 108)
                   │
                   ├─> Send cancel request to server
                   └─> Add done callback → cancel_done (line 109)
       │
       ▼
    cancel_done(future) (line 111)
       │
       ├─> Get cancel_response (line 113)
       │
       ▼
    ┌─────────────────────────┐
    │ goals_canceling > 0?    │ (line 114)
    └─────────────────────────┘
       │
       ├───[YES]─> Log "Goal successfully canceled" (line 115)
       │
       └───[NO]──> Log "Goal failed to cancel" (line 117)
       │
       └─> rclpy.shutdown() (line 119)
       │
       ▼
    [EXIT]

    ┌──────────────────────────────────────────┐
    │ MTC_OPHYD_DEVICE.PY CANCELLATION (line 119) │
    └──────────────────────────────────────────┘
       │
    ┌─────────────────────────┐
    │ _goal_handle exists?    │ (line 121)
    └─────────────────────────┘
       │
       ├───[NO]──> RETURN (nothing to cancel)
       │
       └───[YES]─> Log "Canceling goal..." (line 122)
                   │
                   ├─> call cancel_goal_async() (line 123)
                   └─> spin_until_future_complete() (line 124)
                       └─> timeout=5.0 seconds
       │
       ▼
    [Server processes cancellation]
       │
       ▼
    [Result callback receives CANCELED status]
       │
       └─> status.set_exception(Exception("Canceled"))
       │
       ▼
    [EXIT]
```

---

## State Management

### ActionStatus Lifecycle

**Purpose**: Track the completion state of long-running ROS 2 actions and synchronize with Bluesky's event loop.

```
ACTIONSTATUS STATE MACHINE
════════════════════════════════════════════════════════════════

    [Created]
       │
       │ ActionStatus(device)
       ▼
    ┌─────────────┐
    │   PENDING   │ ← Initial state
    │             │   (status.done = False)
    └─────────────┘   (status.success = None)
       │
       │ Action executing on ROS 2 server
       │ (No state change yet)
       │
       ▼
    ┌──────────────────────────────┐
    │  Waiting for Completion      │
    │                              │
    │  - Spin loop active          │
    │  - Callbacks registered      │
    │  - Futures pending           │
    └──────────────────────────────┘
       │
       │ Result arrives
       ▼
    ┌─────────────────────────┐
    │ Which method called?    │
    └─────────────────────────┘
       │
       ├───[set_finished()]──────────────┐
       │   (ophyd_ros.py line 172)       │
       │   (mtc_ophyd_device.py line 108)│
       │                                  │
       │   ┌─────────────┐               │
       │   │  FINISHED   │               │
       │   │             │               │
       │   │ done = True │               │
       │   │ success=True│               │
       │   └─────────────┘               │
       │                                  │
       └───[set_exception(exc)]──────────┤
           (ophyd_ros.py line 170)       │
           (mtc_ophyd_device.py line 92, │
            111, 114, 117)                │
                                          │
           ┌─────────────┐               │
           │   FAILED    │               │
           │             │               │
           │ done = True │               │
           │ success=False│              │
           │ exception=exc│              │
           └─────────────┘               │
                                          │
       ┌──────────────────────────────────┘
       │
       ▼
    [Bluesky RunEngine Notified]
       │
       │ Plan continues or handles exception
       ▼
    [EXIT]


KEY STATE TRANSITIONS
════════════════════════════════════════════════════════════════

1. PENDING → FINISHED
   Trigger: status.set_finished()
   Condition: ROS 2 action succeeded (status code 4)
   Effect:
   - status.done = True
   - status.success = True
   - Bluesky plan proceeds

2. PENDING → FAILED
   Trigger: status.set_exception(exc)
   Conditions:
   - Goal rejected by server
   - Action aborted (status code 5)
   - Action canceled (status code 6)
   - Unknown status code
   - Python exception during execution
   Effect:
   - status.done = True
   - status.success = False
   - status.exception contains error
   - Bluesky plan raises exception

3. FAILED (on interrupt) → CANCELING
   Trigger: User interrupt (Ctrl+C)
   Path: ActionStatus._handle_failure() → device.cancel_goal()
   Effect: Send cancel request to server
```

### Future Objects Coordination

**Purpose**: Manage asynchronous ROS 2 operations and coordinate callback execution.

```
FUTURE OBJECTS LIFECYCLE (ActionMovable)
════════════════════════════════════════════════════════════════

Member Variables (ophyd_ros.py lines 63-66):
    _send_goal_future: Future      # Tracks goal submission
    _get_result_future: Future     # Tracks result retrieval
    _finalize_future: Future       # Controls spin loop
    _bluesky_status: ActionStatus  # Bluesky status object


Timeline:
════════════════════════════════════════════════════════════════

T0: set() called
    │
    └─> _finalize_future = Future() created (line 66)
        └─> Initially not done

T1: _send_goal() called (line 159)
    │
    └─> _send_goal_future = send_goal_async(...) (line 147)
        ├─> Future created by ActionClient
        └─> done_callback: _goal_response_callback

T2: spin_until_future_complete(_finalize_future) (line 161)
    │
    └─> BLOCKS until _finalize_future.done() = True

T3: _send_goal_future completes
    │
    └─> _goal_response_callback(future) invoked (line 121)
        │
        └─> if accepted:
            └─> _get_result_future = get_result_async() (line 131)
                ├─> Future created by GoalHandle
                └─> done_callback: _stop_spin_callback

T4: Action executing on server
    │
    └─> Feedback callbacks fire (periodic)
        └─> feedback_callback(msg) invoked [ABSTRACT]

T5: _get_result_future completes
    │
    └─> _stop_spin_callback(future) invoked (line 89)
        │
        ├─> get_result_callback(future) [ABSTRACT] (line 94)
        ├─> _update_bluesky_status(future) (line 96)
        │   └─> status.set_finished() or set_exception()
        │
        └─> _finalize_future.set_result(True) (line 98)
            └─> UNBLOCKS spin loop

T6: spin_until_future_complete returns
    │
    └─> set() returns _bluesky_status (line 162)


CALLBACK CHAIN VISUALIZATION
════════════════════════════════════════════════════════════════

send_goal_async()
    │
    └─> _send_goal_future
        │
        └─> done → _goal_response_callback()
                    │
                    └─> get_result_async()
                        │
                        └─> _get_result_future
                            │
                            └─> done → _stop_spin_callback()
                                        │
                                        ├─> get_result_callback()
                                        ├─> _update_bluesky_status()
                                        └─> _finalize_future.set_result()
                                            └─> UNBLOCK spin
```

### ROS 2 Spin Cycle Management

**Two Approaches**:

```
APPROACH 1: spin_until_future_complete (ActionMovable)
════════════════════════════════════════════════════════════════
Location: ophyd_ros.py line 161

rclpy.spin_until_future_complete(self, self._finalize_future)

Behavior:
    - BLOCKS current thread
    - Continuously processes ROS 2 callbacks
    - Exits when _finalize_future.done() = True
    - Event-driven (efficient CPU usage)

Pros:
    ✓ Clean code (no manual loop)
    ✓ Efficient (event-driven)
    ✓ Proper callback ordering guaranteed

Cons:
    ✗ Harder to interrupt
    ✗ Must manage _finalize_future carefully


APPROACH 2: spin_once loop (MTCExecutionDevice)
════════════════════════════════════════════════════════════════
Location: mtc_ophyd_device.py lines 74-75

while not self._bluesky_status.done:
    rclpy.spin_once(self, timeout_sec=0.1)

Behavior:
    - Manual polling loop
    - Processes callbacks for 100ms per iteration
    - Exits when status.done = True
    - Polling-based (fixed interval)

Pros:
    ✓ Easy to understand
    ✓ Easy to interrupt/modify
    ✓ Simple status checking

Cons:
    ✗ Polling overhead (wastes CPU cycles)
    ✗ 100ms latency before detecting completion
    ✗ More verbose code


COMPARISON TABLE
════════════════════════════════════════════════════════════════
│ Feature              │ spin_until_future │ spin_once loop │
├──────────────────────┼───────────────────┼────────────────┤
│ Blocking             │ Yes               │ Yes            │
│ CPU Efficiency       │ High (event)      │ Low (polling)  │
│ Response Latency     │ Immediate         │ Up to 100ms    │
│ Code Complexity      │ Medium            │ Low            │
│ Interrupt Handling   │ Harder            │ Easier         │
│ Callback Ordering    │ Guaranteed        │ Manual         │
│ Production Use       │ Preferred         │ Quick prototyping │
```

---

## Implementation Approaches

### Approach 1: Native Python ROS 2 (ActionMovable + MTCExecutionDevice)

**Files**:
- Base: `/home/aditya/work/github_ws/erobs/src/bluesky_ros/ophyd_ros.py`
- Concrete: `/home/aditya/work/github_ws/erobs/src/bluesky_ros/mtc_ophyd_device.py`
- Example: `/home/aditya/work/github_ws/erobs/src/bluesky_ros/mtc_bluesky_example.py`

**Architecture**:
```
┌────────────────────────────────────────────────────────┐
│                  Bluesky Process                       │
│                                                        │
│  ┌──────────────┐         ┌──────────────────┐       │
│  │  RunEngine   │────────▶│ MTCExecutionDev  │       │
│  └──────────────┘         └──────────────────┘       │
│                                   │                    │
│                                   │ rclpy               │
│                                   │ (in-process)       │
└───────────────────────────────────┼────────────────────┘
                                    │ ROS 2 DDS
                                    ▼
                        ┌───────────────────────┐
                        │ mtc_execution Server  │
                        │ (separate process)    │
                        └───────────────────────┘
```

**Key Characteristics**:

| Aspect | Details |
|--------|---------|
| **Integration Type** | Direct ROS 2 client in Python |
| **Communication** | rclpy ActionClient → DDS → Action Server |
| **Feedback** | Real-time via callbacks during execution |
| **Cancellation** | Supported via `cancel_goal_async()` |
| **Error Handling** | Exception-based with ActionStatus |
| **Dependencies** | rclpy, action message types in Python |

**Execution Flow**:
1. MTCExecutionDevice initializes as ROS 2 Node
2. Creates ActionClient for 'mtc_execution' action
3. `set()` constructs goal from JSON file/string
4. Sends goal asynchronously with feedback callback
5. Spins with `spin_once()` until completion
6. Processes result and updates ActionStatus
7. Returns status to Bluesky

**Code Example** (mtc_bluesky_example.py):
```python
# Lines 25-41
def main():
    rclpy.init()
    try:
        # Create MTC device
        mtc = MTCExecutionDevice(
            name="mtc_executor",
            robot_ip="10.68.82.41"
        )

        # Create Bluesky RunEngine
        RE = RunEngine({})

        # Execute single task
        RE(single_task_plan(mtc, "/root/ws/erobs/beamline_test.json"))
    finally:
        rclpy.shutdown()
```

**Advantages**:
- Real-time progress feedback (current step, percentage, status messages)
- Fine-grained control (can cancel mid-execution)
- Native ROS 2 integration (no subprocess overhead)
- Extensible (easy to add custom callbacks)
- Type-safe (Python action messages)

**Disadvantages**:
- More complex code (futures, callbacks, state management)
- Requires rclpy in same Python environment as Bluesky
- Harder to debug (async callback chains)
- More potential failure modes (network, serialization, etc.)

### Approach 2: Subprocess Wrapper (simple_mtc_bluesky.py)

**File**: `/home/aditya/work/github_ws/erobs/src/bluesky_ros/simple_mtc_bluesky.py`

**Architecture**:
```
┌────────────────────────────────────────────────────────┐
│                  Bluesky Process                       │
│                                                        │
│  ┌──────────────┐         ┌──────────────────┐       │
│  │  RunEngine   │────────▶│  MTCDevice       │       │
│  └──────────────┘         └──────────────────┘       │
│                                   │                    │
│                                   │ subprocess.run()   │
│                                   ▼                    │
└───────────────────────────────────┼────────────────────┘
                                    │ exec()
                        ┌───────────▼───────────┐
                        │  C++ Process          │
                        │                       │
                        │  mtc_action_client_   │
                        │  example              │
                        │                       │
                        └───────────┬───────────┘
                                    │ ROS 2 DDS
                                    ▼
                        ┌───────────────────────┐
                        │ mtc_execution Server  │
                        │ (separate process)    │
                        └───────────────────────┘
```

**Key Characteristics**:

| Aspect | Details |
|--------|---------|
| **Integration Type** | Subprocess wrapper around C++ client |
| **Communication** | subprocess → C++ client → DDS → Server |
| **Feedback** | None (blocking wait for completion) |
| **Cancellation** | Not supported (subprocess runs to completion) |
| **Error Handling** | Exit codes and stderr capture |
| **Dependencies** | Compiled C++ binary, ros2 CLI available |

**Execution Flow**:
1. MTCDevice is simple Python class (not ROS 2 Node)
2. `set()` creates Status object
3. Constructs subprocess command with arguments
4. Calls `subprocess.run()` with 300s timeout
5. Blocks until C++ client exits
6. Checks exit code (0 = success, non-zero = failure)
7. Updates Status based on result
8. Returns status to Bluesky

**Code Example** (simple_mtc_bluesky.py):
```python
# Lines 26-61
class MTCDevice:
    def set(self, task_params):
        json_file_path = task_params['json_file']
        robot_ip = task_params['robot_ip']

        status = Status()

        try:
            # Launch external C++ client
            cmd = ['ros2', 'run', 'mtc_pipeline',
                   'mtc_action_client_example',
                   json_file_path, robot_ip, '300']

            result = subprocess.run(cmd, capture_output=True,
                                    text=True, timeout=300)

            if result.returncode == 0:
                status.set_finished()
            else:
                status.set_exception(
                    Exception(f"MTC task failed: {result.stderr}")
                )
        except subprocess.TimeoutExpired:
            status.set_exception(Exception("MTC task timed out"))
        except Exception as e:
            status.set_exception(e)

        return status
```

**Advantages**:
- Extremely simple implementation (~40 lines)
- No complex async/callback code
- Isolated execution (separate process)
- Easy to debug (just check subprocess output)
- No Python ROS 2 dependencies (uses existing C++ client)
- Robust (C++ client is tested production code)

**Disadvantages**:
- No real-time feedback (can't see progress)
- Cannot cancel mid-execution (subprocess must complete)
- Subprocess overhead (process creation, IPC)
- Requires compiled C++ binary to exist
- Less flexible (hard to customize behavior)
- No direct ROS 2 integration

### Trade-off Analysis

```
WHEN TO USE EACH APPROACH
════════════════════════════════════════════════════════════════

Use Native Python (Approach 1) when:
    ✓ You need real-time feedback on task progress
    ✓ Cancellation/interruption is required
    ✓ You want to extend with custom callbacks
    ✓ You're building a production system with monitoring
    ✓ You want type-safe ROS 2 message handling
    ✓ Integration with Python ROS 2 ecosystem is important

Use Subprocess (Approach 2) when:
    ✓ Simplicity is the priority
    ✓ You have a working C++ client already
    ✓ You don't need progress feedback
    ✓ Tasks are relatively short (<5 minutes)
    ✓ You want to avoid Python ROS 2 dependencies
    ✓ You're prototyping or doing quick tests
    ✓ Process isolation is desirable


MIGRATION PATH
════════════════════════════════════════════════════════════════
Phase 1: Start with Approach 2 (subprocess)
    - Get basic integration working
    - Test end-to-end workflow
    - Validate task execution

Phase 2: Add monitoring with Approach 1
    - Implement MTCExecutionDevice
    - Add feedback callbacks for progress tracking
    - Keep subprocess as fallback

Phase 3: Production deployment
    - Use Approach 1 for interactive sessions
    - Use Approach 2 for automated batch processing
    - Implement both in parallel for redundancy
```

---

## Sequence Diagrams

### Diagram 1: Complete Execution Sequence (Native Python)

```
NATIVE PYTHON EXECUTION SEQUENCE
════════════════════════════════════════════════════════════════

Actor: User
Participant: BlueskyPlan
Participant: MTCExecutionDevice
Participant: ActionStatus
Participant: ActionClient
Participant: ActionServer
Participant: UR5e Robot


User ─────▶ BlueskyPlan: Execute plan
             │
             │ RE(plan)
             ▼
         [Plan starts]
             │
             │ yield from bps.abs_set(mtc, json_file)
             ▼
BlueskyPlan ─────▶ MTCExecutionDevice: set(json_file)
                   │
                   │ [line 58]
                   ├─────▶ ActionStatus: __init__(self)
                   │       │
                   │       └─────▶ ActionStatus created (done=False)
                   │
                   │ [line 61]
                   ├─────▶ self: construct_goal_message(json_file)
                   │       │
                   │       ├─ Read JSON file [line 47-48]
                   │       └─ Create Goal object
                   │
                   │ [line 64]
                   ├─────▶ ActionClient: wait_for_server(10s)
                   │       │
                   │       └─────▶ ActionClient: [Waits for server]
                   │
                   │ [line 67]
                   ├─────▶ ActionClient: send_goal_async(goal, feedback_cb)
                   │       │
                   │       └─────────────────────────▶ ActionServer: [Goal message]
                   │                                   │
                   │                                   ├─ Validate goal
                   │                                   └─ Accept goal
                   │                                   │
                   │       ◀─────────────────────────  │ [Accepted]
                   │       │
                   │       │ _send_goal_future done
                   │       ▼
                   │ ◀───  _goal_response_callback(future) [line 87]
                   │       │
                   │       ├─ goal_handle = future.result()
                   │       ├─ Check accepted [line 90]
                   │       └─ get_result_async() [line 97]
                   │
                   │ [line 74-75]
                   │ while not status.done:
                   │   spin_once(timeout=0.1)
                   │
                   │ [Spinning...] ─────────────────────────────▶ ActionServer: [Executing]
                   │                                               │
                   │                                               ├─ Plan trajectory
                   │                                               └─ Execute stages
                   │                                               │
                   │                                               └──────────────▶ UR5e: Move
                   │                                                               │
                   │ ◀───────────────────────────── Feedback ──   │               │
                   │ _feedback_callback() [line 79]               │               │
                   │   │                                            │               │
                   │   ├─ Log: "Step 1: MoveTo (25%)"             │               │
                   │   └─ Continue spinning                        │               │
                   │                                               │               │
                   │ ◀───────────────────────────── Feedback ──   │               │
                   │ _feedback_callback()                         │               │
                   │   │                                            │               │
                   │   ├─ Log: "Step 2: Grasp (50%)"              │               │
                   │   └─ Continue spinning                        │               │
                   │                                               │               │
                   │                                               │               ├─ Execute
                   │                                               │               │
                   │                                               │ ◀─────────────┘ Done
                   │                                               │
                   │       ◀─────────────────────────  Result ─   │ [Status: SUCCEEDED]
                   │       │
                   │       │ _get_result_future done
                   │       ▼
                   │ ◀───  _result_callback(future) [line 100]
                   │       │
                   │       ├─ result = future.result()
                   │       ├─ Check status == 4 (SUCCEEDED) [line 104]
                   │       ├─ Log completion [line 105-107]
                   │       └─────▶ ActionStatus: set_finished() [line 108]
                   │               │
                   │               └─────▶ done = True, success = True
                   │
                   │ [Detect status.done = True]
                   │ [Exit while loop]
                   │
                   │ return ActionStatus [line 77]
                   │
BlueskyPlan ◀──────┘
             │
             │ [Status indicates success]
             ▼
         [Plan continues]
             │
User ◀───────┘ Task complete


TIMING DIAGRAM
════════════════════════════════════════════════════════════════
Time │ MTCExecutionDevice │ ActionClient │ Server │ Robot
─────┼────────────────────┼──────────────┼────────┼────────
  0s │ set() called       │              │        │
  1s │ Goal constructed   │              │        │
  2s │ wait_for_server()  │ Connecting   │        │
  3s │ send_goal_async()  │ ────────────▶│ Accept │
  4s │ spin_once()        │              │ Plan   │
  5s │ spin_once()        │◀── Feedback ─│ Stage1 │
  6s │ spin_once()        │              │ Stage2 │
  7s │ spin_once()        │◀── Feedback ─│ Stage3 │ Moving
  8s │ spin_once()        │              │ Stage4 │ Grasping
  9s │ spin_once()        │◀── Result ───│ Done   │ Done
 10s │ status.done=True   │              │        │
 11s │ return status      │              │        │
```

### Diagram 2: Error Handling Sequence

```
ERROR HANDLING SEQUENCE
════════════════════════════════════════════════════════════════

Scenario 1: Goal Rejected by Server
────────────────────────────────────

MTCExecutionDevice ─────▶ ActionClient: send_goal_async()
                          │
                          └──────────▶ ActionServer: [Invalid goal]
                                      │
                          ◀───────────┘ [Rejected]
                          │
MTCExecutionDevice ◀──────┘ _goal_response_callback(future)
    │
    ├─ goal_handle.accepted == False [line 90]
    ├─ Log error: "Goal rejected" [line 91]
    └─────▶ ActionStatus: set_exception(Exception("Goal rejected"))
            │
            ├─ done = True
            ├─ success = False
            └─ exception = Exception("Goal rejected")

    [spin_once loop detects done=True]
    [Exit and return failed status]


Scenario 2: Action Aborted During Execution
────────────────────────────────────────────

MTCExecutionDevice: [Spinning in while loop]
    │
    ├─ spin_once()
    ├─ spin_once()
    │
ActionServer ──────────────────▶ [Planning failed]
    │                           │
    │                           └─ Abort with error message
    │
    └─────────────▶ Result: status=5 (ABORTED)
                    │
MTCExecutionDevice ◀┘ _result_callback(future)
    │
    ├─ result.status == 5 [line 109]
    ├─ Log error: result.error_message [line 110]
    └─────▶ ActionStatus: set_exception(Exception(error_message))
            │
            └─ done = True, exception set

    [Exit spin loop]
    [Bluesky plan raises exception]


Scenario 3: User Cancellation (Ctrl+C)
───────────────────────────────────────

User ──────▶ [Ctrl+C pressed]
             │
             └─────▶ KeyboardInterrupt raised
                     │
                     └─────▶ ActionStatus: _handle_failure() [line 17]
                             │
                             └─────▶ MTCExecutionDevice: cancel_goal()
                                     │
                                     ├─ cancel_goal_async() [line 123]
                                     └─────▶ ActionServer: [Cancel request]
                                             │
                                             ├─ Stop execution
                                             └─ Return CANCELED
                                             │
                             ◀───────────────┘ Result: status=6
                             │
                             ├─ _result_callback() [line 100]
                             ├─ status == 6 [line 112]
                             └─────▶ ActionStatus: set_exception(
                                     Exception("Canceled"))

    [Cleanup and exit]


Scenario 4: Subprocess Timeout (simple_mtc_bluesky.py)
───────────────────────────────────────────────────────

MTCDevice: set(task_params)
    │
    └─────▶ subprocess.run(cmd, timeout=300) [line 45]
            │
            │ [Time passes...]
            │ [300 seconds elapse]
            │
            └─ subprocess.TimeoutExpired raised [line 54]
               │
               ├─ Print "MTC task timed out" [line 55]
               └─────▶ Status: set_exception(
                       Exception("MTC task timed out")) [line 56]

    [Return failed status to Bluesky]
```

### Diagram 3: Callback Orchestration (ActionMovable)

```
CALLBACK ORCHESTRATION (ophyd_ros.py)
════════════════════════════════════════════════════════════════

Thread: Main Bluesky Thread

ActionMovable.set(value) [line 157]
    │
    ├─ Create ActionStatus [line 160]
    ├─ _send_goal(value) [line 159]
    │  │
    │  └─ send_goal_async() [line 147]
    │     │
    │     └─ Returns _send_goal_future
    │        │
    │        └─ add_done_callback(_goal_response_callback)
    │
    └─ spin_until_future_complete(_finalize_future) [line 161]
       │
       └─ [BLOCKS here until _finalize_future.done()]


─────────────────────────────────────────────────────────
Event Loop Processing (rclpy spin)
─────────────────────────────────────────────────────────

┌─ When _send_goal_future completes ─────────────────────┐
│                                                         │
│  _goal_response_callback(future) [line 121]            │
│     │                                                   │
│     ├─ Extract goal_handle [line 123]                  │
│     │                                                   │
│     └─ if accepted:                                    │
│        │                                                │
│        └─ get_result_async() [line 131]                │
│           │                                             │
│           └─ Returns _get_result_future                │
│              │                                          │
│              └─ add_done_callback(_stop_spin_callback) │
│                                                         │
└─────────────────────────────────────────────────────────┘

┌─ While executing ───────────────────────────────────────┐
│                                                         │
│  feedback_callback(msg) [ABSTRACT - line 101]          │
│     │                                                   │
│     └─ Process feedback (implementation-specific)      │
│        │                                                │
│        └─ [Can update UI, log progress, etc.]          │
│                                                         │
└─────────────────────────────────────────────────────────┘

┌─ When _get_result_future completes ─────────────────────┐
│                                                         │
│  _stop_spin_callback(future) [line 89]                 │
│     │                                                   │
│     ├─ Check future.done() [line 91]                   │
│     │                                                   │
│     ├─ Call get_result_callback(future) [ABSTRACT]     │
│     │  [line 94]                                        │
│     │  │                                                │
│     │  └─ Process result (implementation-specific)     │
│     │                                                   │
│     ├─ _update_bluesky_status(future) [line 96]        │
│     │  │                                                │
│     │  ├─ if exception: status.set_exception()         │
│     │  └─ elif done: status.set_finished()             │
│     │                                                   │
│     └─ _finalize_future.set_result(True) [line 98]     │
│        │                                                │
│        └─ UNBLOCKS spin_until_future_complete()        │
│                                                         │
└─────────────────────────────────────────────────────────┘

─────────────────────────────────────────────────────────
Back to Main Thread
─────────────────────────────────────────────────────────

spin_until_future_complete() returns
    │
    └─ set() returns ActionStatus [line 162]
       │
       └─ [Bluesky plan continues]


CALLBACK ORDERING GUARANTEE
════════════════════════════════════════════════════════════════

The _stop_spin_callback ensures proper ordering:

1. get_result_callback()      ← Process result first
2. _update_bluesky_status()   ← Update status second
3. _finalize_future.set_result() ← Unblock last

This prevents race conditions where:
- Status might be checked before result is processed
- Spin might exit before status is updated
- Callbacks might execute out of order
```

---

## Error Handling

### Error Categories and Handling Strategies

```
ERROR TAXONOMY
════════════════════════════════════════════════════════════════

┌──────────────────────────────────────────────────────────┐
│                  ERROR CATEGORIES                        │
└──────────────────────────────────────────────────────────┘

1. CONFIGURATION ERRORS (Pre-execution)
   ├─ Server not available
   ├─ Invalid robot IP
   ├─ Malformed JSON
   └─ Missing required parameters

2. VALIDATION ERRORS (Goal submission)
   ├─ Goal rejected by server
   ├─ Type mismatch
   └─ Constraint violations

3. EXECUTION ERRORS (During action)
   ├─ Planning failures
   ├─ Collision detected
   ├─ Joint limits exceeded
   └─ Timeout

4. SYSTEM ERRORS (Infrastructure)
   ├─ Network disconnection
   ├─ ROS 2 node crash
   ├─ DDS communication failure
   └─ Process termination

5. USER ERRORS (Interruption)
   ├─ Ctrl+C (KeyboardInterrupt)
   ├─ Cancellation request
   └─ Emergency stop
```

### Error Handling Patterns

```
PATTERN 1: Server Unavailable
════════════════════════════════════════════════════════════════
Location: ophyd_ros.py line 145, mtc_ophyd_device.py line 64

Code:
    self._action_client.wait_for_server(timeout_sec=10.0)

Behavior:
    - Blocks for up to 10 seconds
    - If timeout: raises no exception (returns False)
    - Silent failure → goal send will fail

Issue:
    ⚠️  No explicit error handling if server not found!

Improvement Needed:
    if not self._action_client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("Action server not available")


PATTERN 2: Goal Rejection
════════════════════════════════════════════════════════════════
Location: mtc_ophyd_device.py lines 90-93

Code:
    if not goal_handle.accepted:
        self.get_logger().error("Goal rejected")
        self._bluesky_status.set_exception(Exception("Goal rejected"))
        return

Behavior:
    - Check goal_handle.accepted flag
    - Log error message
    - Set exception on status
    - Early return (no further processing)

Effect:
    - status.done = True
    - status.success = False
    - Bluesky plan raises exception


PATTERN 3: Action Abort/Cancel
════════════════════════════════════════════════════════════════
Location: mtc_ophyd_device.py lines 104-117

Code:
    if result.status == 4:  # SUCCEEDED
        status.set_finished()
    elif result.status == 5:  # ABORTED
        status.set_exception(Exception(result.result.error_message))
    elif result.status == 6:  # CANCELED
        status.set_exception(Exception("Canceled"))
    else:  # UNKNOWN
        status.set_exception(Exception("Unknown status"))

Behavior:
    - Switch on ROS 2 action status code
    - Extract error message from result
    - Set appropriate exception

ROS 2 Action Status Codes:
    0 = UNKNOWN
    1 = ACCEPTED
    2 = EXECUTING
    3 = CANCELING
    4 = SUCCEEDED ← Success case
    5 = ABORTED   ← Server-side error
    6 = CANCELED  ← User/client cancellation


PATTERN 4: Python Exception Handling
════════════════════════════════════════════════════════════════
Location: ophyd_ros.py line 169, mtc_ophyd_device.py line 92

Code (ophyd_ros.py):
    if future.exception():
        self._bluesky_status.set_exception(future.exception())

Code (simple_mtc_bluesky.py lines 54-59):
    except subprocess.TimeoutExpired:
        status.set_exception(Exception("MTC task timed out"))
    except Exception as e:
        status.set_exception(e)

Behavior:
    - Catch Python exceptions at multiple levels
    - Wrap in ActionStatus/Status exception
    - Propagate to Bluesky plan


PATTERN 5: Cancellation via _handle_failure
════════════════════════════════════════════════════════════════
Location: ophyd_ros.py lines 23-25, mtc_ophyd_device.py line 18

Code:
    class ActionStatus(DeviceStatus):
        def _handle_failure(self):
            self.device.cancel_goal()

Trigger:
    - Called by Ophyd when status fails
    - Can be triggered by timeout
    - Invoked on KeyboardInterrupt

Behavior:
    - Attempts graceful cancellation
    - Sends cancel request to server
    - Waits for cancellation confirmation

Note:
    ⚠️  No guarantee server will cancel!
    Server may complete before cancel is processed.
```

### Error Recovery Strategies

```
RECOVERY STRATEGY TABLE
════════════════════════════════════════════════════════════════
│ Error Type          │ Detection Point       │ Recovery Action      │
├─────────────────────┼───────────────────────┼──────────────────────┤
│ Server unavailable  │ wait_for_server()     │ Retry with backoff   │
│ Goal rejected       │ _goal_response_cb()   │ Log and abort        │
│ Planning failed     │ _result_callback()    │ Retry with replanning│
│ Collision detected  │ _result_callback()    │ Abort, operator alert│
│ Timeout             │ subprocess/spin       │ Cancel, retry        │
│ Network disconnect  │ Future exception      │ Reconnect, resume    │
│ Ctrl+C              │ _handle_failure()     │ Cancel, cleanup      │
│ Joint limit         │ _result_callback()    │ Abort, safe config   │
└─────────────────────┴───────────────────────┴──────────────────────┘


BLUESKY INTEGRATION ERROR FLOW
════════════════════════════════════════════════════════════════

When ActionStatus.set_exception(exc) is called:
    │
    ├─ status.done = True
    ├─ status.success = False
    ├─ status.exception = exc
    │
    └─▶ Bluesky RunEngine detects failure
        │
        ├─ Raises exception in plan
        │
        ├─▶ Plan can catch with try/except:
        │   │
        │   try:
        │       yield from bps.abs_set(device, value)
        │   except Exception as e:
        │       # Recovery logic
        │       yield from bps.abs_set(device, fallback_value)
        │
        └─▶ Or let it propagate to RE:
            │
            └─ RE aborts plan execution
               ├─ Emits 'stop' document
               └─ Returns to interactive prompt


DEFENSIVE PROGRAMMING RECOMMENDATIONS
════════════════════════════════════════════════════════════════

1. Always check wait_for_server() return value:
   if not client.wait_for_server(timeout_sec=10):
       raise TimeoutError("Server not available")

2. Validate JSON before sending:
   try:
       json.loads(goal.full_json)
   except json.JSONDecodeError as e:
       raise ValueError(f"Invalid JSON: {e}")

3. Add timeout to all blocking operations:
   spin_until_future_complete(node, future, timeout_sec=300)

4. Log all state transitions:
   logger.info(f"State: {old_state} → {new_state}")

5. Implement retry logic for transient failures:
   for attempt in range(3):
       try:
           result = device.set(value)
           break
       except NetworkError:
           if attempt == 2:
               raise
           time.sleep(2 ** attempt)  # Exponential backoff

6. Clean up resources in finally blocks:
   try:
       RE(plan)
   finally:
       rclpy.shutdown()
```

---

## Design Insights

```
★ Design Insights ───────────────────────────────────────────────
```

### 1. Bridge Pattern Implementation

**Pattern**: Bridge pattern separating abstraction (Bluesky Movable) from implementation (ROS 2 ActionClient).

**Location**: ActionMovable class (ophyd_ros.py lines 28-177)

**How it works**:
- **Abstraction side**: Bluesky expects `Movable.set(value) → Status`
- **Implementation side**: ROS 2 expects action goals, feedback, results
- **Bridge**: ActionMovable translates between these two worlds

**Benefits**:
- Bluesky plans are decoupled from ROS 2 details
- Can swap ROS 2 implementation without changing plans
- Single abstraction works for any ROS 2 action type
- Testable in isolation (mock either side)

**Evidence**:
```python
# Bluesky interface (lines 157-162)
def set(self, value) -> ActionStatus:
    self._send_goal(value)
    self._bluesky_status = ActionStatus(self)
    rclpy.spin_until_future_complete(self, self._finalize_future)
    return self._bluesky_status

# ROS 2 interface (lines 134-148)
def _send_goal(self, goal: Any) -> None:
    goal_msg = self.construct_goal_mesage(goal)
    self._action_client.send_goal_async(goal_msg, ...)
```

This clean separation means Bluesky developers never see ROS 2 futures, callbacks, or action types.

### 2. Template Method Pattern for Extensibility

**Pattern**: Template method pattern with abstract methods defining customization points.

**Location**: ActionMovable abstract methods (lines 73-103)

**Customization points**:
1. `action_type` (property) - Define message type
2. `construct_goal_message()` - Build goal from user input
3. `feedback_callback()` - Process progress updates
4. `get_result_callback()` - Handle completion

**Rationale**:
- Different ROS 2 actions have different message types and semantics
- Framework handles lifecycle, subclass handles domain logic
- Enforces consistent structure across all action wrappers

**Example specialization** (mtc_ophyd_device.py):
```python
# Lines 31-32: Dynamically load action type
from rosidl_runtime_py.utilities import get_action
self.action_type = get_action('mtc_pipeline/MTCExecution')

# Lines 41-54: MTC-specific goal construction
def construct_goal_message(self, json_path_or_string):
    goal = self.action_type.Goal()
    if json_path_or_string.endswith('.json'):
        with open(json_path_or_string, 'r') as f:
            goal.full_json = f.read()
    else:
        goal.full_json = json_path_or_string
    goal.robot_ip = self.robot_ip
    return goal
```

Creating a new action wrapper requires ~50 lines implementing these 4 methods.

### 3. Future Coordination for Async Control Flow

**Pattern**: Coordinated futures with sentinel future for spin control.

**Location**: ophyd_ros.py lines 63-66, 89-98, 161

**Problem being solved**:
- ROS 2 actions are async (send goal, wait for result)
- Bluesky expects synchronous `set()` method (blocks until complete)
- Need to block Bluesky while allowing ROS 2 callbacks to fire

**Solution**:
- Create `_finalize_future` that remains pending (line 66)
- Spin blocked on this future (line 161)
- Only set `_finalize_future.set_result(True)` in final callback (line 98)
- This guarantees all callbacks execute before returning to Bluesky

**Why it's clever**:
- No busy-wait polling required
- Event-driven (CPU efficient)
- Callbacks execute in correct order
- Thread-safe coordination

**Sequence**:
```
_send_goal_future → _goal_response_callback
                  → _get_result_future → _stop_spin_callback
                                       → _finalize_future.set_result()
                                       → spin exits
```

This pattern ensures deterministic callback ordering despite async execution.

### 4. Two-Stage Status Synchronization

**Pattern**: Bridge between ROS 2 action status and Bluesky ActionStatus with explicit update method.

**Location**: ophyd_ros.py lines 164-173

**Why two separate status objects?**
- **ROS 2 status**: Result.status (integer codes: 4=success, 5=abort, etc.)
- **Bluesky status**: ActionStatus (done/success/exception model)

**Synchronization point** (`_update_bluesky_status`):
```python
def _update_bluesky_status(self, future: Future) -> None:
    if future.exception():
        self._bluesky_status.set_exception(future.exception())
    elif future.done():
        self._bluesky_status.set_finished()
```

**Critical timing**: Called in `_stop_spin_callback` BEFORE setting `_finalize_future` (lines 96-98).

**Why this order matters**:
1. Result arrives → `_stop_spin_callback` invoked
2. Process result → `get_result_callback()` (custom logic)
3. Update Bluesky status → `_update_bluesky_status()` (set finished/exception)
4. Unblock spin → `_finalize_future.set_result(True)`

If step 4 happened before step 3, Bluesky might check status before it's updated, seeing stale state.

### 5. Simplicity vs Feature Tradeoff

**Observation**: Two implementations with inverse complexity/capability profiles.

**Native Python (MTCExecutionDevice)**:
- **Complexity**: ~125 lines, 4 callback methods, future management
- **Features**: Real-time feedback, cancellation, type safety
- **Use case**: Production systems requiring monitoring and control

**Subprocess (MTCDevice)**:
- **Complexity**: ~40 lines, simple blocking call
- **Features**: None (black box execution)
- **Use case**: Quick prototypes, batch automation, simple tasks

**Design philosophy**:
> "Make the simple things simple, and the complex things possible."

The codebase provides both:
- Start with subprocess for quick results
- Migrate to native when you need control
- Both implement same `set()` interface (interchangeable)

**Evidence of tradeoff** (simple_mtc_bluesky.py lines 38-61):
```python
# Entire set() implementation:
status = Status()
try:
    cmd = ['ros2', 'run', 'mtc_pipeline', 'mtc_action_client_example',
           json_file_path, robot_ip, '300']
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    if result.returncode == 0:
        status.set_finished()
    else:
        status.set_exception(Exception(f"MTC task failed: {result.stderr}"))
except subprocess.TimeoutExpired:
    status.set_exception(Exception("MTC task timed out"))
return status
```

Compare to native: ~100+ lines for equivalent functionality. The 60-line difference buys you progress visibility and cancellation.

### 6. Impedance Mismatch Between Bluesky and ROS 2

**Observation**: Fundamental differences in execution models require careful bridging.

**Bluesky model**:
- Generator-based plans (yield from)
- Synchronous device operations (blocking `set()`)
- Event document stream (begin/end/event/descriptor)
- Single-threaded execution

**ROS 2 model**:
- Callback-based async operations
- Non-blocking action sends
- Feedback during execution
- Multi-threaded by default

**Integration challenges**:

| Challenge | Solution | Location |
|-----------|----------|----------|
| Async → Sync | Block on future in `set()` | line 161, 74-75 |
| Callbacks in sync context | Spin loop processes callbacks | line 161, 75 |
| Progress updates | Store in instance vars, log | line 79-85 |
| Thread safety | Single thread (spin blocks) | N/A |
| Error propagation | Future → ActionStatus → Exception | line 169-172 |

**Key insight**: The spin loop is the "impedance matching transformer":
- Input: ROS 2 async callbacks firing
- Output: Bluesky synchronous blocking call
- Transformation: Event loop → blocking wait

Without this transformer, Bluesky and ROS 2 couldn't communicate.

### 7. Defensive Design with Multiple Failure Modes

**Pattern**: Multiple error detection layers with different triggering conditions.

**Layers of error handling**:

1. **Pre-execution** (lines 64, 145):
   - `wait_for_server()` timeout
   - JSON parsing errors

2. **Goal submission** (lines 90-93):
   - Goal rejected by server
   - Type validation failures

3. **During execution** (lines 104-117):
   - Action aborted (status 5)
   - Action canceled (status 6)
   - Unknown status codes

4. **Infrastructure** (line 169):
   - Future exceptions (network, serialization)
   - ROS 2 node failures

5. **User interruption** (lines 17-18, 23-25):
   - Ctrl+C handling
   - `_handle_failure()` callback

**Why multiple layers?**
Robotic systems fail in creative ways:
- Network disconnects mid-execution
- Planning succeeds but execution hits obstacle
- User needs emergency stop
- Server crashes without sending result

Each layer catches different failure modes. Missing any layer leaves a blind spot where system hangs or crashes ungracefully.

**Example of multi-layer catch** (lines 87-117 in mtc_ophyd_device.py):
```python
# Layer 1: Goal acceptance
if not goal_handle.accepted:
    self._bluesky_status.set_exception(Exception("Goal rejected"))
    return

# Layer 2: Result status codes
if result.status == 4:
    self._bluesky_status.set_finished()
elif result.status == 5:
    self._bluesky_status.set_exception(Exception(error_message))
elif result.status == 6:
    self._bluesky_status.set_exception(Exception("Canceled"))
else:
    self._bluesky_status.set_exception(Exception("Unknown status"))
```

Each `else` clause handles progressively rarer failure modes.

---

## Usage Examples

### Example 1: Native Python Approach

**File**: `/home/aditya/work/github_ws/erobs/src/bluesky_ros/mtc_bluesky_example.py`

```python
#!/usr/bin/env python3
"""Single Task Execution Example"""

import rclpy
import bluesky.plan_stubs as bps
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice

def single_task_plan(mtc_device, json_file):
    """Execute a single MTC task"""
    print(f"Executing task from: {json_file}")
    yield from bps.abs_set(mtc_device, json_file, wait=True)
    print("Task complete")

def main():
    # Initialize ROS2
    rclpy.init()

    try:
        # Create MTC device
        mtc = MTCExecutionDevice(
            name="mtc_executor",
            robot_ip="10.68.82.41"  # Your robot IP
        )

        # Create Bluesky RunEngine
        RE = RunEngine({})

        # Execute single task
        RE(single_task_plan(mtc, "/path/to/task.json"))

    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

**Output during execution**:
```
Executing task from: /path/to/task.json
[INFO] [mtc_executor]: Waiting for action server...
[INFO] [mtc_executor]: Sending goal...
[INFO] [mtc_executor]: Goal accepted
[INFO] [mtc_executor]: Step 1: MoveTo (25.0%) - Moving to approach
[INFO] [mtc_executor]: Step 2: Grasp (50.0%) - Closing gripper
[INFO] [mtc_executor]: Step 3: Retreat (75.0%) - Moving away
[INFO] [mtc_executor]: Task completed: 3/3 steps
Task complete
```

### Example 2: Multiple Tasks with Error Handling

```python
def multi_task_plan_with_retry(mtc_device, task_list, max_retries=3):
    """Execute multiple tasks with retry logic"""

    for i, json_file in enumerate(task_list):
        print(f"\n=== Task {i+1}/{len(task_list)} ===")

        for attempt in range(max_retries):
            try:
                # Try to execute task
                yield from bps.abs_set(mtc_device, json_file, wait=True)
                print(f"✓ Task {i+1} completed successfully")
                break  # Success, move to next task

            except Exception as e:
                print(f"✗ Attempt {attempt+1} failed: {e}")

                if attempt < max_retries - 1:
                    print(f"Retrying in 5 seconds...")
                    yield from bps.sleep(5)
                else:
                    print(f"✗ Task {i+1} failed after {max_retries} attempts")
                    raise  # Re-raise on final failure

        # Pause between tasks
        yield from bps.sleep(2)

    print(f"\n✓ All {len(task_list)} tasks completed!")

# Usage
tasks = [
    "/path/to/pick_task.json",
    "/path/to/place_task.json",
    "/path/to/inspect_task.json"
]

RE(multi_task_plan_with_retry(mtc, tasks, max_retries=3))
```

### Example 3: Subprocess Approach (Production Script)

**File**: `/home/aditya/work/github_ws/erobs/src/bluesky_ros/simple_mtc_bluesky.py`

```bash
# Command-line usage examples:

# 1. Use default JSON file and env ROBOT_IP
python3 simple_mtc_bluesky.py

# 2. Use specific JSON file
python3 simple_mtc_bluesky.py /path/to/beamline_test.json

# 3. Override robot IP
python3 simple_mtc_bluesky.py --robot-ip 192.168.1.101

# 4. Multiple files with custom IP
python3 simple_mtc_bluesky.py task1.json task2.json --robot-ip 10.0.0.5
```

**Integration in automated workflow**:
```python
#!/usr/bin/env python3
"""Automated beamline data collection workflow"""

from bluesky import RunEngine
import bluesky.plan_stubs as bps
from simple_mtc_bluesky import MTCDevice

def beamline_experiment(detector, robot, sample_positions):
    """
    Automated workflow:
    1. Robot moves sample to position
    2. Detector acquires data
    3. Repeat for all positions
    """

    for i, position in enumerate(sample_positions):
        print(f"\n=== Position {i+1}/{len(sample_positions)} ===")

        # Move robot to position
        task = {
            'json_file': position['robot_task'],
            'robot_ip': '10.69.26.90'
        }
        yield from bps.abs_set(robot, task, wait=True)

        # Acquire data
        yield from bps.trigger_and_read([detector])

        # Return to safe position
        yield from bps.abs_set(robot, {
            'json_file': '/path/to/safe_position.json',
            'robot_ip': '10.69.26.90'
        }, wait=True)

    print("\n✓ Experiment complete!")

# Setup
RE = RunEngine({})
robot = MTCDevice("robot_device")

# Sample positions for experiment
positions = [
    {'robot_task': '/path/to/position1.json'},
    {'robot_task': '/path/to/position2.json'},
    {'robot_task': '/path/to/position3.json'},
]

# Execute
RE(beamline_experiment(detector, robot, positions))
```

### Example 4: Custom Action Wrapper

**How to create a new action wrapper for a different ROS 2 action**:

```python
"""Custom wrapper for Gripper control action"""

from bluesky_ros.ophyd_ros import ActionMovable, ActionStatus
from rclpy.task import Future
from control_msgs.action import GripperCommand  # Your action type

class GripperDevice(ActionMovable):
    """Ophyd device for gripper control"""

    def __init__(self, name="gripper", **kwargs):
        super().__init__(
            node_name=name,
            action_client_name='gripper_controller/gripper_action',
            **kwargs
        )

    @property
    def action_type(self):
        """Define the action type"""
        return GripperCommand

    def construct_goal_mesage(self, position, max_effort=100.0):
        """Construct gripper goal

        Args:
            position: Gripper position (0.0 = closed, 1.0 = open)
            max_effort: Maximum effort in Newtons
        """
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        return goal

    def feedback_callback(self, feedback_msg):
        """Log gripper feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Gripper position: {fb.position:.3f}, "
            f"effort: {fb.effort:.3f}"
        )

    def get_result_callback(self, future: Future):
        """Process gripper result"""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(
                f"Gripper reached position: {result.result.position:.3f}"
            )
        else:
            self.get_logger().error(
                f"Gripper action failed with status {result.status}"
            )

# Usage in Bluesky plan
def pick_and_place_plan(gripper, mtc_device):
    """Combined gripper and MTC task"""

    # Open gripper
    yield from bps.abs_set(gripper, 1.0, wait=True)  # position=1.0 (open)

    # Move to object
    yield from bps.abs_set(mtc_device, "/path/to/approach.json", wait=True)

    # Close gripper
    yield from bps.abs_set(gripper, 0.0, wait=True)  # position=0.0 (closed)

    # Move to place location
    yield from bps.abs_set(mtc_device, "/path/to/place.json", wait=True)

    # Release gripper
    yield from bps.abs_set(gripper, 1.0, wait=True)
```

---

## Summary and Best Practices

### Architecture Summary

This integration provides a production-ready bridge between:
- **Bluesky**: Data acquisition orchestration framework (Python generators)
- **ROS 2**: Real-time robotic control middleware (action-based communication)
- **MoveIt Task Constructor**: High-level manipulation planning (sequential task stages)

The bridge operates at the Ophyd device abstraction layer, wrapping ROS 2 actions as Bluesky `Movable` devices.

### Key Files Summary

| File | Purpose | Lines | Complexity |
|------|---------|-------|------------|
| `ophyd_ros.py` | Abstract base for action wrappers | 177 | High |
| `mtc_ophyd_device.py` | MTC-specific implementation | 125 | Medium |
| `simple_mtc_bluesky.py` | Subprocess-based alternative | 160 | Low |
| `mtc_bluesky_example.py` | Usage example | 53 | Low |

### Best Practices

**For Users (Writing Bluesky Plans)**:
1. Always use `wait=True` in `bps.abs_set()` for robotic moves
2. Wrap long sequences in try/except for error handling
3. Add `bps.sleep()` pauses between heavy operations
4. Use subprocess approach for simple automation
5. Use native Python approach when you need feedback/cancellation

**For Developers (Extending the Framework)**:
1. Inherit from `ActionMovable` for new action types
2. Implement all 4 abstract methods (action_type, construct_goal, feedback, result)
3. Always validate goals before sending
4. Add logging at all state transitions
5. Test cancellation/timeout behavior
6. Document expected goal format

**For System Integrators**:
1. Ensure ROS 2 action server is running before starting Bluesky
2. Set appropriate timeouts (consider task duration)
3. Monitor feedback messages for debugging
4. Implement retry logic for transient failures
5. Use separate ROS 2 domain IDs for isolation
6. Configure network firewalls for DDS multicast

### Common Pitfalls and Solutions

**Pitfall 1**: Forgetting to initialize rclpy
```python
# ✗ Wrong
mtc = MTCExecutionDevice()  # Will fail

# ✓ Correct
rclpy.init()
mtc = MTCExecutionDevice()
```

**Pitfall 2**: Not waiting for server
```python
# ✗ Dangerous (silent failure)
client.wait_for_server(timeout_sec=10.0)

# ✓ Safe (explicit check)
if not client.wait_for_server(timeout_sec=10.0):
    raise RuntimeError("Server not available")
```

**Pitfall 3**: Ignoring ActionStatus completion
```python
# ✗ Wrong (doesn't wait)
status = device.set(value)
print("Done!")  # Executed immediately!

# ✓ Correct (waits for completion)
yield from bps.abs_set(device, value, wait=True)
print("Done!")  # Executed after completion
```

**Pitfall 4**: Not cleaning up ROS 2 resources
```python
# ✗ Wrong (resources leak)
rclpy.init()
RE(plan)

# ✓ Correct (cleanup in finally)
rclpy.init()
try:
    RE(plan)
finally:
    rclpy.shutdown()
```

### Performance Considerations

**Latency breakdown** (typical MTC task):
- Goal construction: ~1-5ms
- Network send: ~5-20ms (local), ~50-200ms (remote)
- Server processing: ~100-5000ms (planning)
- Execution: ~5-60 seconds (motion)
- Result return: ~5-20ms

**Bottlenecks**:
1. **Planning time**: Dominant factor (seconds)
2. **Spin loop overhead**: 100ms latency in `spin_once()` approach
3. **JSON parsing**: ~10-50ms for large task descriptions
4. **Network**: Minimal on local networks, significant on WiFi

**Optimization tips**:
- Use `spin_until_future_complete()` instead of `spin_once()` loop
- Pre-compile JSON strings (don't re-read files)
- Keep ROS 2 nodes on same machine when possible
- Use wired Ethernet for robot communication
- Increase spin timeout for slow networks

### Future Enhancements

**Potential improvements**:
1. Add progress bar visualization using feedback
2. Implement action pre-emption (switch tasks mid-execution)
3. Add result caching for repeated tasks
4. Create Bluesky-aware ROS 2 action server base class
5. Support parallel action execution (multiple robots)
6. Add telemetry export to Bluesky documents
7. Implement automatic retry with exponential backoff
8. Add dry-run mode (validate without executing)

---

## References

**Bluesky Documentation**:
- Bluesky Project: https://blueskyproject.io/
- Ophyd Devices: https://blueskyproject.io/ophyd/
- RunEngine: https://blueskyproject.io/bluesky/

**ROS 2 Documentation**:
- Actions: https://docs.ros.org/en/rolling/Tutorials/Understanding-ROS2-Actions.html
- rclpy: https://docs.ros2.org/latest/api/rclpy/

**MoveIt Task Constructor**:
- MTC Tutorial: https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html

**Related Code**:
- MTC Action Server: `/home/aditya/work/github_ws/erobs/src/mtc_pipeline/`
- Robot Configuration: `/home/aditya/work/github_ws/erobs/src/erobs_planning_scene/`

---

**Document Version**: 1.0
**Last Updated**: 2025-11-18
**Author**: AI Analysis of erobs codebase
