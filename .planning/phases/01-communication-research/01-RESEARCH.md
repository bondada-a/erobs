# Phase 1: Communication Research - Research

**Researched:** 2026-01-28
**Domain:** Bluesky/Ophyd → ROS2 Action Client communication for robotic sample handling
**Confidence:** HIGH

<research_summary>
## Summary

This research documents the current communication path from Bluesky experiment orchestration to EROBS robot execution. The system uses a custom Ophyd device that wraps a ROS2 ActionClient, enabling scientists to write `yield from bps.abs_set(robot, "task.json")` while the underlying system handles DDS communication between Docker containers.

The current architecture involves two containers (`bsui` for Bluesky, `erobs-common-img` for ROS/MoveIt) communicating over ROS2 DDS. The key finding is that this architecture was a proof-of-concept for Bluesky integration, but production beamlines have their own Bluesky installation on separate networks—requiring a different bridge mechanism in Phase 4.

**Primary recommendation:** Document the current communication flow thoroughly (this phase's goal), then design a standalone EROBS that exposes an external API (REST/message broker/ROS2 bridge) for any Bluesky installation to connect to.
</research_summary>

<standard_stack>
## Standard Stack

The existing codebase uses these technologies for communication:

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| rclpy | 3.3.x (Humble) | ROS2 Python client | Official ROS2 Python API |
| ophyd | 1.9.x | Bluesky hardware abstraction | Standard at NSLS-II beamlines |
| bluesky | 1.12.x | Experiment orchestration | RunEngine for scientific workflows |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| rclpy.action.ActionClient | Humble | Async goal/result/feedback | ROS2 action communication |
| ophyd.status.DeviceStatus | 1.9.x | Track async operations | Bluesky wait/completion |
| threading.Thread | stdlib | Background ROS spinning | Non-blocking set() methods |
| rosidl_runtime_py | Humble | Dynamic action type loading | `get_action('beambot/MTCExecution')` |

### Alternatives Considered
| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| Ophyd (sync) | ophyd-async | ophyd-async is newer but less mature for non-EPICS devices |
| ROS2 Actions | ROS2 Services | Services are simpler but lack feedback/cancel |
| Docker DDS | Discovery Server | Discovery Server needed for cross-network deployment |

**Current Installation (bsui container):**
```bash
pip3 install bluesky ophyd nslsii ipython
apt-get install ros-humble-rclpy python3-colcon-common-extensions
```
</standard_stack>

<architecture_patterns>
## Architecture Patterns

### Current Communication Architecture
```
┌─────────────────────────────────────────────────────────────────────────┐
│                           BSUI Container                                 │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  Bluesky RunEngine                                               │   │
│  │       │                                                          │   │
│  │       ▼                                                          │   │
│  │  bps.abs_set(robot, "task.json", wait=True)                     │   │
│  │       │                                                          │   │
│  │       ▼                                                          │   │
│  │  MTCExecutionDeviceAsync.set()                                   │   │
│  │       │                                                          │   │
│  │       ├─► Construct MTCExecution.Goal(full_json=...)            │   │
│  │       ├─► self._action_client.send_goal_async()                 │   │
│  │       ├─► Start background thread: rclpy.spin_once() loop       │   │
│  │       └─► Return ActionStatus immediately                        │   │
│  │           (Bluesky waits on this status object)                  │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                    │                                    │
│                                    │ ROS2 DDS (Docker Bridge Network)   │
└────────────────────────────────────│────────────────────────────────────┘
                                     │
┌────────────────────────────────────│────────────────────────────────────┐
│                                    ▼                                    │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  MTCOrchestratorServer (orchestrator.py)                        │   │
│  │       │                                                          │   │
│  │       ├─► ActionServer receives Goal                            │   │
│  │       ├─► Parse JSON: tasks, poses, start_gripper               │   │
│  │       ├─► MoveIt Lifecycle Manager (launch/shutdown MoveIt)     │   │
│  │       ├─► Task batching (consecutive moveto/end_effector)       │   │
│  │       └─► Dispatch to specialized action servers                │   │
│  │           (MoveToAction, PickPlaceAction, VisionAction, etc.)   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                           EROBS Container                               │
└─────────────────────────────────────────────────────────────────────────┘
```

### Pattern 1: Ophyd Device Wrapping ROS2 ActionClient
**What:** The `MTCExecutionDeviceAsync` class inherits from both `rclpy.node.Node` and `bluesky.protocols.Movable`, bridging two ecosystems.
**When to use:** Integrating ROS2 robots with Bluesky experiment orchestration
**Example:**
```python
# Source: src/bluesky_ros/mtc_ophyd_device_async.py
class MTCExecutionDeviceAsync(Node, Movable):
    def __init__(self, name="mtc_executor", robot_ip="192.168.56.101", **kwargs):
        super().__init__(name, **kwargs)
        self.name = name  # For Ophyd

        # Dynamic action type loading (avoids compile-time dependency)
        from rosidl_runtime_py.utilities import get_action
        self.action_type = get_action('beambot/MTCExecution')

        self._action_client = ActionClient(self, self.action_type, 'beambot_execution')
```

### Pattern 2: Background Thread for ROS Spinning
**What:** Ophyd expects `set()` to return immediately with a Status object. ROS2 actions require spinning to process callbacks. Solution: background thread.
**When to use:** Any ROS2 integration that needs non-blocking behavior
**Example:**
```python
# Source: src/bluesky_ros/mtc_ophyd_device_async.py
def set(self, json_path_or_string):
    self._bluesky_status = ActionStatus(self)

    # Send goal async
    self._send_goal_future = self._action_client.send_goal_async(
        goal_msg, feedback_callback=self._feedback_callback
    )
    self._send_goal_future.add_done_callback(self._goal_response_callback)

    # Background thread to process callbacks
    self._spinning = True
    self._spin_thread = Thread(target=self._spin_in_background, daemon=True)
    self._spin_thread.start()

    return self._bluesky_status  # Return immediately!
```

### Pattern 3: DeviceStatus for Bluesky Completion Tracking
**What:** Ophyd's `DeviceStatus` object signals when an async operation completes. Called from ROS2 callbacks.
**When to use:** Reporting completion/failure of ROS2 operations to Bluesky
**Example:**
```python
# Source: src/bluesky_ros/mtc_ophyd_device_async.py
def _result_callback(self, future):
    result = future.result()
    if result.status == 4:  # SUCCEEDED
        self._bluesky_status.set_finished()
    elif result.status == 5:  # ABORTED
        self._bluesky_status.set_exception(Exception(result.result.error_message))
```

### Anti-Patterns to Avoid
- **Blocking in set():** Never call `rclpy.spin_until_future_complete()` in `set()` - Bluesky expects immediate return
- **Shared ROS context:** Each process needs its own `rclpy.init()` / `rclpy.shutdown()` lifecycle
- **Missing thread safety:** ROS callbacks execute in spin thread - use locks for shared state
</architecture_patterns>

<dont_hand_roll>
## Don't Hand-Roll

Problems that have existing solutions:

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Action client callbacks | Manual future polling | `add_done_callback()` | ROS2's future pattern handles edge cases |
| Status notification | Custom event system | `ophyd.status.DeviceStatus` | Integrates with Bluesky's wait/timeout |
| Dynamic action type loading | Hardcoded imports | `rosidl_runtime_py.utilities.get_action` | Allows runtime message type resolution |
| Cross-container DDS | Manual socket code | ROS2 DDS (FastDDS/CycloneDDS) | Handles discovery, QoS, serialization |
| Goal cancellation | Custom flag passing | `goal_handle.cancel_goal_async()` | ROS2 action protocol handles this |

**Key insight:** The Ophyd + rclpy integration is already correctly implemented in the codebase. The challenge isn't the implementation but understanding the full message flow for documentation purposes.
</dont_hand_roll>

<common_pitfalls>
## Common Pitfalls

### Pitfall 1: DDS Multicast Not Working Across Networks
**What goes wrong:** ROS2 nodes in different containers/networks can't discover each other
**Why it happens:** Default DDS uses multicast (239.255.0.1:7400) which doesn't cross network boundaries
**How to avoid:** Use FastDDS Discovery Server or ensure containers share the same Docker network
**Warning signs:** `ros2 topic list` shows nothing, action server "not available"

### Pitfall 2: Blocking the Bluesky RunEngine Thread
**What goes wrong:** `RE(plan)` hangs indefinitely or Bluesky can't process pause/abort
**Why it happens:** `set()` called `rclpy.spin_until_future_complete()` instead of returning Status
**How to avoid:** Always return DeviceStatus immediately, spin ROS in background thread
**Warning signs:** Ctrl+C doesn't pause, Bluesky UI freezes

### Pitfall 3: ROS2 Context Not Initialized
**What goes wrong:** `Node not initialized` or `rclpy not initialized` errors
**Why it happens:** Missing `rclpy.init()` before creating nodes, or called after `rclpy.shutdown()`
**How to avoid:** Call `rclpy.init()` once at process start, `rclpy.shutdown()` only at process end
**Warning signs:** Exceptions on first node creation, "context already shutdown"

### Pitfall 4: Thread Deadlock on Spin Thread Join
**What goes wrong:** Process hangs on shutdown
**Why it happens:** Trying to `join()` spin thread from within a callback that runs on that thread
**How to avoid:** Check `current_thread() != self._spin_thread` before joining
**Warning signs:** `join()` never returns, process hangs on Ctrl+C

### Pitfall 5: Action Type Mismatch Between Containers
**What goes wrong:** Goal not accepted, mysterious serialization errors
**Why it happens:** Different message definitions compiled in each container
**How to avoid:** Build interfaces first (`colcon build --packages-select beambot_interfaces`), ensure same version
**Warning signs:** "goal rejected", ROS2 serialization warnings
</common_pitfalls>

<code_examples>
## Code Examples

Verified patterns from the existing codebase:

### Complete Ophyd Device Integration (Async Pattern)
```python
# Source: src/bluesky_ros/mtc_ophyd_device_async.py
# This is the production pattern - use this, not the sync version

from ophyd.status import DeviceStatus
from rclpy.action import ActionClient
from bluesky.protocols import Movable
from threading import Thread, Lock, current_thread

class ActionStatus(DeviceStatus):
    """Ophyd Status that triggers ROS2 goal cancellation on failure"""
    def _handle_failure(self):
        self.device.cancel_goal()

class MTCExecutionDeviceAsync(Node, Movable):
    def set(self, json_path_or_string):
        """Non-blocking execution - returns immediately"""
        self._bluesky_status = ActionStatus(self)

        # Wait for server with timeout
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self._bluesky_status.set_exception(
                Exception("Action server not available")
            )
            return self._bluesky_status

        # Send goal async with callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

        # Start background spinning
        self._spinning = True
        self._spin_thread = Thread(target=self._spin_in_background, daemon=True)
        self._spin_thread.start()

        return self._bluesky_status  # Bluesky waits on this
```

### Bluesky Plan Using the Device
```python
# Source: src/bluesky_ros/simple_mtc_bluesky.py
import bluesky.plan_stubs as bps
from bluesky import RunEngine

# In a Bluesky plan:
def my_robot_plan(robot_device, json_files):
    for json_file in json_files:
        yield from bps.abs_set(robot_device, json_file, wait=True)
        # Bluesky waits for ActionStatus.set_finished()

# Execution:
RE = RunEngine({})
robot = MTCExecutionDeviceAsync(name="ur5e")
RE(my_robot_plan(robot, ["task1.json", "task2.json"]))
```

### Dynamic Action Type Loading
```python
# Source: src/bluesky_ros/mtc_ophyd_device.py
# This allows runtime message type resolution without compile-time imports

from rosidl_runtime_py.utilities import get_action

# Load action type by name (requires interfaces package to be sourced)
action_type = get_action('beambot/MTCExecution')

# Create client with dynamically loaded type
client = ActionClient(node, action_type, 'beambot_execution')

# Create goal from loaded type
goal = action_type.Goal()
goal.full_json = '{"tasks": [...], "poses": {...}}'
```
</code_examples>

<sota_updates>
## State of the Art (2025-2026)

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| ophyd (sync) | ophyd-async | 2023+ | ophyd-async native for new devices, but Ophyd still standard for ROS2 |
| FastDDS multicast | Discovery Server | 2022+ | Required for cross-network ROS2 communication |
| Manual DDS config | rmw_cyclonedds_cpp | 2024+ | Cyclone DDS more reliable for Docker setups |

**New tools/patterns to consider:**
- **ophyd-async:** Modern async-first device abstraction, but documentation sparse for non-EPICS devices
- **FastDDS Discovery Server:** Enables cross-network ROS2 without multicast - critical for Phase 4
- **DDS Router:** eProsima tool for bridging separate DDS networks - alternative to Discovery Server
- **ROS2 Zenoh Bridge:** Emerging alternative to DDS for WAN communication

**Deprecated/outdated:**
- **Direct multicast for production:** Doesn't work across network boundaries
- **ophyd sync pattern:** The blocking version in `mtc_ophyd_device.py` should be deprecated in favor of async
</sota_updates>

<open_questions>
## Open Questions

Things that need clarification in Phase 4:

1. **Network Architecture at Real Beamlines**
   - What we know: Beamline Bluesky runs on separate network from robot
   - What's unclear: What ports/protocols are allowed through firewall? VPN? DMZ?
   - Recommendation: Document in Phase 4 after IT consultation

2. **Security Requirements**
   - What we know: Phase 4 blocked on beamline IT input
   - What's unclear: Authentication requirements, encryption, audit logging
   - Recommendation: Gather requirements before designing bridge architecture

3. **Latency Requirements**
   - What we know: Some operations need real-time, others can be batched
   - What's unclear: Specific latency budgets for different operation types
   - Recommendation: Profile current DDS latency, establish baseline

4. **Ophyd-async vs Ophyd for ROS2**
   - What we know: Current implementation uses classic Ophyd
   - What's unclear: Whether ophyd-async would simplify the integration
   - Recommendation: Evaluate if async pattern reduces threading complexity
</open_questions>

<sources>
## Sources

### Primary (HIGH confidence)
- **src/bluesky_ros/mtc_ophyd_device_async.py** - Production async implementation
- **src/bluesky_ros/mtc_ophyd_device.py** - Original sync implementation
- **src/beambot/beambot/action_servers/orchestrator.py** - Action server receiving goals
- **docker/bsui/Dockerfile** - Bluesky container setup
- **docker/erobs-common-img/Dockerfile** - EROBS container setup
- **.planning/codebase/ARCHITECTURE.md** - Existing architecture documentation

### Secondary (MEDIUM confidence)
- [ophyd Status objects documentation](https://blueskyproject.io/ophyd/status.html) - DeviceStatus patterns
- [rclpy action client API](https://docs.ros2.org/latest/api/rclpy/api/actions.html) - Official ROS2 Python API
- [ROS2 DDS Discovery Server tutorial](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html) - Cross-network communication

### Tertiary (LOW confidence - needs validation in Phase 4)
- [Docker ROS2 networking guide](https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/) - Container networking patterns
- [DDS Router documentation](https://husarnet.com/blog/ros2-dds-router) - Network bridging options
</sources>

<metadata>
## Metadata

**Research scope:**
- Core technology: Bluesky/Ophyd ↔ ROS2 rclpy integration
- Ecosystem: Docker DDS networking, action client/server patterns
- Patterns: Ophyd device wrapping, async status tracking, background spinning
- Pitfalls: DDS discovery, threading, blocking operations

**Confidence breakdown:**
- Standard stack: HIGH - from existing working codebase
- Architecture: HIGH - traced actual code paths
- Pitfalls: HIGH - discovered from code analysis and web research
- Code examples: HIGH - from production codebase

**Research date:** 2026-01-28
**Valid until:** 2026-03-28 (60 days - architecture stable, Phase 4 will update networking)
</metadata>

---

*Phase: 01-communication-research*
*Research completed: 2026-01-28*
*Ready for planning: yes*
