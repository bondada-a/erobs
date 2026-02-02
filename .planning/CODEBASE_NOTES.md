# EROBS Codebase Notes

*Generated during overnight codebase learning session — 2026-02-01*

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    Bluesky RunEngine                             │
│                (experiment orchestration)                        │
└────────────────────────┬────────────────────────────────────────┘
                         │ Ophyd Device (ROS2 ActionClient wrapper)
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│            MTCOrchestratorActionServer (orchestrator.py)         │
│                    1108 lines — Central coordinator              │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │ • JSON task parser                                          ││
│  │ • Task batching (~1.5s saved per batch)                     ││
│  │ • MoveIt lifecycle management (start/stop)                  ││
│  │ • Gripper state tracking                                    ││
│  │ • Pause/Resume functionality                                ││
│  │ • Dispatches to specialized action servers                  ││
│  └─────────────────────────────────────────────────────────────┘│
└────────────────────────┬────────────────────────────────────────┘
                         │ Internal ActionClient calls
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│              Specialized Action Servers (7 types)                │
│  ┌──────────────┬──────────────┬──────────────┬───────────────┐ │
│  │  move_to     │  pick_place  │ end_effector │ tool_exchange │ │
│  │  (36 lines)  │  (36 lines)  │  (33 lines)  │  (35 lines)   │ │
│  └──────┬───────┴──────┬───────┴──────┬───────┴───────┬───────┘ │
│  ┌──────┴───────┬──────┴───────┬──────┴───────────────────────┐ │
│  │   vision     │   pipettor   │     vision_pick_place        │ │
│  │ (143 lines)  │  (37 lines)  │       (63 lines)             │ │
│  └──────────────┴──────────────┴──────────────────────────────┘ │
│         All inherit from BaseActionServer (103 lines)           │
└────────────────────────┬────────────────────────────────────────┘
                         │ Uses stage implementations
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Stage Implementations                         │
│  ┌──────────────────────────────────────────────────────────────┐│
│  │  base_stages.py (393 lines) — MTC utilities, planners       ││
│  │  ├── joints_from_degrees() — Convert deg to rad             ││
│  │  ├── create_*_planner() — OMPL/Cartesian planner factories  ││
│  │  ├── DIRECTION_VECTORS — For relative moves                 ││
│  │  └── Module-level rclcpp.Node for MTC                       ││
│  └──────────────────────────────────────────────────────────────┘│
│  ┌──────────────────────────────────────────────────────────────┐│
│  │  vision_stages.py (1336 lines) — Vision operations          ││
│  │  ├── VisionStages class — Main vision handler               ││
│  │  ├── Detection methods: ArUco, circle, contour              ││
│  │  ├── Multi-position scanning for accuracy                   ││
│  │  ├── Tag pose caching for batch operations                  ││
│  │  └── IK trajectory execution                                ││
│  └──────────────────────────────────────────────────────────────┘│
│  ┌──────────────────────────────────────────────────────────────┐│
│  │  pick_place_stages.py (345 lines) — 9-stage pick/place      ││
│  │  ├── open → approach → pick → close → retreat               ││
│  │  └── approach → place → open → retreat                      ││
│  └──────────────────────────────────────────────────────────────┘│
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│           MoveIt Task Constructor (MTC)                          │
│              Motion planning & execution                         │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                 UR5e Robot + Grippers                            │
│        Hand-E | ePick | Pipettor (swappable)                     │
└─────────────────────────────────────────────────────────────────┘
```

## Key Design Patterns

### 1. Thin Action Server Pattern

Action servers are minimal wrappers (~35 lines) that:
1. Inherit from `BaseActionServer`
2. Implement `initialize_stages()` to create their stage handler
3. Let the base class handle goal lifecycle

```python
class MoveToServer(BaseActionServer):
    def __init__(self):
        super().__init__(
            node_name="move_to_action_server",
            action_name="move_to",
            action_type=MoveToAction
        )
    
    def initialize_stages(self):
        self._stages = MoveToStages()
```

### 2. Module-Level MTC Node

MTC requires `rclcpp.Node` (C++ pybind11), not `rclpy.Node`. The solution:

```python
# In base_stages.py
rclcpp.init()
_mtc_node = rclcpp.Node("beambot", _options)
```

This is safe because:
- Python module caching prevents double-init
- Each action server runs in separate process
- MTC needs C++ node for SharedPtr

### 3. Task Batching in Orchestrator

Consecutive `moveto` and `end_effector` tasks are batched:
- Saves ~1.5s per batched task (MTC task setup overhead)
- Batching happens transparently in orchestrator
- Improves overall execution speed significantly

### 4. Camera-Agnostic Vision (Factory Pattern)

```python
# camera/__init__.py
def get_camera(camera_type: str) -> BaseCamera:
    if camera_type == "zivid":
        return ZividCamera()
    elif camera_type == "azure_kinect":
        return AzureKinectCamera()
    # ...
```

Allows swapping cameras without changing vision logic.

### 5. Detection Retry with Backoff

Vision detection uses configurable retries:
```python
DEFAULT_RETRY_COUNT = 10
DEFAULT_RETRY_DELAY = 0.5  # seconds
```

Handles transient detection failures (lighting, occlusion).

## File Size Summary

| File | Lines | Purpose |
|------|-------|---------|
| `orchestrator.py` | 1108 | Central task coordinator |
| `vision_stages.py` | 1336 | All vision operations |
| `base_stages.py` | 393 | MTC utilities & planners |
| `pick_place_stages.py` | 345 | 9-stage pick/place |
| `vision_pick_place_stages.py` | 332 | Hybrid vision+hardcoded |
| `zivid.py` | 783 | Zivid camera interface |
| `base_action_server.py` | 103 | Action server base class |

## Key Constants

```python
# base_stages.py
VELOCITY_SCALING = 0.2          # 20% max velocity
ACCELERATION_SCALING = 0.2       # 20% max acceleration
DEFAULT_ARM_GROUP = "ur_arm"
DEFAULT_IK_FRAME = "flange"

# vision_stages.py
DEFAULT_RETRY_COUNT = 10
DEFAULT_RETRY_DELAY = 0.5
SAMPLE_OFFSET_X = 0.02           # 20mm offset for grasp
IK_TRAJECTORY_DURATION = 2.0     # seconds
```

## Action Interfaces (beambot_interfaces)

| Action | Purpose |
|--------|---------|
| `MTCExecution` | Orchestrator goal (full JSON task script) |
| `MoveTo` | Joint/pose motion |
| `PickPlace` | 9-stage pick and place |
| `EndEffector` | Gripper open/close/suction |
| `ToolExchange` | Swap grippers at tool changer |
| `Vision` | Vision-guided operations |
| `VisionPickPlace` | Hybrid vision+hardcoded |
| `Pipettor` | Pipetting operations |

## Bluesky Integration (bluesky_ros)

Two Ophyd device implementations:
1. `mtc_ophyd_device.py` — Synchronous (blocking)
2. `mtc_ophyd_device_async.py` — **Recommended** (non-blocking)

Pattern:
```python
# Bluesky side
robot = MTCExecutionDeviceAsync(name="ur5e")
yield from bps.abs_set(robot, "task.json", wait=True)

# Internally:
# 1. Construct MTCExecution.Goal with full_json
# 2. Send via ROS2 ActionClient
# 3. Background thread spins for callbacks
# 4. DeviceStatus signals completion to Bluesky
```

## Container Architecture

```
┌──────────────────┐     ┌──────────────────┐
│      bsui        │     │  erobs-common-img │
│                  │     │                   │
│ • Bluesky        │────▶│ • MoveIt          │
│ • Ophyd device   │ DDS │ • beambot servers │
│ • IPython/BSUI   │     │ • Zivid SDK       │
│                  │     │ • UR driver       │
└──────────────────┘     └───────────────────┘
     ~500MB goal           Current: ~5GB
```

## Current Work Items

From CLAUDE.md, active development:
1. **Smart tagless detection** — Circle/contour detection in progress
2. **Motion planning improvements** — OMPL tuning TODO
3. **Minimal bsui container** — Reduce from 5GB to 500MB
4. **Point cloud obstacle avoidance** — Complete (Octomap)

## Development Tips

### Building
```bash
colcon build --symlink-install --packages-select beambot
source install/setup.bash
```

### Testing Action Servers
```bash
# Check action server is running
ros2 action list

# Send test goal
ros2 action send_goal /beambot_execution beambot_interfaces/action/MTCExecution \
  "{full_json: '{\"tasks\": [{\"task_type\": \"moveto\", \"target\": \"home\"}], \"poses\": {\"home\": [0,-90,90,-90,-90,0]}}'}"
```

### Debugging Vision
```bash
# Capture Zivid frame
ros2 service call /zivid_camera/capture std_srvs/srv/Trigger

# View detection
ros2 run mtc_gui mtc_gui_client  # Use "Detect Contours" button
```

---

*Notes compiled from overnight codebase learning session*
