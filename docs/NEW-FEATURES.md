# New Features - Ranked by Immediate Impact
*Date: 2026-03-06 | Branch: humble-experimental*

Features that would show positive impact right away.

---

## HIGH Impact (Immediate Value)

### 1. MCP Execute Task Tool
**Impact**: Enables Claude to run full robot sequences through MCP
**Effort**: 4-8 hours
**Why now**: Without this, Claude can see but not act. This is the #1 gap.

Add an `execute_task` tool to `erobs_mcp_server.py` that:
- Accepts a task JSON string or a simplified command format
- Creates a ROS2 action client to `beambot_execution`
- Sends the goal, streams feedback, returns result
- Supports timeout and cancellation

```python
@mcp.tool()
async def execute_task(task_json: str, timeout: float = 300.0) -> str:
    """Execute a robot task sequence..."""
```

### 2. MCP Robot State Tool
**Impact**: Lets Claude know what the robot is doing before taking action
**Effort**: 2-4 hours
**Why now**: Claude currently operates blind about robot state.

Add `get_robot_state` tool returning:
- Joint positions (radians and degrees)
- Current gripper (from `/beambot/current_gripper`)
- Execution state (from `/beambot/execution_state`)
- Whether robot is in motion
- Current end-effector position (from TF)

### 3. MCP System Health Tool
**Impact**: Lets Claude diagnose issues before attempting operations
**Effort**: 4-8 hours
**Why now**: When things go wrong, Claude needs diagnostics.

Check and report:
- UR driver connection status
- Zivid camera availability
- Controller states (active/inactive)
- MoveIt availability
- ROS2 node list / topic connectivity

### 4. Quick-Action MCP Tools
**Impact**: Reduces Claude's cognitive load for common operations
**Effort**: 4-8 hours
**Why now**: These are the 80/20 — most common operations made trivial.

```python
@mcp.tool()
async def move_to_pose(pose_name: str) -> str:
    """Move robot to a named joint pose."""

@mcp.tool()
async def move_relative(direction: str, distance: float) -> str:
    """Move robot relative to current position."""

@mcp.tool()
async def gripper_action(action: str) -> str:
    """Open or close the current gripper."""

@mcp.tool()
async def pick_and_place(pick_pose: str, place_pose: str) -> str:
    """Execute a pick-and-place sequence."""
```

These wrap the JSON task format into simple function calls. Claude doesn't need to construct task JSON for basic operations.

### 5. Sample Location Registry
**Impact**: Named sample locations persist across sessions
**Effort**: 4-8 hours
**Why now**: Currently, poses are embedded in task JSONs. A persistent registry enables "go to sample A1" without JSON construction.

Add a YAML-based sample/location registry:
```yaml
# beambot/config/locations.yaml
locations:
  hotplate_1:
    type: joint
    values: [0, -90, 90, -90, -90, 0]
    description: "Hotplate position 1"
  sample_rack_A1:
    type: joint
    values: [30, -70, 80, -90, -90, 30]
    description: "Sample rack row A, column 1"
```

MCP tool: `list_locations()`, `get_location(name)`, `save_location(name)`

---

## MEDIUM Impact

### 6. Vision Scan + Cache Before Pick
**Impact**: Eliminates per-pick detection delay
**Effort**: Already exists (vision_scan), needs MCP exposure
**Why now**: The scan_all_tags capability exists but isn't accessible from MCP.

Add MCP tool:
```python
@mcp.tool()
async def scan_workspace() -> str:
    """Scan workspace from multiple positions, cache all detected objects."""
```

### 7. Teach Mode
**Impact**: Scientists can teach new positions by jogging the robot
**Effort**: 8-16 hours
**Why now**: Currently all positions must be manually measured or copied from teach pendant.

Add MCP tool:
```python
@mcp.tool()
async def teach_current_pose(name: str, description: str = "") -> str:
    """Save the robot's current position as a named pose."""
```

This reads current joint state, saves to the locations registry, and makes it immediately available for task execution.

### 8. Experiment Plan Builder
**Impact**: Claude can compose multi-step experiments from high-level descriptions
**Effort**: 8-16 hours
**Why now**: Building task JSONs manually is error-prone.

```python
@mcp.tool()
async def build_experiment(steps: str) -> str:
    """Build a task JSON from a high-level experiment description.

    Steps format: semicolon-separated actions like:
    'scan workspace; pick sample 5; move to hotplate; place; pick sample 3; move to spincoater; place'
    """
```

### 9. Automated Recovery
**Impact**: System self-heals from common failures
**Effort**: 8-16 hours
**Why now**: Currently, failures require manual intervention.

Common recovery patterns:
- Controller stopped → auto-restart via dashboard
- Planning failed → retry with different planner
- Vision detection failed → re-scan from different angle
- Gripper didn't close → retry grasp with offset

Implement as a recovery layer in the orchestrator that retries failed steps with configurable strategies.

### 10. Execution History & Replay
**Impact**: Enables debugging, experiment reproducibility
**Effort**: 8-16 hours

Log every task execution with:
- Timestamp, task JSON, result, duration
- Joint positions at each step
- Vision detection results
- Errors and recovery actions

MCP tools: `get_execution_history()`, `replay_task(execution_id)`

---

## LOW Impact (Future Enhancements)

### 11. Live Camera Feed as MCP Resource
**Impact**: Claude can view camera in real-time
**Effort**: Medium

Stream Zivid images as an MCP resource that Claude can periodically check.

### 12. Collision Object Editor via MCP
**Impact**: Dynamically add/remove obstacles from planning scene
**Effort**: Medium

```python
@mcp.tool()
async def add_obstacle(name: str, type: str, pose: list, size: list) -> str:
    """Add a collision object to the planning scene."""

@mcp.tool()
async def remove_obstacle(name: str) -> str:
    """Remove a collision object from the planning scene."""
```

### 13. Multi-Robot Support
**Impact**: Control multiple robots from single Claude session
**Effort**: Large

Namespace all topics/services/actions per robot. MCP tools accept robot_id parameter.

### 14. Simulation Mode via MCP
**Impact**: Test experiment plans without moving real robot
**Effort**: Medium

```python
@mcp.tool()
async def set_simulation_mode(enabled: bool) -> str:
    """Enable/disable simulation mode (no real robot motion)."""
```

### 15. Experiment Templates
**Impact**: Reusable experiment patterns
**Effort**: Medium

Library of common experiment patterns:
- `sample_transfer(from, to)` - Pick from A, place at B
- `liquid_dispense(vial, target, volume)` - Tool change, aspirate, dispense
- `full_cycle(sample_id)` - Complete sample processing cycle
