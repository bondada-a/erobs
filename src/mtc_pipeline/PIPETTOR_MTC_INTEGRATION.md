# Pipettor MTC Integration

## Overview

Pipettor operations are now integrated into the MTC pipeline architecture through a dedicated action server. This provides clean integration with the orchestrator while maintaining descriptive logging for operation tracking.

## Architecture

### Components

1. **PipettorStages** (`src/pipettor_stages.cpp`)
   - Inherits from `BaseStages`
   - Formats operation names for logging
   - Calls pipettor hardware action server
   - Returns success/failure

2. **PipettorActionServer** (`src/pipettor_action_server.cpp`)
   - Action server interface for orchestrator
   - Converts action goals to JSON format
   - Delegates to `PipettorStages::run()`

3. **PipettorAction** (`action/PipettorAction.action`)
   - Unified action interface for all pipettor operations
   - Supports SUCK, EXPEL, EJECT_TIP, SET_LED

### Data Flow

```
Orchestrator → /pipettor_action → PipettorActionServer → PipettorStages → /pipettor_operation (hardware)
```

## Operation Names

Operations show descriptive names in logs:

- **SUCK**: `"SUCK 50%"` (shows volume percentage)
- **EXPEL**: `"EXPEL 80%"` (shows volume percentage)
- **EJECT_TIP**: `"EJECT_TIP"`
- **SET_LED**: `"SET_LED (0,255,0)"` (shows RGB values)

## Usage

### Through Orchestrator

Send tasks via the orchestrator action with JSON:

```json
{
  "start_gripper": "pipettor",
  "poses": {},
  "tasks": [
    {
      "task_type": "pipettor",
      "operation": "SUCK",
      "volume_pct": 0.5
    }
  ]
}
```

### Direct Action Call

Test individual operations:

```bash
ros2 action send_goal /pipettor_action mtc_pipeline/action/PipettorAction \
  "{operation: 'SUCK', volume_pct: 0.5, led_color: {r: 0.0, g: 0.0, b: 0.0, a: 1.0}, poses_json: '{}'}"
```

## Logging and Feedback

### Console Output

```
[pipettor_action_server]: Executing goal
[pipettor_action_server]: Executing pipettor operation: SUCK 50%
[pipettor_action_server]: Sending pipettor operation: SUCK
[pipettor_action_server]: Pipettor operation succeeded
[pipettor_action_server]: Goal completed successfully
```

### Action Feedback

Published to `/pipettor_action/_action/feedback`:
```
status: "Executing pipettor operation: SUCK 50%"
```

### Orchestrator Feedback

Published to `/mtc_execution/_action/feedback`:
```
current_action: "pipettor"
status_message: "Executing: pipettor"
current_step: 1
progress_percentage: 50.0
```

## Visualization Notes

### Current State

Pipettor operations are **non-motion actions** that bypass MTC task planning. They:
- ✅ Show up in console logs with descriptive names
- ✅ Provide action feedback
- ✅ Integrate cleanly with orchestrator workflow
- ❌ Don't appear in RViz MTC task tree (by design)

### Why Not MTC Stages?

Pipettor operations don't involve:
- Robot motion planning
- Inverse kinematics
- Collision checking
- Trajectory generation

Adding them as MTC stages would require:
- Dummy planning (e.g., `ModifyPlanningScene` no-ops)
- Complex stage inheritance
- Overhead without benefit

### Future Visualization Options

If RViz visualization is needed:

**Option 1: Custom Markers**
Publish visualization markers from pipettor_action_server:
```cpp
visualization_msgs::msg::Marker marker;
marker.header.frame_id = "tool0";
marker.type = Marker::TEXT_VIEW_FACING;
marker.text = "SUCK 50%";
marker_pub_->publish(marker);
```

**Option 2: Status Panel**
Create RViz plugin to display:
- Current operation
- Volume percentage
- LED color
- Operation progress

**Option 3: Timeline View**
Publish to `/diagnostics` or custom topic for timeline display

## Testing

### 1. Launch System

Terminal 1 - Orchestrator:
```bash
ros2 launch mtc_pipeline modular_action_servers.launch.py robot_ip:=192.168.1.10
```

### 2. Verify Action Server

```bash
ros2 action list | grep pipettor
# Should show: /pipettor_action
```

### 3. Test Direct Action

```bash
ros2 action send_goal /pipettor_action mtc_pipeline/action/PipettorAction \
  "{operation: 'SUCK', volume_pct: 0.5, led_color: {r: 0.0, g: 0.0, b: 0.0, a: 1.0}, poses_json: '{}'}"
```

### 4. Test Through Orchestrator

```bash
ros2 run mtc_pipeline mtc_action_client_example \
  src/mtc_pipeline/test_tasks/pipettor_test.json \
  192.168.1.10
```

### 5. Monitor Feedback

Terminal 3:
```bash
# Pipettor action feedback
ros2 topic echo /pipettor_action/_action/feedback

# Orchestrator feedback
ros2 topic echo /mtc_execution/_action/feedback
```

## Files Modified

### New Files
- `include/mtc_pipeline/pipettor_stages.hpp`
- `src/pipettor_stages.cpp`
- `src/pipettor_action_server.cpp`
- `action/PipettorAction.action`

### Modified Files
- `CMakeLists.txt` - Added build targets
- `include/mtc_pipeline/mtc_orchestrator_action_server.hpp` - Updated action type
- `src/mtc_orchestrator_action_server.cpp` - Updated action client
- `launch/modular_action_servers.launch.py` - Added pipettor_action_server node

## Troubleshooting

### "Action server not available"
- Ensure orchestrator is running
- Check `ros2 action list` includes `/pipettor_action`

### "Pipettor operation failed"
- Verify hardware driver is running
- Check `/pipettor_operation` action server exists
- Ensure `/tmp/ttyUR` serial port exists

### "No feedback received"
- Monitor topic: `ros2 topic hz /pipettor_action/_action/feedback`
- Check action server logs for errors

## Performance

- **Latency**: ~10-50ms (action server overhead)
- **Timeout**: 60 seconds per operation
- **Hardware**: Depends on Arduino execution time

## Future Enhancements

1. **Progress Feedback**: Real-time plunger/tip position
2. **RViz Visualization**: Custom markers or status panel
3. **Error Recovery**: Retry logic for transient failures
4. **Batch Operations**: Multi-step pipetting sequences
5. **Calibration**: Volume calibration interface
