# UR Tool Voltage Management Implementation

## Overview

This implementation handles automatic tool voltage management for UR robot tool exchanges. The system ensures proper voltage settings for different end-effectors to prevent gripper failures (like Hand-E's "Failed to write registers (Modbus failure)" error).

## Voltage Requirements by Gripper

| Gripper    | Required Voltage | Notes                              |
|------------|------------------|-------------------------------------|
| none       | 0V               | Standalone mode, no tool attached  |
| hande      | 24V              | Hand-E gripper requires 24V        |
| epick      | 24V              | EPick vacuum gripper               |
| pipettor   | 24V              | Pipettor tool                      |

## Implementation Architecture

### Location
All voltage management logic is in `mtc_orchestrator_action_server.cpp`:
- `set_tool_voltage(int voltage)` - Sends URScript commands to set voltage
- `restart_ur_program()` - Restarts the external control program after voltage changes
- `initialize_moveit_stack()` - Integrates voltage setting into MoveIt startup
- `handle_tool_exchange()` - Special handling for tool exchange sequences

### Key Functions

#### `set_tool_voltage(int voltage)`
```cpp
bool MTCOrchestratorActionServer::set_tool_voltage(int voltage)
```
- Publishes URScript command to `/urscript_interface/script_command` topic
- Waits up to 10 seconds for topic to be available
- Sends `set_tool_voltage(N)` command
- Returns true on success

#### `restart_ur_program()`
```cpp
bool MTCOrchestratorActionServer::restart_ur_program()
```
- Calls `/dashboard_client/play` service to restart the UR program
- Required after sending URScript commands (they stop the running program)
- Waits 2 seconds for program to fully start

## Execution Sequences

### Standard Gripper Launch
When launching a gripper configuration:
1. Launch MoveIt config for gripper
2. Wait for planning service (30s timeout)
3. Wait for robot hardware (5s)
4. Restart UR program
5. **Set voltage for gripper type**
6. **Restart UR program again** (voltage command stops it)
7. Mark gripper as active

### Tool Exchange - Dock Operation
```json
{"task_type": "tool_exchange", "operation": "dock", "gripper": "hande"}
```
1. Execute tool exchange motion (MTC pipeline)
2. Switch to "none" gripper config → Sets voltage to 0V

### Tool Exchange - Load Operation
```json
{"task_type": "tool_exchange", "operation": "load", "gripper": "hande"}
```
1. **Set voltage to 0V** (safety - done before motion)
2. **Restart UR program**
3. Execute tool exchange motion (MTC pipeline)
4. Switch to requested gripper config → Sets voltage to 24V

This sequence ensures voltage is OFF during the physical tool exchange motion, preventing electrical issues.

## Important Constraints

1. **MoveIt must be running before voltage commands**: The `/urscript_interface/script_command` topic doesn't exist without MoveIt launched

2. **URScript commands stop the program**: Every `set_tool_voltage()` call requires a subsequent `restart_ur_program()` call

3. **Safety first for load operations**: Voltage-sensitive tools (hande, epick, pipettor) get 0V set BEFORE the tool exchange motion executes

4. **No voltage commands during motion**: Voltage changes only happen at MoveIt startup or before tool exchange motions begin

## Testing

### Manual Voltage Testing
```bash
# Check that urscript_interface is available
ros2 topic list | grep urscript

# Set voltage to 0V
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data: "set_tool_voltage(0)"}'

# Restart program
ros2 service call /dashboard_client/play std_srvs/srv/Trigger

# Set voltage to 24V
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data: "set_tool_voltage(24)"}'

# Restart program again
ros2 service call /dashboard_client/play std_srvs/srv/Trigger
```

### Expected Log Output
When loading Hand-E after tool exchange:
```
[mtc_orchestrator_action_server]: Setting voltage to 0V before loading hande
[mtc_orchestrator_action_server]: Voltage command sent: set_tool_voltage(0)
[mtc_orchestrator_action_server]: Restarting UR external control program
[mtc_orchestrator_action_server]: UR program restarted successfully
[mtc_orchestrator_action_server]: Starting MoveIt configuration for gripper: hande
[mtc_orchestrator_action_server]: MoveIt fully initialized and ready for planning
[mtc_orchestrator_action_server]: Setting tool voltage to 24V
[mtc_orchestrator_action_server]: Voltage command sent: set_tool_voltage(24)
[mtc_orchestrator_action_server]: UR program restarted successfully
```

## Troubleshooting

### "No subscribers to urscript_interface"
- **Cause**: MoveIt config not fully launched
- **Solution**: The code waits up to 10s for the topic; if warning appears, it continues anyway
- **Check**: `ros2 topic info /urscript_interface/script_command`

### "Dashboard play service not available"
- **Cause**: ur_robot_driver dashboard client not running
- **Solution**: Ensure robot_bringup.launch.py includes dashboard client
- **Check**: `ros2 service list | grep dashboard`

### Hand-E still fails with "Modbus failure"
- **Possible causes**:
  1. Voltage set too late (after gripper driver starts)
  2. UR program not fully restarted after voltage change
  3. Hand-E not physically connected properly
- **Debug steps**:
  1. Check logs for "Setting tool voltage to 24V" message
  2. Verify voltage on teach pendant: Installation → General → Tool I/O
  3. Manually test voltage commands (see "Manual Voltage Testing" above)

### Robot doesn't move after voltage command
- **Cause**: UR program stopped but not restarted
- **Solution**: Call `/dashboard_client/play` service
- **Check**: Teach pendant should show "External Control" program running

## Files Modified

- `mtc_pipeline/include/mtc_pipeline/mtc_orchestrator_action_server.hpp`
  - Added `set_tool_voltage()` and `restart_ur_program()` declarations
  - Added `std_msgs/msg/String` include

- `mtc_pipeline/src/mtc_orchestrator_action_server.cpp`
  - Implemented voltage management functions
  - Integrated voltage setting into `initialize_moveit_stack()`
  - Added 0V pre-set in `handle_tool_exchange()` for load operations

- `mtc_pipeline/package.xml`
  - Added `<depend>std_msgs</depend>`

- `mtc_pipeline/CMakeLists.txt`
  - Added `find_package(std_msgs REQUIRED)`
  - Added `std_msgs` to `ORCHESTRATOR_DEPS`

## Future Improvements

1. **Configurable voltage map**: Move gripper voltage requirements to a parameter file
2. **Voltage verification**: Query actual tool voltage after setting to confirm
3. **Async program restart**: Use async service calls to avoid blocking
4. **Retry logic**: Add retries for voltage setting if initial attempt fails
