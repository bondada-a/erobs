# Tool Voltage Management

## Overview
The orchestrator sets tool voltage via TCP socket (port 30002) before launching MoveIt configurations, ensuring grippers receive proper power during hardware initialization.

## Voltage Mappings

| Gripper  | Voltage | Reason                    |
|----------|---------|---------------------------|
| none     | 0V      | No gripper attached       |
| epick    | 24V     | Vacuum gripper            |
| hande    | 24V     | Modbus communication      |
| pipettor | 24V     | Tool power                |

## Implementation

**Location:** `mtc_orchestrator_action_server.cpp`

**Method:** Direct socket connection to robot port 30002 (Secondary Client Interface)

**Command:** `set_tool_voltage(N)\n`

**Timing:** Voltage set before MoveIt launch, with 200ms delay for robot to process

## Execution Flow

```
Tool Exchange (none → hande)
├─ 1. Kill MoveIt stack (none config)
├─ 2. Set voltage to 24V via socket ✓
├─ 3. Wait 200ms
├─ 4. Launch HandE MoveIt config
│   └─ HandE hardware interface initializes with power ✓
└─ 5. Tool exchange completes
```

## Testing

```bash
# Verify voltage after launch
ros2 topic echo /io_and_status_controller/tool_data --once | grep tool_output_voltage

# Manual voltage setting
printf "set_tool_voltage(24)\n" | nc -q 0 192.168.56.101 30002
```

## Troubleshooting

**Socket connection fails:**
- Robot must be in Remote Control mode
- Check network connectivity on port 30002

**Voltage not updating:**
- URSim may not reflect voltage changes in topic (simulation limitation)
- Verify on teach pendant: Installation → General → Tool I/O

**HandE Modbus still fails:**
- See [docs/troubleshooting/HANDE_MODBUS_FAILURE_ANALYSIS.md](docs/troubleshooting/HANDE_MODBUS_FAILURE_ANALYSIS.md) for detailed diagnostics

## Implementation Details

**Files Modified:**
- `mtc_pipeline/include/mtc_pipeline/mtc_orchestrator_action_server.hpp`
- `mtc_pipeline/src/mtc_orchestrator_action_server.cpp`

**Error Handling:** Non-blocking - voltage setting failure logs error but doesn't abort tool exchange
