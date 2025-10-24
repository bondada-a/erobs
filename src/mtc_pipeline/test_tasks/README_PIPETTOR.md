# Pipettor Test Tasks

Test JSON files for pipettor integration with MTC orchestrator.

## Available Test Files

### 1. `pipettor_suck_only.json` - Quick Test
Simple single operation test - performs only SUCK operation.

**Usage:**
```bash
ros2 run mtc_pipeline mtc_action_client_example \
  src/mtc_pipeline/test_tasks/pipettor_suck_only.json \
  192.168.1.10
```

### 2. `pipettor_test.json` - Basic Operations
Tests the core pipettor operations sequence:
1. SUCK (50% volume)
2. EXPEL (50% volume)
3. EJECT_TIP

**Usage:**
```bash
ros2 run mtc_pipeline mtc_action_client_example \
  src/mtc_pipeline/test_tasks/pipettor_test.json \
  192.168.1.10
```

### 3. `pipettor_with_led_test.json` - Full Operations with LED
Complete test including LED control:
1. SET_LED to green
2. SUCK (80% volume)
3. SET_LED to red
4. EXPEL (80% volume)
5. SET_LED to blue
6. EJECT_TIP

**Usage:**
```bash
ros2 run mtc_pipeline mtc_action_client_example \
  src/mtc_pipeline/test_tasks/pipettor_with_led_test.json \
  192.168.1.10
```

## Prerequisites

Before running any test:

1. **Launch Orchestrator** (Terminal 1):
   ```bash
   cd /home/aditya/work/github_ws/erobs
   source install/setup.bash
   ros2 launch mtc_pipeline modular_action_servers.launch.py robot_ip:=192.168.1.10
   ```

2. **Wait for orchestrator to be ready** (you should see):
   ```
   [INFO] [mtc_orchestrator_action_server]: MTC Orchestrator Action Server started
   ```

3. **Run test** (Terminal 2):
   ```bash
   cd /home/aditya/work/github_ws/erobs
   source install/setup.bash
   ros2 run mtc_pipeline mtc_action_client_example <json_file> 192.168.1.10
   ```

## What Happens

When you send a pipettor task:

1. **Orchestrator receives task** with `"gripper": "pipettor"`
2. **Orchestrator launches MoveIt** using `ur_zivid_pipettor_moveit_config`
3. **MoveIt starts tool_communication** which creates `/tmp/ttyUR`
4. **pipette_driver_node starts** (after 3 second delay) and connects to `/tmp/ttyUR`
5. **Orchestrator executes each pipettor step** by calling the `/pipettor_operation` action
6. **You get feedback** showing progress for each operation

## Pipettor Operations

- **SUCK**: Aspirate liquid (volume_pct: 0.0-1.0)
- **EXPEL**: Dispense liquid (volume_pct: 0.0-1.0)
- **EJECT_TIP**: Eject the current pipette tip
- **SET_LED**: Change LED color (requires led_color field)

## LED Colors

RGB values from 0.0 to 1.0:
- Green: `{"r": 0.0, "g": 1.0, "b": 0.0, "a": 1.0}`
- Red: `{"r": 1.0, "g": 0.0, "b": 0.0, "a": 1.0}`
- Blue: `{"r": 0.0, "g": 0.0, "b": 1.0, "a": 1.0}`
- Yellow: `{"r": 1.0, "g": 1.0, "b": 0.0, "a": 1.0}`
- Purple: `{"r": 1.0, "g": 0.0, "b": 1.0, "a": 1.0}`
- Cyan: `{"r": 0.0, "g": 1.0, "b": 1.0, "a": 1.0}`
- White: `{"r": 1.0, "g": 1.0, "b": 1.0, "a": 1.0}`

## Cancellation

Press **Ctrl+C** during execution to cancel the current task.

## Troubleshooting

### Error: "Action server not available"
- Make sure orchestrator is running
- Wait 10 seconds after launching orchestrator

### Error: "could not open port /tmp/ttyUR"
- MoveIt hasn't started yet (orchestrator starts it dynamically)
- Wait for task execution to begin
- Check that tool_communication is enabled in robot_bringup.launch.py

### Error: "Task script missing required start_gripper"
- Make sure JSON has `"start_gripper": "pipettor"` (not "gripper")
- Make sure JSON has `"tasks"` array (not "steps")
- Include `"poses": {}` field (can be empty)

### Error: "Goal was rejected by server"
- Check that JSON format is correct
- Make sure start_gripper is set to "pipettor"
- Check orchestrator logs for details
