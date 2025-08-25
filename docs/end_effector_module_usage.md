# End Effector Control Module

The End Effector Control Module provides a unified interface for controlling different types of end effectors in your robotic system. This module supports grippers, vacuum systems, and can be extended for custom end effectors.

## Features

- **Multi-End Effector Support**: Control different types of end effectors (grippers, vacuum systems, etc.)
- **Configurable Parameters**: Customize positions, forces, pressures, and topics
- **Extensible Design**: Easy to add support for new end effector types
- **Error Handling**: Robust error handling and timeout management
- **Action/Service Integration**: Supports both ROS2 actions and services

## Supported End Effectors

### 1. Grippers (hande, gripper)
- **Actions**: `open`, `close`
- **Parameters**: `position`, `force`
- **Protocol**: ROS2 Action (`control_msgs/action/GripperCommand`)

### 2. Vacuum Systems (epick, vacuum)
- **Actions**: `on`, `off`
- **Parameters**: `pressure`
- **Protocol**: ROS2 Service (`std_srvs/srv/SetBool`)

### 3. Custom End Effectors
- **Actions**: User-defined
- **Parameters**: User-defined
- **Protocol**: Extensible framework

## Configuration

### Basic Configuration Structure

```json
{
  "end_effector": {
    "type": "hande",
    "gripper_topic": "/gripper_action",
    "vacuum_topic": "/vacuum_control",
    "gripper_open_position": 0.085,
    "gripper_close_position": 0.0,
    "gripper_force": 100.0
  }
}
```

### Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `type` | string | "hande" | End effector type (hande, epick, gripper, vacuum) |
| `gripper_topic` | string | "/gripper_action" | ROS2 action topic for gripper control |
| `vacuum_topic` | string | "/vacuum_control" | ROS2 service topic for vacuum control |
| `gripper_open_position` | double | 0.085 | Position for gripper open (meters) |
| `gripper_close_position` | double | 0.0 | Position for gripper close (meters) |
| `gripper_force` | double | 100.0 | Default force for gripper actions (N) |

## Usage

### Basic Structure

```json
{
  "action": "end_effector",
  "end_effector_type": "hande",
  "action": "open",
  "position": 0.085,
  "force": 100.0
}
```

### Parameters

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `action` | string | Yes | Must be "end_effector" |
| `end_effector_type` | string | No | End effector type (defaults to config) |
| `action` | string | Yes | Control action (open/close for grippers, on/off for vacuum) |
| `position` | double | No | Target position (for grippers) |
| `force` | double | No | Force to apply (for grippers) |
| `pressure` | double | No | Pressure setting (for vacuum) |
| `params` | object | No | Custom parameters for custom end effectors |

## Examples

### Gripper Control (hande)

```json
{
  "start_gripper": "hande",
  "end_effector": {
    "type": "hande",
    "gripper_topic": "/gripper_action",
    "gripper_open_position": 0.085,
    "gripper_close_position": 0.0,
    "gripper_force": 100.0
  },
  "sequence": [
    {
      "action": "end_effector",
      "end_effector_type": "hande",
      "action": "open"
    },
    {
      "action": "end_effector",
      "end_effector_type": "hande",
      "action": "close",
      "force": 150.0
    }
  ]
}
```

### Vacuum Control (epick)

```json
{
  "start_gripper": "epick",
  "end_effector": {
    "type": "epick",
    "vacuum_topic": "/vacuum_control"
  },
  "sequence": [
    {
      "action": "end_effector",
      "end_effector_type": "epick",
      "action": "on",
      "pressure": 0.8
    },
    {
      "action": "end_effector",
      "end_effector_type": "epick",
      "action": "off"
    }
  ]
}
```

### Complete Pick and Place with End Effector

```json
{
  "start_gripper": "hande",
  "end_effector": {
    "type": "hande",
    "gripper_topic": "/gripper_action"
  },
  "poses": {
    "home": [0.0, -90.0, 0.0, -90.0, 0.0, 0.0],
    "pickup": [45.0, -45.0, 45.0, -45.0, 45.0, 0.0],
    "place": [-45.0, -45.0, 45.0, -45.0, 45.0, 0.0]
  },
  "sequence": [
    {
      "action": "moveto",
      "target_type": "pose",
      "target": "home",
      "planning_type": "joint"
    },
    {
      "action": "end_effector",
      "action": "open"
    },
    {
      "action": "moveto",
      "target_type": "pose",
      "target": "pickup",
      "planning_type": "joint"
    },
    {
      "action": "end_effector",
      "action": "close"
    },
    {
      "action": "moveto",
      "target_type": "pose",
      "target": "place",
      "planning_type": "joint"
    },
    {
      "action": "end_effector",
      "action": "open"
    },
    {
      "action": "moveto",
      "target_type": "pose",
      "target": "home",
      "planning_type": "joint"
    }
  ]
}
```

## Integration with Other Modules

The end effector module works seamlessly with other modules:

### With MoveTo Module
```json
{
  "action": "moveto",
  "target_type": "pose",
  "target": "pickup_pose",
  "planning_type": "joint"
},
{
  "action": "end_effector",
  "action": "close"
}
```

### With Pick and Place Module
The pick and place module can use end effector control for grasping operations.

### With Tool Exchange Module
The tool exchange module can use end effector control during tool attachment/detachment.

## Error Handling

The module includes comprehensive error handling:

- **Service/Action Availability**: Checks if required services/actions are available
- **Timeout Management**: Configurable timeouts for operations
- **Result Validation**: Verifies operation success
- **Fallback Options**: Graceful degradation when possible

## Extending for Custom End Effectors

To add support for a new end effector type:

1. **Add Configuration**: Extend the configuration structure
2. **Implement Control Method**: Add a new control method in `EndEffectorStages`
3. **Update Run Method**: Add handling for the new end effector type
4. **Add Documentation**: Document the new end effector type

### Example Custom End Effector

```cpp
bool EndEffectorStages::controlCustomEndEffector(const std::string& action, const nlohmann::json& params) {
    // Implement custom end effector control
    // Use params for configuration
    return true;
}
```

## Troubleshooting

### Common Issues

1. **Service Not Available**
   - Check if the end effector service/action is running
   - Verify the topic name in configuration
   - Check ROS2 service list: `ros2 service list`

2. **Action Timeout**
   - Increase timeout values in configuration
   - Check end effector hardware status
   - Verify network connectivity

3. **Invalid Actions**
   - Ensure action names match supported actions
   - Check end effector type configuration
   - Verify parameter values

### Debug Commands

```bash
# List available services
ros2 service list

# List available actions
ros2 action list

# Check service info
ros2 service info /vacuum_control

# Check action info
ros2 action info /gripper_action
```

## Best Practices

1. **Configuration Management**: Use consistent configuration across all modules
2. **Error Handling**: Always check return values from end effector operations
3. **Parameter Validation**: Validate parameters before sending to hardware
4. **Timeout Settings**: Set appropriate timeouts for your hardware
5. **Logging**: Use appropriate log levels for debugging

## Testing

Test the end effector module with:

```bash
# Test gripper control
ros2 launch mtc_pipeline orchestrator_launch.launch.py robot_ip:=192.168.56.101 use_fake_hardware:=true poses_file:=end_effector_example.json

# Test vacuum control
ros2 launch mtc_pipeline orchestrator_launch.launch.py robot_ip:=192.168.56.101 use_fake_hardware:=true poses_file:=vacuum_example.json
```

## Future Enhancements

- **Force Feedback**: Real-time force feedback integration
- **Multi-End Effector**: Support for multiple end effectors simultaneously
- **Advanced Gripping**: Complex gripping strategies and force control
- **Safety Features**: Enhanced safety monitoring and emergency stops
- **Calibration**: Automatic end effector calibration procedures

