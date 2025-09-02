# MTC Action Client GUI

This GUI application provides a user-friendly interface for the MoveIt Task Constructor (MTC) action client, replacing the need to manually edit JSON files.

## Features

- **Robot Configuration**: Set robot IP address and initial gripper configuration
- **Task Sequence Editor**: Create and edit robot task sequences visually
- **Pose Management**: Add, edit, and manage robot poses with a visual editor
- **Real-time Execution**: Execute tasks and monitor progress
- **JSON Import/Export**: Load and save configurations to/from JSON files

## Supported Task Types

### 1. MoveTo
- Move robot to a specific pose
- Configurable planning type (joint or cartesian)
- Arm group selection

### 2. Pick and Place
- Complete pick and place operations
- Configurable gripper type
- Approach and target poses

### 3. Tool Exchange
- Load or dock end effectors
- Dock number configuration
- Approach pose specification

### 4. End Effector Control
- Control gripper actions (open/close, vacuum on/off)
- Support for different end effector types

## Installation

1. Build the package:
```bash
colcon build --packages-select mtc_pipeline
source install/setup.bash
```

2. The GUI client will be installed as `mtc_gui_client.py` in the package's lib directory.

## Usage

### Running the GUI

```bash
# Method 1: Direct execution
ros2 run mtc_pipeline mtc_gui_client.py

# Method 2: Using launch file
ros2 launch mtc_pipeline mtc_gui_client.launch.py

# Method 3: With custom robot IP
ros2 launch mtc_pipeline mtc_gui_client.launch.py robot_ip:=192.168.1.102
```

### Basic Workflow

1. **Configure Robot Settings**
   - Set the robot IP address
   - Choose the initial gripper configuration
   - Test the connection to the action server

2. **Manage Poses**
   - Click "Manage Poses" to open the pose editor
   - Add new poses or edit existing ones
   - Use preset poses (Home, Pick, Place, etc.)
   - Import poses from existing JSON files

3. **Create Task Sequence**
   - Use the toolbar buttons to add different task types
   - Double-click on steps to edit their parameters
   - Arrange steps in the desired execution order
   - Remove unwanted steps

4. **Execute Tasks**
   - Click "Execute Task" to start execution
   - Monitor progress in the status log
   - Use "Stop Execution" to cancel if needed

### Pose Editor

The pose editor provides:
- Visual joint value input (6 DOF)
- Increment/decrement buttons for fine-tuning
- Preset pose templates
- Validation of joint limits and values

### Configuration Validation

Before execution, the GUI validates:
- All referenced poses exist in the configuration
- Task sequence is properly formatted
- Required parameters are set

## File Operations

### Loading JSON
- Use File → Load JSON to import existing configurations
- The GUI will parse and display the loaded configuration
- Poses and task sequences are loaded into the editor

### Saving JSON
- Use File → Save JSON to export the current configuration
- All GUI settings are included in the exported file
- Compatible with the existing MTC action server

## Troubleshooting

### Common Issues

1. **Action Server Not Available**
   - Ensure the MTC action server is running
   - Check that the robot and MoveIt stack are properly configured
   - Verify network connectivity to the robot

2. **Missing Poses**
   - Use the pose manager to add required poses
   - Check that pose names in task steps match defined poses
   - Import poses from existing JSON files if available

3. **Task Execution Fails**
   - Check the status log for error messages
   - Verify robot configuration and poses
   - Ensure the robot is in a safe state for execution

### Debug Information

- All operations are logged in the status panel
- Connection tests provide feedback on server availability
- Configuration validation shows specific error messages

## Integration

The GUI client integrates with:
- ROS2 action system
- MTC action server (`mtc_execution`)
- Existing JSON configuration format
- MoveIt Task Constructor framework

## Development

### Adding New Task Types

1. Extend the `add_task_step` method in `MTCGUIClient`
2. Create corresponding edit form methods
3. Update the task tree display logic
4. Add validation rules if needed

### Customizing the Interface

The GUI is built with tkinter and can be customized:
- Modify colors and themes
- Add new menu items
- Extend the toolbar with additional buttons
- Customize the layout and sizing

## Dependencies

- Python 3.8+
- tkinter (usually included with Python)
- rclpy (ROS2 Python client library)
- mtc_pipeline package

## License

This GUI client is part of the mtc_pipeline package and follows the same license terms.
