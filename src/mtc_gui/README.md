# MTC GUI Package

A dedicated package for the MoveIt Task Constructor (MTC) graphical user interface components.

## Overview

This package contains all the GUI-related code for the MTC pipeline, providing a clean separation of concerns and better organization of the codebase.

## Components

### Core GUI Client
- **`mtc_gui_client.py`** - Main GUI application for creating and executing MTC tasks
- **`pose_editor.py`** - Dialog for editing individual robot poses
- **`poses_manager.py`** - Dialog for managing collections of robot poses
- **`test_gui_components.py`** - Test script for GUI components

### Launch Files
- **`mtc_gui_client.launch.py`** - Launch file for starting the GUI client

## Features

- **Visual Task Builder** - Create robot task sequences through a graphical interface
- **Pose Management** - Edit and manage robot poses with a visual editor
- **Real-time Execution** - Monitor task execution with live feedback
- **Configuration Validation** - Validate task configurations before execution
- **JSON Import/Export** - Load and save task configurations

## Dependencies

- `mtc_pipeline` - Core MTC functionality
- `rclpy` - ROS2 Python client library
- `python3-tkinter` - GUI framework

## Usage

### Launch the GUI
```bash
ros2 launch mtc_gui mtc_gui_client.launch.py
```

### Run GUI directly
```bash
ros2 run mtc_gui mtc_gui_client.py
```

### Test components
```bash
ros2 run mtc_gui test_gui_components.py
```

## Architecture

The GUI package is designed to be independent of the core MTC functionality:

1. **Separation of Concerns** - GUI logic is separate from MTC execution logic
2. **Modular Design** - Each component (pose editor, poses manager) is self-contained
3. **Clean Interface** - Communicates with MTC pipeline through well-defined interfaces
4. **Easy Testing** - Components can be tested independently

## Building

```bash
colcon build --packages-select mtc_gui
```

## Integration

This package integrates with the `mtc_pipeline` package by:
- Using the MTC action server for task execution
- Calling MTC executables for task processing
- Sharing configuration formats (JSON)
- Providing a user-friendly interface to MTC capabilities

