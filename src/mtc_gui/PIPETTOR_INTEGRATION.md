# Pipettor Integration for MTC GUI

## Overview
This document describes the integration of pipettor support into the MTC GUI application (`mtc_gui_client.py`).

## Changes Made

### 1. Start Gripper Configuration
**Location:** `create_robot_config_frame()` method (lines 121-126)

**Change:** Added "pipettor" to the start gripper dropdown options
```python
values=["epick", "hande", "pipettor", "none"]
```

### 2. Task Editor Toolbar
**Location:** `create_task_editor_frame()` method (line 153)

**Change:** Added "Add Pipettor" button to the toolbar
```python
ttk.Button(toolbar, text="Add Pipettor", command=lambda: self.add_task_step("pipettor"))
```

### 3. Add Task Step Handler
**Location:** `add_task_step()` method (lines 280-285)

**Change:** Added pipettor task type creation with default values
```python
elif action_type == "pipettor":
    step = {
        "task_type": "pipettor",
        "operation": "SUCK",
        "volume_pct": 0.5
    }
```

### 4. Pipettor Edit Form
**Location:** New method `create_pipettor_edit_form()` (lines 577-727)

**Features:**
- **Operation Selector:** Dropdown with options: SUCK, EXPEL, EJECT_TIP, SET_LED
- **Volume Control:**
  - Text entry for precise value (0.0 - 1.0)
  - Slider for easy adjustment
  - Auto-sync between entry and slider
- **LED Color Control:**
  - Individual RGBA sliders (Red, Green, Blue, Alpha)
  - Real-time color preview canvas
  - Preset color buttons: Red, Green, Blue, Yellow, Purple, White, Off
- **Information Panel:** Describes each operation's purpose
- **Validation:** Ensures volume_pct is a valid float

**UI Components:**
1. Operation dropdown (readonly combobox)
2. Volume percentage entry and slider (0.0-1.0 range)
3. LED color sliders (4 sliders for RGBA)
4. Color preview canvas (updates in real-time)
5. Preset color buttons (7 common colors)
6. Operation info box (documentation)
7. Save button with validation

### 5. Task Tree Display
**Location:** `update_task_tree()` method (lines 768-780)

**Change:** Added pipettor task display logic showing:
- Operation name
- Volume percentage for SUCK/EXPEL operations
- RGB values for SET_LED operations
- Plain operation name for EJECT_TIP

Example display:
```
pipettor    SUCK at 80% volume
pipettor    SET_LED (R:1.0, G:0.0, B:0.0)
pipettor    EJECT_TIP
```

### 6. Edit Dialog Routing
**Location:** `edit_step_dialog()` method (lines 318-349)

**Changes:**
- Added conditional dialog sizing (550x900 for pipettor, 500x600 for others)
- Added routing to `create_pipettor_edit_form()` for pipettor task types

## JSON Format

### Pipettor Task Structure
```json
{
  "task_type": "pipettor",
  "operation": "SUCK|EXPEL|EJECT_TIP|SET_LED",
  "volume_pct": 0.0-1.0,
  "led_color": {
    "r": 0.0-1.0,
    "g": 0.0-1.0,
    "b": 0.0-1.0,
    "a": 0.0-1.0
  }
}
```

### Example Complete Configuration
See `pipettor_gui_test.json` for a complete example with:
- Starting with pipettor gripper
- Moving to home position
- Setting LED to green
- Sucking liquid at 80% volume
- Setting LED to red
- Expelling liquid at 80% volume
- Setting LED to blue
- Ejecting tip

## Operations

### SUCK (Aspirate)
- **Purpose:** Aspirate liquid into pipette
- **Parameters:** `volume_pct` (0.0-1.0)
- **LED:** Uses existing color (optional)

### EXPEL (Dispense)
- **Purpose:** Dispense liquid from pipette
- **Parameters:** `volume_pct` (0.0-1.0)
- **LED:** Uses existing color (optional)

### EJECT_TIP
- **Purpose:** Eject the pipette tip
- **Parameters:** None (`volume_pct` ignored)
- **LED:** Uses existing color (optional)

### SET_LED
- **Purpose:** Change the pipettor LED color
- **Parameters:** `led_color` (RGBA values 0.0-1.0)
- **Note:** `volume_pct` ignored for this operation

## User Interface Features

### Volume Control
- Dual input: text entry for precision, slider for quick adjustment
- Range: 0.0 (empty) to 1.0 (full)
- Auto-sync between entry and slider
- Display shows percentage (0-100%) in task tree

### LED Color Control
- Four sliders for Red, Green, Blue, Alpha channels
- Real-time color preview shows selected color
- Preset buttons for common colors:
  - Red (1.0, 0.0, 0.0)
  - Green (0.0, 1.0, 0.0)
  - Blue (0.0, 0.0, 1.0)
  - Yellow (1.0, 1.0, 0.0)
  - Purple (0.5, 0.0, 0.5)
  - White (1.0, 1.0, 1.0)
  - Off (0.0, 0.0, 0.0)

### Visual Feedback
- Task tree shows operation and key parameters
- Color preview updates in real-time during editing
- Validation errors shown via message box
- Log messages confirm step creation/updates

## Testing

### Quick Test
1. Launch the MTC GUI: `ros2 run mtc_gui mtc_gui_client`
2. Set "Start Gripper" to "pipettor"
3. Click "Add Pipettor" button
4. Double-click the created task to edit
5. Try different operations and LED colors
6. Save configuration and execute

### Load Test Configuration
1. Launch the MTC GUI
2. File → Load JSON
3. Select `pipettor_gui_test.json`
4. Review the loaded pipettor tasks
5. Double-click any task to see the edit interface

## Integration with MTC Orchestrator

The GUI generates JSON that is compatible with the MTC orchestrator's pipettor task handler. The orchestrator expects:
- `task_type`: "pipettor"
- `operation`: One of the four supported operations
- `volume_pct`: Float 0.0-1.0 (used for SUCK/EXPEL)
- `led_color`: RGBA dict (used for SET_LED)

The GUI always includes both `volume_pct` and `led_color` in the JSON for consistency, even if not all fields are used by every operation.

## Code Quality

- Follows existing GUI patterns (similar to other task types)
- Uses same UI framework (tkinter/ttk)
- Consistent naming conventions
- Proper error handling and validation
- Comprehensive documentation in code comments
- No breaking changes to existing functionality

## Files Modified

1. `/home/aditya/work/github_ws/erobs/src/mtc_gui/src/mtc_gui_client.py`
   - Main GUI application with pipettor support

## Files Created

1. `/home/aditya/work/github_ws/erobs/src/mtc_gui/pipettor_gui_test.json`
   - Example configuration for testing pipettor functionality

2. `/home/aditya/work/github_ws/erobs/src/mtc_gui/PIPETTOR_INTEGRATION.md`
   - This documentation file
