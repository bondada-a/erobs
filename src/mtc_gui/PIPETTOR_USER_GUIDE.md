# Pipettor User Guide for MTC GUI

## Quick Start

### 1. Setting Pipettor as Start Gripper
At the top of the GUI in the "Robot Configuration" section:
1. Locate the "Start Gripper" dropdown
2. Select "pipettor" from the options (epick, hande, pipettor, none)

### 2. Adding Pipettor Tasks
In the "Task Sequence Editor" toolbar:
1. Click the "Add Pipettor" button
2. A new pipettor task will be added to the task list
3. Default operation: SUCK at 50% volume

### 3. Editing Pipettor Tasks
1. Double-click any pipettor task in the task tree
2. A large edit dialog (550x900) will open with all controls
3. Configure the operation and parameters
4. Click "Save" to apply changes

## Edit Dialog Components

### Operation Selector
**Location:** Top of dialog
- **Type:** Dropdown (readonly)
- **Options:**
  - SUCK - Aspirate liquid
  - EXPEL - Dispense liquid
  - EJECT_TIP - Eject pipette tip
  - SET_LED - Change LED color

### Volume Control Section
**Components:**
1. **Text Entry Field**
   - Enter precise values (e.g., 0.75)
   - Range: 0.0 to 1.0
   - Used for SUCK and EXPEL operations

2. **Slider Control**
   - Drag to adjust volume easily
   - Automatically updates text entry
   - Range: 0% (left) to 100% (right)

**Note:** Volume is ignored for EJECT_TIP and SET_LED operations

### LED Color Control Section
**Components:**
1. **Red Slider** - Adjust red component (0.0-1.0)
2. **Green Slider** - Adjust green component (0.0-1.0)
3. **Blue Slider** - Adjust blue component (0.0-1.0)
4. **Alpha Slider** - Adjust brightness/alpha (0.0-1.0)

5. **Color Preview Canvas**
   - Shows the selected color in real-time
   - Updates as you move sliders
   - Visual confirmation of color choice

6. **Preset Color Buttons**
   - **Red:** Pure red (1.0, 0.0, 0.0)
   - **Green:** Pure green (0.0, 1.0, 0.0)
   - **Blue:** Pure blue (0.0, 0.0, 1.0)
   - **Yellow:** Bright yellow (1.0, 1.0, 0.0)
   - **Purple:** Purple (0.5, 0.0, 0.5)
   - **White:** White (1.0, 1.0, 1.0)
   - **Off:** Turn off LED (0.0, 0.0, 0.0)

**Note:** LED color is only used for SET_LED operation

### Operation Info Panel
**Purpose:** Quick reference for each operation
- SUCK: Aspirate liquid (uses volume_pct)
- EXPEL: Dispense liquid (uses volume_pct)
- EJECT_TIP: Eject pipette tip (volume_pct ignored)
- SET_LED: Change LED color (uses led_color, volume_pct ignored)

## Task Tree Display

Pipettor tasks appear in the task tree with descriptive details:

### SUCK/EXPEL Operations
```
Step  Action     Details
1     pipettor   SUCK at 80% volume
2     pipettor   EXPEL at 50% volume
```

### SET_LED Operation
```
Step  Action     Details
3     pipettor   SET_LED (R:1.0, G:0.0, B:0.0)
```

### EJECT_TIP Operation
```
Step  Action     Details
4     pipettor   EJECT_TIP
```

## Common Workflows

### Workflow 1: Simple Liquid Transfer
1. Set start gripper to "pipettor"
2. Add MoveTo task (approach source)
3. Add Pipettor task:
   - Operation: SET_LED
   - Color: Green (indicates ready)
4. Add Pipettor task:
   - Operation: SUCK
   - Volume: 0.8 (80%)
5. Add MoveTo task (approach destination)
6. Add Pipettor task:
   - Operation: SET_LED
   - Color: Red (indicates full)
7. Add Pipettor task:
   - Operation: EXPEL
   - Volume: 0.8 (80%)
8. Add Pipettor task:
   - Operation: EJECT_TIP

### Workflow 2: LED Status Indicators
Use LED colors to indicate pipettor state:
- **Green:** Ready/Empty
- **Red:** Full/Aspirated
- **Blue:** Complete/Ejected
- **Yellow:** Warning/Processing
- **Purple:** Custom state
- **White:** Cleaning/Neutral
- **Off:** Disabled/Standby

### Workflow 3: Multiple Transfers
For multiple liquid transfers:
1. SUCK operation
2. Move to first destination
3. EXPEL partial volume (e.g., 0.3)
4. Move to second destination
5. EXPEL remaining volume (e.g., 0.5)
6. EJECT_TIP when done

## Tips and Best Practices

### Volume Control
- Use slider for quick adjustments
- Use text entry for precise values
- Common values:
  - 0.25 = 25% (small amount)
  - 0.5 = 50% (half capacity)
  - 0.75 = 75% (most capacity)
  - 1.0 = 100% (full capacity)

### LED Colors
- Always SET_LED before operations to indicate state
- Use color preview to verify color before saving
- Use preset buttons for standard colors
- Remember alpha channel affects brightness
- SET_LED operations are quick, use liberally for visual feedback

### Task Organization
- Group related pipettor operations together
- Add MoveTo tasks between pipettor operations as needed
- Use descriptive LED colors to track workflow state
- Always EJECT_TIP at the end of a sequence

### Validation
- Volume must be between 0.0 and 1.0
- Invalid values will show error dialog
- All fields are validated on save
- LED colors are always saved (even if not used)

## Keyboard Shortcuts

When edit dialog is open:
- **Tab:** Navigate between fields
- **Enter:** (in text field) Move to next field
- **Esc:** Close dialog without saving
- Click "Save" or close dialog to cancel

## Saving and Loading

### Save Configuration
1. Configure all pipettor tasks
2. File → Save JSON
3. Choose filename (e.g., my_pipettor_sequence.json)
4. Configuration saved with all pipettor parameters

### Load Configuration
1. File → Load JSON
2. Select a saved configuration
3. All pipettor tasks load with their parameters
4. Double-click tasks to view/edit settings

## Execution

### Before Execution
1. Verify all pipettor tasks in task tree
2. Check volume percentages are correct
3. Verify LED colors if using status indicators
4. Test connection to MTC Server (button in config frame)

### During Execution
1. Click "Execute Task" button
2. Watch status logs for progress
3. LED colors will change on physical pipettor
4. Use "Stop Execution" if needed

### After Execution
1. Review logs for any errors
2. Check if all operations completed
3. Verify pipettor is in expected state
4. Clean up (EJECT_TIP if not already done)

## Troubleshooting

### Task Not Showing in Tree
- Check if task was added (should see log message)
- Try clicking "Add Pipettor" button again
- Check task tree has focus

### Can't Edit Task
- Ensure you double-clicked the task
- Check dialog didn't open behind main window
- Try selecting task and double-clicking again

### Volume Not Saving
- Ensure value is between 0.0 and 1.0
- Check for decimal point (0.5 not .5 or 5)
- Look for error dialog
- Try using slider instead of typing

### LED Color Not Showing Correctly
- Check all RGBA sliders
- Verify alpha is not 0.0 (invisible)
- Use preset buttons to reset
- Check color preview matches expectation

### Execution Fails
- Verify MTC server is running
- Check robot IP is correct
- Ensure pipettor is properly connected
- Review logs for specific error messages
- Verify all referenced poses exist

## Example Configurations

See `pipettor_gui_test.json` for a complete working example that demonstrates:
- All four pipettor operations
- LED color changes (green, red, blue)
- Volume control (80% suck/expel)
- Proper sequencing
- Integration with MoveTo tasks

## Support

For issues or questions:
1. Check PIPETTOR_INTEGRATION.md for technical details
2. Review example JSON configurations
3. Check MTC orchestrator logs
4. Verify pipettor hardware connection
5. Test with simple single-operation tasks first
