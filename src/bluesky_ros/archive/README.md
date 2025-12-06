# Archived Bluesky-ROS Integration Files

This directory contains deprecated Bluesky integration files that are no longer functional or maintained.

## Archived Files

### `pdf_beamtime.py`
- **Deprecated:** Sep 2023
- **Reason:** Missing dependencies (`custom_msgs.action.PickPlace`)
- **Original Purpose:** Beamline workflow with pick-place robot actions
- **Replaced by:** `simple_mtc_bluesky.py` and `mtc_ophyd_device.py`

### `pdf_beamtime_demo.py`
- **Deprecated:** Sep 2023
- **Reason:** Missing dependencies (`pdf_beamtime_interfaces.action.PickPlaceControlMsg`)
- **Original Purpose:** PDF beamline demo with hardcoded joint goals
- **Replaced by:** MTC-based approach with JSON task definitions

### `re_demo.py`
- **Deprecated:** Sep 2023
- **Reason:** Missing dependencies (`hello_moveit_interfaces.action.PickPlaceRepeat`)
- **Original Purpose:** Early Bluesky+ROS demo from Brookhaven collaboration
- **Replaced by:** Current MTC integration examples

## Current Active Files (Parent Directory)

Use these instead:
- **`ophyd_ros.py`** - Base class for ROS 2 Action → Bluesky integration
- **`mtc_ophyd_device.py`** - Native Python ROS 2 Ophyd device for MTC
- **`mtc_bluesky_example.py`** - Example using native MTC Ophyd device
- **`simple_mtc_bluesky.py`** - Production approach using subprocess wrapper

---

**Note:** These files are kept for historical reference only. They will not run without the missing action message packages.
