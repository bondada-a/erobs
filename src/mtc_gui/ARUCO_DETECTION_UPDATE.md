# ArUco Marker Detection Update for MTC GUI

## Overview

The MTC GUI has been updated to support **ArUco marker detection** using Zivid's built-in 3D marker detection capabilities. The interface now reflects this change throughout.

## Changes Made

### 1. Vision MoveTo Task Editor

**Updated Fields:**
- "AprilTag ID" → **"ArUco Marker ID"**
- Added **"Marker Dictionary"** dropdown with 12 common ArUco dictionaries:
  - 4x4: aruco4x4_50, aruco4x4_100, aruco4x4_250
  - 5x5: aruco5x5_50, aruco5x5_100, aruco5x5_250
  - 6x6: aruco6x6_50, aruco6x6_100, aruco6x6_250
  - 7x7: aruco7x7_50, aruco7x7_100, aruco7x7_250

**Updated Info Text:**
```
Vision MoveTo will:
1. Capture 3D point cloud using Zivid camera
2. Detect ArUco marker using built-in detection
3. Transform marker pose to robot base frame
4. Move gripper to detected marker position
5. Cache detection for efficiency (30s)
```

### 2. Camera View Panel

**Live Visualization:**
- Status text: "X ArUco marker(s) detected"
- Detection overlay: Green boxes around detected markers
- Marker ID labels: "ID: X" displayed on each marker

**Detection Info Panel:**
```
Detected X ArUco marker(s):

Marker ID: 3
Family: tag36h11
Hamming: 0
Decision Margin: 95.32
---
```

### 3. Task List Display

Tasks now show marker dictionary:
```
vision_moveto | Detect ArUco 3 (aruco4x4_50, timeout: 10.0s)
```

## How It Works

### Unified Zivid Detection System

The GUI now uses **the same Zivid detection service** as the backend for consistent ArUco marker detection:

#### **Zivid 3D Detection** (used by both backend and GUI)
- **Service:** `/capture_and_detect_markers`
- **Method:** Captures full 3D point cloud with built-in ArUco detection
- **Purpose:** Accurate 3D pose detection for robot control + GUI visualization
- **Dictionary:** Configurable (aruco4x4_50, aruco5x5_50, etc.)
- **Backend:** On-demand when executing vision_moveto/vision_pick_place tasks
- **GUI:** Manual on-demand detection via button

#### **GUI Visualization Details**
- Manual detection via "Detect Markers" button (no automatic polling)
- Calls Zivid service on-demand when button is clicked
- Caches detection results and overlays them on live 2D stream
- Uses `corners_in_pixel_coordinates` from service response for 2D overlay
- Displays marker ID and 3D position in camera frame
- Shows green bounding boxes around detected markers
- Overlays persist on live video until next detection

### Why This Approach?

- **Consistency:** GUI and backend use identical detection method
- **3D Accuracy:** Full 3D pose information available for display
- **No Extra Nodes:** No need for separate apriltag_ros node
- **True ArUco:** Uses actual ArUco detection (not AprilTag approximation)
- **Efficient:** Only captures 3D point cloud when explicitly requested
- **Live Overlay:** Cached detections overlaid on continuous 2D stream

## Configuration

### In Task JSON Files

```json
{
  "task_type": "vision_moveto",
  "tag_id": 3,
  "marker_dictionary": "aruco4x4_50",
  "timeout": 10.0
}
```

### Marker Dictionary Selection

Choose the dictionary that matches your **physical markers**:

| Dictionary | Marker Size | Total IDs | Use Case |
|-----------|-------------|-----------|----------|
| aruco4x4_50 | 4x4 bits | 50 IDs (0-49) | Small workspace, few objects |
| aruco4x4_100 | 4x4 bits | 100 IDs | Medium workspace |
| aruco5x5_50 | 5x5 bits | 50 IDs | More robust detection |
| aruco6x6_250 | 6x6 bits | 250 IDs | Large workspace, many objects |
| aruco7x7_250 | 7x7 bits | 250 IDs | Most robust, longer range |

**Important:** The dictionary in your GUI task **must match** the actual markers you printed!

## Usage

### Creating a Vision MoveTo Task

1. **Click "Add Vision MoveTo"** in the toolbar
2. **Edit the task:**
   - Set **Marker ID** (e.g., 3)
   - Select **Marker Dictionary** (e.g., aruco4x4_50)
   - Set **Timeout** (default: 10.0s)
3. **Save** and add to task sequence
4. **Execute** - the task will use Zivid for 3D detection

### Monitoring in Camera View

**To detect markers:**
1. Click **"Detect Markers"** button in camera panel
2. Wait ~1 second for 3D capture and detection
3. Detection overlays appear on live video

**Visual feedback:**
- **Green boxes** appear around detected markers
- **Marker IDs** are labeled on each detection
- **Status** shows count: "X ArUco marker(s) detected"
- **Info panel** shows 3D position of each marker
- **Overlays persist** on live video until next detection

## Troubleshooting

### "No markers detected" in GUI

The GUI and backend use the **same Zivid detection service**. If GUI shows no markers:

**Check Zivid service:**
```bash
# Verify Zivid service is available
ros2 service list | grep capture_and_detect

# Test manual detection
ros2 service call /capture_and_detect_markers zivid_interfaces/srv/CaptureAndDetectMarkers \
  "{marker_ids: [3], marker_dictionary: 'aruco4x4_50'}"
```

**Common issues:**
- Zivid camera node not running (check launch file)
- Marker dictionary mismatch (printed marker vs. configured dictionary)
- Marker too small or too far from camera
- Poor lighting conditions
- Marker partially occluded or damaged
- Camera not initialized (takes ~10 seconds on startup)

### Task fails to detect marker

- Same troubleshooting as GUI (same detection system)
- Check that marker dictionary in task matches physical markers
- Verify markers are within camera's working distance (typically 0.3-1.5m)

## Testing

```bash
# 1. Launch action servers (includes Zivid camera)
ros2 launch mtc_pipeline modular_action_servers.launch.py

# 2. Launch GUI
ros2 launch mtc_gui mtc_gui_client.launch.py

# 3. In GUI:
# - Camera view shows live 2D stream
# - Click "Detect Markers" button
# - Wait ~1 second for detection
# - Green boxes appear around markers on live video
# - Detection info panel shows marker IDs and 3D positions

# 4. Create and execute tasks:
# - Create vision_moveto task with marker_id = 3, dictionary = aruco4x4_50
# - Execute task
# - Robot moves to detected marker pose using same Zivid detection
```

## Backend Integration

The GUI changes are fully integrated with the updated `vision_stages.cpp`:

```cpp
// vision_stages.cpp calls Zivid service with GUI parameters
auto request = std::make_shared<zivid_interfaces::srv::CaptureAndDetectMarkers::Request>();
request->marker_ids = {tag_id};              // From GUI
request->marker_dictionary = marker_dictionary_;  // From GUI

auto future = capture_marker_client_->async_send_request(request);
```

## Summary

✅ **GUI Updated:** All "AprilTag" references changed to "ArUco Marker"
✅ **Dictionary Selection:** Dropdown with 12 common ArUco dictionaries
✅ **Live Visualization:** Camera view shows live 2D stream with detection overlays
✅ **Manual Detection:** "Detect Markers" button for on-demand 3D detection
✅ **Unified Detection:** GUI and backend use same Zivid service
✅ **3D Information:** Detection info shows full 3D pose in camera frame
✅ **No Extra Dependencies:** Removed apriltag_ros requirement
✅ **Efficient:** Only captures 3D when requested (no continuous polling)
✅ **Backward Compatible:** Existing tasks still work

The system now provides professional ArUco marker detection with:
- Consistent detection method between GUI and robot control
- Efficient use of camera (no unnecessary 3D captures)
- Live overlay of cached detections on continuous 2D stream