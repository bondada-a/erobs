# Vision System - Simple Guide

## Overview
Detect AprilTags and move the robot to them.

## Quick Start

### 1. Launch the Vision System (Simulation)
```bash
ros2 launch mtc_pipeline vision_system_sim.launch.py
```

This starts:
- Vision action server
- Mock AprilTag detector (publishes 3 fake tags)

### 2. Launch the Orchestrator
```bash
ros2 launch mtc_pipeline mtc_orchestrator.launch.py
```

### 3. Test It
```bash
# Move to tag 0
python3 src/mtc_pipeline/scripts/test_vision.py 0

# Move to tag 1
python3 src/mtc_pipeline/scripts/test_vision.py 1
```

## What It Does
1. Waits for AprilTag detection on `/apriltag/detections` topic
2. Gets tag pose via TF2 transform
3. Plans and executes motion to move robot to tag pose

## Tag Positions (Simulation)
- Tag 0: [0.5, 0.0, 0.02] (center, 50cm forward)
- Tag 1: [0.4, 0.2, 0.02] (left side)
- Tag 2: [0.4, -0.2, 0.02] (right side)

## Using Real Camera
Replace simulation launch with:
```bash
ros2 launch mtc_pipeline vision_system.launch.py
```

Make sure:
- Zivid camera is publishing to `/zivid_camera/color/image_raw`
- AprilTag config matches your tag sizes in `config/apriltag_config.yaml`

## Files
- `src/vision_stages.cpp` - Core logic: detect tag, move to it
- `launch/vision_system.launch.py` - Launch file for vision system
- `scripts/test_vision.py` - Simple test script
