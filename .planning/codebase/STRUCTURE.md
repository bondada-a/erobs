# Codebase Structure

**Analysis Date:** 2026-01-27

## Directory Layout

```
experimental/
├── src/
│   ├── beambot/                    # Core robotics package
│   ├── beambot_interfaces/         # Action/message definitions
│   ├── mtc_gui/                    # GUI client
│   ├── bluesky_ros/                # Bluesky-ROS integration
│   ├── custom-ur-descriptions/     # Robot & gripper descriptions
│   ├── end_effectors/              # Gripper drivers
│   ├── vision/                     # Camera drivers & samples
│   ├── aruco_pose/                 # ArUco marker detection
│   ├── demos/                      # Example implementations
│   └── pdf/                        # PDF beamline specific
├── docker/                         # Container definitions
├── install/                        # Built packages (generated)
├── build/                          # Build artifacts (generated)
└── CLAUDE.md                       # Project documentation
```

## Directory Purposes

**src/beambot/**
- Purpose: Main robotics framework - action servers and orchestrator
- Contains: Python package with action servers, stages, camera abstraction
- Key files:
  - `beambot/action_servers/orchestrator.py` - Central coordinator
  - `beambot/stages/base_stages.py` - MTC utilities
  - `beambot/camera/zivid.py` - Vision abstraction
  - `launch/beambot_bringup.launch.py` - Main entry point
- Subdirectories:
  - `action_servers/` - 8 action server implementations
  - `stages/` - MTC stage compositions
  - `camera/` - Camera abstraction layer
  - `core/` - Infrastructure (MoveIt lifecycle)
  - `config/` - YAML configuration files
  - `launch/` - ROS2 launch files
  - `scripts/` - Test utilities

**src/beambot_interfaces/**
- Purpose: ROS2 action definitions
- Contains: 9 `.action` files, CMakeLists.txt for generation
- Key files:
  - `action/MTCExecution.action` - Main orchestrator action
  - `action/PickPlaceAction.action` - Pick/place action
  - `action/VisionMoveToAction.action` - Vision action
- Subdirectories: `action/` only

**src/mtc_gui/**
- Purpose: Tkinter GUI for task composition and execution
- Contains: Python GUI application
- Key files:
  - `mtc_gui/mtc_gui_client.py` - Main application
  - `mtc_gui/pose_editor.py` - Pose creation/editing
- Subdirectories: `mtc_gui/`, `launch/`

**src/bluesky_ros/**
- Purpose: Bluesky experiment integration
- Contains: Ophyd device wrappers, task builders
- Key files:
  - `mtc_ophyd_device.py` - Sync Ophyd device
  - `mtc_ophyd_device_async.py` - Async variant
  - `task_builder.py` - JSON task construction helpers

**src/custom-ur-descriptions/**
- Purpose: Robot URDF and MoveIt configurations
- Contains: XACRO files, MoveIt configs per gripper
- Key files:
  - `ur5e_robot_description/urdf/ur_with_zivid_hande.xacro` - Main URDF
  - `ur5e_robot_description/urdf/zivid_camera_mount.xacro` - Camera calibration
- Subdirectories:
  - `ur5e_robot_description/` - Base robot description
  - `ur5e_moveit_configs/` - MoveIt configs per gripper type

**src/end_effectors/**
- Purpose: Gripper hardware drivers
- Contains: ros2_control interfaces for each gripper
- Key files:
  - `robotiq_hande_driver/` - Hand-E driver
  - `ros2_epick_gripper/` - ePick driver
  - `pipettor/` - Pipettor driver
- Subdirectories: One per gripper type

**src/vision/**
- Purpose: Camera drivers and utilities
- Contains: Zivid ROS2 driver, ZED driver, calibration tools
- Key files:
  - `zivid-ros/zivid_camera/` - Zivid driver
  - `zivid-python-samples/` - Calibration tools
- Subdirectories: `zivid-ros/`, `zed-ros2-wrapper/`, `zivid-python-samples/`

**docker/**
- Purpose: Container definitions for deployment
- Contains: Dockerfiles for each deployment target
- Key files:
  - `erobs-common-img/Dockerfile` - Main robot container
  - `bsui/Dockerfile` - Bluesky container

## Key File Locations

**Entry Points:**
- `src/beambot/launch/beambot_bringup.launch.py` - Main system startup
- `src/mtc_gui/launch/mtc_gui_client.launch.py` - GUI client
- `src/bluesky_ros/mtc_ophyd_device.py` - Bluesky integration

**Configuration:**
- `src/beambot/config/default_beamline.yaml` - Single source of truth
- `src/beambot/config/grippers.yaml` - Gripper definitions
- `src/beambot/config/zivid_3d_settings.yml` - Camera settings
- `src/custom-ur-descriptions/*/config/` - MoveIt configs

**Core Logic:**
- `src/beambot/beambot/action_servers/orchestrator.py` - Central coordinator
- `src/beambot/beambot/stages/base_stages.py` - MTC primitives
- `src/beambot/beambot/core/moveit_lifecycle_manager.py` - MoveIt control

**Testing:**
- `src/beambot/scripts/test_*.py` - Vision test scripts
- `src/end_effectors/*/test/` - Hardware driver tests

**Documentation:**
- `CLAUDE.md` - Project overview and instructions
- `src/beambot/docs/` - Investigation notes

## Naming Conventions

**Files:**
- `snake_case.py` - Python modules
- `*_server.py` - Action server implementations
- `*_stages.py` - Stage compositions
- `*.launch.py` - ROS2 launch files
- `*.action` - ROS2 action definitions (PascalCase)

**Directories:**
- `snake_case` - All directories
- Plural for collections: `action_servers/`, `stages/`

**Special Patterns:**
- `*_moveit_config/` - MoveIt configuration packages
- `*_description/` - URDF/XACRO packages
- `*_driver/` - Hardware driver packages

## Where to Add New Code

**New Action Server:**
- Implementation: `src/beambot/beambot/action_servers/{name}_server.py`
- Stages: `src/beambot/beambot/stages/{name}_stages.py`
- Action definition: `src/beambot_interfaces/action/{Name}Action.action`
- Launch: Add to `src/beambot/launch/beambot_bringup.launch.py`

**New Gripper:**
- Driver: `src/end_effectors/{gripper_name}_driver/`
- Description: `src/custom-ur-descriptions/{gripper}_description/`
- MoveIt config: `src/custom-ur-descriptions/ur5e_moveit_configs/ur_zivid_{gripper}_moveit_config/`
- URDF: `src/custom-ur-descriptions/ur5e_robot_description/urdf/ur_with_zivid_{gripper}.xacro`
- Config entry: `src/beambot/config/grippers.yaml`

**New Detection Method:**
- Implementation: `src/beambot/beambot/camera/zivid.py` (add method)
- Stage support: `src/beambot/beambot/stages/vision_stages.py`

**Utilities:**
- Shared helpers: `src/beambot/beambot/core/`
- Type definitions: Within module or dedicated `types.py`

## Special Directories

**install/**
- Purpose: Built ROS2 packages
- Source: Generated by `colcon build`
- Committed: No (in .gitignore)

**build/**
- Purpose: Build artifacts
- Source: Generated by `colcon build`
- Committed: No (in .gitignore)

**.planning/**
- Purpose: GSD planning documents
- Source: Created by `/gsd:map-codebase`
- Committed: Yes

---

*Structure analysis: 2026-01-27*
*Update when directory structure changes*
