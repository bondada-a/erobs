# Codebase Structure

**Analysis Date:** 2026-01-17

## Directory Layout

```
experimental/
├── src/                           # Source packages
│   ├── beambot/                  # Core orchestration and action servers
│   ├── beambot_interfaces/       # Action/message definitions
│   ├── bluesky_ros/              # Bluesky integration (no package.xml)
│   ├── mtc_gui/                  # GUI client
│   ├── custom-ur-descriptions/   # Robot URDFs and MoveIt configs
│   ├── end_effectors/            # Gripper drivers and descriptions
│   ├── vision/                   # Vision submodules (Zivid, ArUco)
│   ├── aruco_pose/               # ArUco marker detection (C++)
│   └── demos/                    # Example implementations
├── docker/                        # Container definitions
├── install/                       # Built packages (generated)
├── build/                         # Build artifacts (generated)
├── log/                           # Build logs (generated)
├── recorded_bags/                 # ROS bag recordings
├── CLAUDE.md                      # Project documentation
└── .planning/                     # GSD planning files
```

## Directory Purposes

**src/beambot/ (Core Package):**
- Purpose: Main orchestration, action servers, and stage implementations
- Contains: Python action servers, MTC stages, camera abstraction
- Key files:
  - `beambot/action_servers/orchestrator.py` - Central coordinator
  - `beambot/action_servers/base_action_server.py` - Base class for all servers
  - `beambot/stages/base_stages.py` - MTC stage utilities
  - `beambot/camera/zivid.py` - Zivid camera wrapper
- Subdirectories:
  - `action_servers/` - 8 action server implementations
  - `stages/` - 7 stage class implementations
  - `camera/` - Camera abstraction layer
  - `core/` - MoveIt lifecycle management
  - `launch/` - ROS2 launch files
  - `config/` - YAML configurations

**src/beambot_interfaces/ (Message Package):**
- Purpose: ROS2 action and message definitions
- Contains: 8 action types for all operations
- Key files:
  - `action/MTCExecution.action` - Orchestrator goal format
  - `action/MoveToAction.action` - Joint/Cartesian moves
  - `action/PickPlaceAction.action` - 9-stage pick/place
  - `action/VisionMoveToAction.action` - Vision-guided moves

**src/bluesky_ros/ (Integration):**
- Purpose: Bluesky experiment control integration
- Contains: Ophyd device wrappers, task builders
- Key files:
  - `mtc_ophyd_device.py` - Main Ophyd device
  - `simple_mtc_bluesky.py` - Usage examples
- Note: No package.xml (not a ROS2 package)

**src/mtc_gui/ (GUI Package):**
- Purpose: Desktop control interface
- Contains: Tkinter GUI for task creation and execution
- Key files:
  - `mtc_gui/mtc_gui_client.py` - Main GUI application
  - `mtc_gui/pose_editor.py` - Pose editing dialogs
  - `mtc_gui/poses_manager.py` - Pose storage management

**src/custom-ur-descriptions/ (Robot Configs):**
- Purpose: UR5e URDF and MoveIt configurations
- Contains: Robot descriptions, gripper-specific MoveIt configs
- Subdirectories:
  - `ur5e_robot_description/` - Base URDF + camera mount XACRO
  - `ur5e_moveit_configs/` - 4 gripper-specific MoveIt configurations:
    - `ur_standalone_moveit_config/`
    - `ur_zivid_hande_moveit_config/`
    - `ur_zivid_epick_moveit_config/`
    - `ur_zivid_pipettor_moveit_config/`

**src/end_effectors/ (Gripper Packages):**
- Purpose: Hardware drivers and descriptions for all grippers
- Subdirectories:
  - `robotiq_hande_driver/` - Hand-E ros2_control plugin
  - `robotiq_hande_description/` - Hand-E URDF
  - `ros2_epick_gripper/` - ePick driver and description
  - `pipettor/` - Pipettor driver and description

**src/vision/ (Vision Submodules):**
- Purpose: Camera drivers and calibration tools
- Subdirectories:
  - `zivid-ros/` - Official Zivid ROS2 driver (external)
  - `zivid-python-samples/` - Hand-eye calibration tools
  - `zed-ros2-wrapper/` - ZED camera support (optional)

**src/aruco_pose/ (C++ Package):**
- Purpose: ArUco marker detection and pose estimation
- Contains: C++ ROS2 node for marker detection
- Key files: `src/aruco_detection.cpp`

**docker/ (Containers):**
- Purpose: Docker container definitions
- Subdirectories:
  - `erobs-common-img/` - Main production container
  - `bsui/` - Lightweight Bluesky container

## Key File Locations

**Entry Points:**
- `src/beambot/launch/beambot_bringup.launch.py` - Main launch file
- `src/mtc_gui/mtc_gui/mtc_gui_client.py` - GUI entry point
- `src/bluesky_ros/mtc_ophyd_device.py` - Bluesky integration

**Configuration:**
- `src/beambot/config/default_beamline.yaml` - Beamline settings
- `src/beambot/config/grippers.yaml` - Gripper definitions
- `src/beambot/config/zivid_settings.yml` - Camera settings
- `src/custom-ur-descriptions/ur5e_moveit_configs/*/config/*.yaml` - MoveIt configs

**Core Logic:**
- `src/beambot/beambot/action_servers/orchestrator.py` - Central coordinator
- `src/beambot/beambot/stages/base_stages.py` - MTC stage foundation
- `src/beambot/beambot/stages/pick_place_stages.py` - 9-stage pick/place
- `src/beambot/beambot/camera/zivid.py` - Vision processing

**Testing:**
- `src/beambot/scripts/test_contour_detection.py` - Contour detection test
- `src/beambot/scripts/test_wafer_detection.py` - Wafer detection test

**Documentation:**
- `CLAUDE.md` - Comprehensive project documentation
- `src/mtc_gui/README.md` - GUI documentation

## Naming Conventions

**Files:**
- snake_case.py for Python modules (e.g., `move_to_server.py`, `base_stages.py`)
- snake_case.yaml for configs (e.g., `default_beamline.yaml`)
- PascalCase.action for action definitions (e.g., `MoveToAction.action`)
- *.launch.py for launch files (e.g., `beambot_bringup.launch.py`)
- *.xacro for URDF templates (e.g., `zivid_camera_mount.xacro`)

**Directories:**
- lowercase with underscores for packages (e.g., `beambot`, `mtc_gui`)
- lowercase with hyphens for external packages (e.g., `zivid-ros`)
- Plural for collections (e.g., `action_servers/`, `stages/`)

**Special Patterns:**
- `*_server.py` - Action server implementations
- `*_stages.py` - MTC stage implementations
- `test_*.py` - Test scripts
- `*_description/` - URDF packages
- `*_driver/` - Hardware driver packages

## Where to Add New Code

**New Action Server:**
- Implementation: `src/beambot/beambot/action_servers/{name}_server.py`
- Stages: `src/beambot/beambot/stages/{name}_stages.py`
- Action definition: `src/beambot_interfaces/action/{Name}Action.action`
- Registration: Add to `beambot_bringup.launch.py`

**New Camera Type:**
- Implementation: `src/beambot/beambot/camera/{name}.py`
- Registration: Update factory in `src/beambot/beambot/camera/__init__.py`

**New Gripper:**
- Driver: `src/end_effectors/{name}_driver/`
- Description: `src/end_effectors/{name}_description/`
- MoveIt config: `src/custom-ur-descriptions/ur5e_moveit_configs/ur_zivid_{name}_moveit_config/`
- Beamline config: Add gripper entry to `default_beamline.yaml`

**New Beamline Configuration:**
- Config file: `src/beambot/config/{beamline_name}.yaml`
- Usage: `ros2 launch beambot beambot_bringup.launch.py beamline_config:=path/to/config.yaml`

**Utilities:**
- Shared helpers: `src/beambot/beambot/core/`
- Type definitions: In relevant module or `beambot_interfaces`

## Special Directories

**install/, build/, log/:**
- Purpose: Generated by colcon build
- Source: Build artifacts, not version controlled
- Committed: No (.gitignore)

**recorded_bags/:**
- Purpose: ROS bag recordings for debugging
- Source: Manual recording during development
- Committed: No (large binary files)

**.planning/:**
- Purpose: GSD planning and codebase documentation
- Source: Generated by /gsd commands
- Committed: Yes (project planning artifacts)

---

*Structure analysis: 2026-01-17*
*Update when directory structure changes*
