# External Integrations

**Analysis Date:** 2026-01-27

## Hardware Interfaces

**UR5e Robot Arm:**
- Driver: `ur_robot_driver` - Official UR ROS2 driver
- Description: `ur_description` - URDF models
- Communication: ROS 2 Control framework
- Configuration: `src/custom-ur-descriptions/ur5e_robot_description/urdf/`
- Secondary port: TCP 30002 for tool voltage commands

**Zivid 3D Camera (Primary Vision):**
- SDK Version: 2.16.0 (`docker/erobs-common-img/Dockerfile`)
- ROS2 Driver: `zivid_camera` package (`src/vision/zivid-ros/`)
- Services:
  - `CaptureAndDetectMarkers` - ArUco marker detection
  - `zivid_camera/capture` - Point cloud capture
- Configuration: `src/beambot/config/zivid_3d_settings.yml`
- Hand-eye calibration: `src/custom-ur-descriptions/ur5e_robot_description/urdf/zivid_camera_mount.xacro`

**ZED Stereo Camera (Alternative):**
- ROS2 Wrapper: `zed_wrapper` (`src/vision/zed-ros2-wrapper/`)
- Version: 5.0.0
- Features: Depth sensing, stereo vision, point cloud

**Robotiq Hand-E Gripper:**
- Driver: `robotiq_hande_driver` (`src/end_effectors/robotiq_hande_driver/`)
- Communication: Modbus over serial
- ros2_control: Hardware interface plugin
- Description: `robotiq_hande_description`

**OnRobot ePick Vacuum Gripper:**
- Driver: `epick_driver` (`src/end_effectors/ros2_epick_gripper/`)
- Communication: Serial interface
- ros2_control: Hardware interface plugin
- Controllers: `epick_controllers`

**Custom Pipettor Tool:**
- Driver: `pipette_driver` (`src/end_effectors/pipettor/`)
- Communication: Serial/socket interface
- Python-based hardware interface

## Third-Party SDKs

**Zivid Python SDK:**
- Installed: Via package manager in Docker
- Used in: `src/beambot/beambot/camera/zivid.py`
- Features:
  - Marker detection
  - Point cloud processing
  - Image capture with settings profiles

**OpenCV:**
- Installed: `python3-opencv` in Docker
- Used for:
  - ArUco marker detection
  - Hough circle detection
  - Contour finding and analysis
  - Image preprocessing

**Open3D:**
- Installed: Via pip (`src/vision/zivid-python-samples/requirements.txt`)
- Used for: 3D point cloud processing and visualization

**URRTDE (UR RTDE):**
- Location: `src/vision/zivid-python-samples/source/.../3rdParty/rtde-2.3.6/`
- Used for: Hand-eye calibration robot control

## ROS2 Ecosystem

**Motion Planning:**
- `moveit_task_constructor_core` - Task composition (pybind11 bindings)
- `moveit_ros_planning_interface` - MoveIt Python interface
- `moveit_kinematics` - IK solver (KDL plugin)
- `moveit_planners` - OMPL and Pilz planners

**Transform & Geometry:**
- `tf2_ros` - Transform tree
- `tf2_geometry_msgs` - Geometry message conversions
- `tf_transformations` - TF utilities

**Vision & Perception:**
- `cv_bridge` - OpenCV-ROS message conversion
- `image_transport` - Efficient image streaming
- `sensor_msgs` - Image and PointCloud2 messages

**Controllers:**
- `ros2_control` - Controller framework
- `controller_manager_msgs` - Controller switching
- `gripper_controllers` - Generic gripper control

## Bluesky Integration

**Bluesky RunEngine:**
- Installed: Via pip in bsui container
- Used for: Experiment orchestration, AI-driven sample selection

**Ophyd:**
- Installed: Via pip in bsui container
- Implementation: `src/bluesky_ros/mtc_ophyd_device.py`
- Pattern: ROS2 action client wrapped as Ophyd device

**EPICS:**
- Installed: Compiled from source in bsui container
- Used for: Beamline control integration

**nslsii:**
- Installed: Via pip in bsui container
- Purpose: NSLS-II specific beamline infrastructure

## Containerization

**Docker Images:**

1. `docker/erobs-common-img/Dockerfile` - Main robot runtime
   - Base: `osrf/ros:humble-desktop-full`
   - Contains: ROS Humble, MoveIt, Zivid SDK 2.16.0, Intel OpenCL

2. `docker/bsui/Dockerfile` - Bluesky user interface
   - Base: `osrf/ros:humble-desktop-full`
   - Contains: ROS Humble, Bluesky, Ophyd, EPICS

3. `.devcontainer/Dockerfile` - Development environment
   - Contains: Full dev tools, Pixi package manager

**Network Architecture:**
- ROS2 DDS between containers
- TCP/IP over Docker bridge network
- Robot on separate network (robot IP in config)

## Environment Configuration

**Development:**
- Required env vars: None (uses defaults from launch files)
- Configuration: `src/beambot/config/default_beamline.yaml`
- Fake hardware mode: `use_fake_hardware:=true` launch arg

**Production:**
- Robot IP: Configured in `default_beamline.yaml`
- Camera: Must be connected and accessible
- Grippers: Tool voltage set via UR secondary port

## Data Formats

**Task JSON:**
```json
{
  "start_gripper": "hande",
  "tasks": [
    {"task_type": "moveto", "target": "home"},
    {"task_type": "pick_and_place", ...}
  ],
  "poses": {"home": [0, -90, 90, -90, -90, 0]}
}
```

**Configuration YAML:**
```yaml
beamline: "default"
robot:
  ip: "192.168.1.101"
grippers:
  hande:
    moveit_package: "ur_zivid_hande_moveit_config"
    tool_voltage: 24
```

## Webhooks & Callbacks

**Incoming:**
- ROS2 action goals - Task execution requests
- ROS2 service calls - Pause/resume, status queries

**Outgoing:**
- ROS2 action feedback - Execution progress
- ROS2 topic publishing - Execution state

---

*Integration audit: 2026-01-27*
*Update when adding/removing external services*
