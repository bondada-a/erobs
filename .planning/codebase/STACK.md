# Technology Stack

**Analysis Date:** 2026-01-17

## Languages

**Primary:**
- Python 3.10+ - All application code (action servers, stages, GUI, camera)
- C++ - ROS2 nodes (ArUco detection, Zivid ROS driver)

**Secondary:**
- YAML - Configuration files, ROS2 parameters
- XACRO/URDF - Robot descriptions
- Dockerfile - Container definitions

## Runtime

**Environment:**
- ROS2 Humble (LTS) - Base image: `osrf/ros:humble-desktop-full`
- Python 3.10+ (ROS2 Humble baseline)
- Constraint: `numpy<2` pinned for ROS2 Humble compatibility (cv_bridge, tf_transformations)

**Package Manager:**
- pip - Python dependencies
- rosdep - ROS package dependency resolution
- colcon - ROS2 build tool
- vcs - Multi-repo version control (`.repos` files)

## Frameworks

**Core:**
- ROS2 Humble - Middleware (nodes, actions, services, topics)
- MoveIt 2 - Motion planning and kinematics
- MoveIt Task Constructor (MTC) - Hierarchical task planning (C++ with pybind11)

**Motion Planning:**
- OMPL - Open Motion Planning Library (RRTConnect default)
- KDL Kinematics Plugin - IK/FK solving

**Vision:**
- OpenCV (cv2) - Image processing, ArUco detection, circle/contour detection
- Open3D (optional) - Point cloud processing, voxel downsampling
- Octomap - 3D occupancy mapping for collision avoidance

**Experiment Control:**
- Bluesky (optional) - Adaptive experiment orchestration
- Ophyd - Hardware abstraction for Bluesky integration

**GUI:**
- Tkinter - Desktop control interface

**Build/Dev:**
- ament_cmake - CMake-based ROS2 build
- ament_cmake_python - Python package integration
- setuptools - Python packaging

## Key Dependencies

**Critical:**
- `moveit_task_constructor_core` - Task-level motion planning (`src/beambot/package.xml`)
- `moveit_ros_planning_interface` - Planning scene and kinematics
- `ur_robot_driver` - Universal Robots UR5e control
- `zivid_interfaces` - Zivid camera service definitions
- `beambot_interfaces` - Custom action/message definitions (8 action types)

**Hardware Drivers:**
- `robotiq_hande_driver` - Hand-E parallel gripper (Modbus/serial)
- `epick_driver` - ePick vacuum gripper (serial/USB)
- `pipette_driver` - Custom pipettor (ROS2 action)

**Infrastructure:**
- `ros2_control` - Hardware abstraction framework
- `controller_manager` - Controller lifecycle management
- `tf2_ros`, `tf2_geometry_msgs` - Coordinate transforms
- `cv_bridge` - OpenCV ↔ ROS image conversion
- `sensor_msgs_py` - Point cloud utilities

## Configuration

**Environment:**
- No environment variables required for basic operation
- Docker containers handle all dependencies
- Configuration via YAML files and launch arguments

**Build:**
- `package.xml` - ROS2 package manifests
- `CMakeLists.txt` - C++ build configuration
- `setup.py` / `setup.cfg` - Python package configuration

**Runtime Configuration:**
- `src/beambot/config/default_beamline.yaml` - Beamline-specific settings
- `src/beambot/config/grippers.yaml` - Gripper definitions
- `src/beambot/config/zivid_settings.yml` - Camera capture settings
- `src/custom-ur-descriptions/ur5e_moveit_configs/*/config/*.yaml` - MoveIt configs

## Platform Requirements

**Development:**
- Linux (Ubuntu 22.04 recommended for ROS2 Humble)
- Docker for containerized development
- colcon for building: `colcon build && source install/setup.bash`

**Production:**
- Docker containers (3 services):
  - `erobs-common-img` - MTC pipeline, MoveIt, Zivid SDK, action servers
  - `bsui` - Bluesky experiment orchestration (lightweight)
  - `ur-driver` - UR robot communication (separate for real hardware)
- Zivid SDK 2.16.0 (pre-installed in Docker)
- Intel OpenCL runtime (CPU compute for Zivid in GPU-less environments)

---

*Stack analysis: 2026-01-17*
*Update after major dependency changes*
