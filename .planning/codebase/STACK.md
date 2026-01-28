# Technology Stack

**Analysis Date:** 2026-01-27

## Languages

**Primary:**
- Python 3.10 - All application code (`src/beambot/`, `src/mtc_gui/`, `src/bluesky_ros/`)
- C++ (rclcpp) - MoveIt Task Constructor bindings, gripper drivers

**Secondary:**
- YAML - Configuration files (`src/beambot/config/*.yaml`)
- XML/XACRO - Robot URDF descriptions (`src/custom-ur-descriptions/ur5e_robot_description/urdf/`)

## Runtime

**Environment:**
- ROS 2 Humble (Ubuntu 22.04) - `docker/erobs-common-img/Dockerfile`
- Python 3.10 - Explicitly declared in Dockerfiles
- Zivid SDK 2.16.0 - `docker/erobs-common-img/Dockerfile` (lines 32-39)
- Intel OneAPI OpenCL Runtime - CPU computation for Zivid

**Package Manager:**
- colcon - ROS 2 build system
- pip3 - Python packages
- rosdep - ROS dependency resolution
- Lockfile: None (ROS uses rosdep for dependency resolution)

## Frameworks

**Core:**
- ROS 2 Humble - Distributed robotics middleware (`rclpy`, `rclcpp`)
- MoveIt 2 - Motion planning framework (`moveit_task_constructor_core`)
- Bluesky - Experiment orchestration (`docker/bsui/Dockerfile`)
- Ophyd - Device abstraction layer for Bluesky

**Testing:**
- Pytest - Python unit tests (`src/beambot/package.xml`)
- GTest - C++ unit tests (`src/end_effectors/ros2_epick_gripper/epick_driver/tests/`)
- ament_lint_auto - Code quality enforcement

**Build/Dev:**
- ament_cmake - CMake build helper for ROS 2 packages
- ament_python - Python package build helper
- colcon - Workspace build system

## Key Dependencies

**Critical:**
- `moveit_task_constructor_core` - Task planning with stages (`src/beambot/package.xml`)
- `ur_robot_driver` - UR5e robot control
- `zivid_camera` - Zivid 3D camera ROS2 driver
- `tf2_ros` - Transform broadcasting/listening

**Infrastructure:**
- `rclpy` - Python ROS 2 client library
- `geometry_msgs`, `sensor_msgs` - ROS message types
- `controller_manager_msgs` - Controller management
- `cv_bridge` - OpenCV-ROS bridge (`src/mtc_gui/package.xml`)

**Python Scientific:**
- `numpy<2` - Pinned for ROS 2 Humble compatibility (`docker/erobs-common-img/Dockerfile`)
- `scipy` - Hand-eye calibration (`src/vision/zivid-python-samples/*/requirements.txt`)
- `open3d` - 3D point cloud processing
- `opencv-python` - Computer vision

## Configuration

**Environment:**
- YAML beamline configs - `src/beambot/config/default_beamline.yaml`
- Gripper registry - `src/beambot/config/grippers.yaml`
- Vision settings - `src/beambot/config/zivid_3d_settings.yml`

**Build:**
- `package.xml` - ROS 2 package manifest (per package)
- `CMakeLists.txt` / `setup.py` - Build configuration
- Docker multi-stage builds for deployment

## Platform Requirements

**Development:**
- Linux (Ubuntu 22.04 recommended)
- Docker for container builds
- ROS 2 Humble desktop-full or via containers

**Production:**
- Docker containers on VM
- Two containers: `erobs-common-img` (robot), `bsui` (Bluesky)
- ROS 2 DDS networking between containers

---

*Stack analysis: 2026-01-27*
*Update after major dependency changes*
