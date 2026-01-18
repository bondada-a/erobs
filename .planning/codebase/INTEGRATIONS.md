# External Integrations

**Analysis Date:** 2026-01-17

## Hardware Interfaces

**UR5e Robot Arm (Universal Robots):**
- 6-DOF collaborative robot arm
- Communication: RTDE (Real-Time Data Exchange) over TCP/IP
- Driver: `ur_robot_driver` (official UR ROS2 integration)
- Secondary interface: Port 30002 for tool voltage commands
- Configuration: `src/custom-ur-descriptions/ur5e_robot_description/`

**Zivid 2+ 3D Camera (eye-in-hand mounted):**
- 3D structured light camera (depth + RGB)
- SDK: Zivid SDK 2.16.0 (C++ API)
- ROS2 driver: `src/vision/zivid-ros/zivid_camera/`
- Python wrapper: `src/beambot/beambot/camera/zivid.py`
- Hand-eye calibration: `src/vision/zivid-python-samples/source/applications/advanced/hand_eye_calibration/`
- Network discovery: Default IP 10.68.81.52 (configurable in Docker)
- Mount transform: `src/custom-ur-descriptions/ur5e_robot_description/urdf/zivid_camera_mount.xacro`

**Robotiq Hand-E Gripper (pneumatic parallel):**
- Communication: Modbus over serial (RS-485)
- Hardware interface: `src/end_effectors/robotiq_hande_driver/`
- URDF model: `src/end_effectors/robotiq_hande_description/`
- ros2_control plugin for integration

**Robotiq ePick Gripper (electric suction cup):**
- Communication: USB/Serial
- Driver: `src/end_effectors/ros2_epick_gripper/epick_driver/`
- URDF model: `src/end_effectors/ros2_epick_gripper/epick_description/`

**Pipettor (liquid handling end-effector):**
- Custom hardware interface
- Driver: `src/end_effectors/pipettor/pipette_driver/`
- Description: `src/end_effectors/pipettor/pipette_description/`

## External SDKs & Libraries

**Zivid SDK 2.16.0:**
- Installed in Docker: `docker/erobs-common-img/Dockerfile`
- Python bindings via `zivid_interfaces` ROS2 package
- Hand-eye calibration GUI: `src/vision/zivid-python-samples/source/applications/advanced/hand_eye_calibration/hand_eye_gui.py`

**Intel OpenCL Runtime:**
- CPU compute for Zivid SDK (GPU-less environments)
- Installed in Docker for container compatibility
- Packages: `intel-oneapi-runtime-opencl`, `intel-oneapi-runtime-compilers`

**RTDE Python Library (UR Real-Time Data Exchange):**
- Version 2.3.6 (vendored)
- Path: `src/vision/zivid-python-samples/source/applications/advanced/hand_eye_calibration/ur_hand_eye_calibration/3rdParty/rtde-2.3.6/`

## Third-Party ROS2 Packages

**From `src/ros2.repos`:**
- `moveit_task_constructor` - GitHub: ros-planning/moveit_task_constructor (branch: humble)

**From `src/end_effectors/end_effectors.repos`:**
- `serial` - Serial port library (GitHub: tylerjw/serial, branch: ros2)
- `robotiq_hande_driver` - Hand-E gripper driver (GitHub: AGH-CEAI/robotiq_hande_driver, branch: humble-devel)
- `robotiq_hande_description` - Hand-E URDF (GitHub: macmacal/robotiq_hande_description, branch: humble-devel)
- `ros2_epick_gripper` - ePick gripper package (GitHub: PickNikRobotics/ros2_epick_gripper, branch: main)
- `pipettor` - Pipettor driver (GitHub: sixym3/pipettor, branch: main)

**From `src/vision/vision.repos`:**
- `zivid-ros` - Zivid ROS2 driver (GitHub: zivid/zivid-ros, branch: master)
- `zed-ros2-wrapper` - Stereolabs ZED camera wrapper (GitHub: stereolabs/zed-ros2-wrapper, branch: master) [optional]

## Experiment Control Integration

**Bluesky (optional):**
- Adaptive experiment control framework
- Integration: `src/bluesky_ros/mtc_ophyd_device.py`
- Protocol: Ophyd Movable interface
- Communication: ROS2 action client to `beambot_orchestrator`

**Ophyd:**
- Hardware abstraction for Bluesky
- Custom `MTCExecutionDevice` wraps MTC orchestrator action
- Asynchronous goal handling with status callbacks

## ROS2 Communication

**Action Servers (8 types):**
- `/beambot_orchestrator` - Central task coordinator (MTCExecution)
- `/beambot_moveto` - Joint/Cartesian moves (MoveToAction)
- `/beambot_endeffector` - Gripper control (EndEffectorAction)
- `/beambot_pickplace` - Pick/place operations (PickPlaceAction)
- `/beambot_toolexchange` - Tool dock/load (ToolExchangeAction)
- `/beambot_vision_server` - Vision-guided moves (VisionMoveToAction)
- `/beambot_vision_pickplace` - Vision pick + hardcoded place (VisionPickPlaceAction)
- `/beambot_pipettor` - Pipettor operations (PipettorAction)

**Services:**
- `/capture_and_detect_markers` - Zivid camera capture + ArUco detection
- `/beambot/pause` - Pause execution (std_srvs/Trigger)

**Topics:**
- `/beambot/execution_state` - Execution state updates
- `/zivid/image_color` - RGB image from Zivid
- `/zivid/points/xyzrgba` - Point cloud from Zivid

## Environment Configuration

**Development:**
- Required: ROS2 Humble, colcon, rosdep
- Configuration: `src/beambot/config/default_beamline.yaml`
- Simulation: `ros2 launch beambot beambot_bringup.launch.py use_fake_hardware:=true`

**Production:**
- Secrets management: Not applicable (no cloud services)
- Network: Private network between robot, camera, and control PC
- Docker orchestration: docker-compose (not yet fully defined)

## Webhooks & Callbacks

**Incoming:**
- ROS2 action feedback callbacks from all action servers
- Controller state notifications from ros2_control

**Outgoing:**
- UR secondary interface commands (tool voltage, external control restart)
- Zivid SDK capture commands

---

*Integration audit: 2026-01-17*
*Update when adding/removing external services*
