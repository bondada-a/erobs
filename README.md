# EROBS - Extensible Robotic Beamline Scientist

Autonomous robotic sample handling for synchrotron beamlines.

## Overview

EROBS integrates ROS2 robotics with Bluesky experiment orchestration to enable self-driving beamlines at NSLS-II. Scientists can run 24/7 sample handling without manual intervention.

### Source Contents

- [ros2.repos](./src/ros2.repos): ROS2 workspace file for downloading the required external ROS2 dependencies.
- [beambot](./src/beambot): Main ROS2 package providing action servers, MTC stages, and the orchestrator for robot control.
- [beambot_interfaces](./src/beambot_interfaces): ROS2 package defining action interfaces (8 actions: Orchestrator, MoveTo, PickPlace, EndEffector, ToolExchange, Vision, Pipettor, VisionPickPlace).
- [mtc_gui](./src/mtc_gui): GUI client for composing and executing robot tasks.
- [end_effectors](./src/end_effectors): End effector drivers and configuration for grippers and vacuum systems.
  - [end_effectors.repos](./src/end_effectors/end_effectors.repos): VCS file for downloading robotiq_hande_driver, robotiq_hande_description, ros2_epick_gripper, and serial packages.
  - [epick_config](./src/end_effectors/epick_config): Site-specific configuration overlay for EPick vacuum gripper.
- [custom-ur-descriptions](./src/custom-ur-descriptions): Custom UR5e robot arm descriptions with attached grippers.
  - [cms_robot_description](./src/custom-ur-descriptions/cms_robot_description): UR5e robot arm with Zivid camera mount and gripper attachments.
  - [cms_moveit_configs](./src/custom-ur-descriptions/cms_moveit_configs): MoveIt configurations for UR5e with each gripper type (HandE, EPick, Pipettor).
- [vision](./src/vision): Vision system packages for Zivid 3D camera and ArUco marker detection.
- [bluesky_ros](./src/bluesky_ros): Python module for integrating Bluesky and ROS2 via Ophyd devices.
- [pdf](./src/pdf): PDF beamline specific applications (placeholder).
- [cms](./src/cms): CMS beamline specific applications (placeholder).
- [lix](./src/lix): LIX beamline specific applications (placeholder).
- [demos](./src/demos): Demonstration applications.
  - [hello_moveit](./src/demos/hello_moveit): ROS2 package demonstrating simple MoveIt actions.
  - [hello_moveit_interfaces](./src/demos/hello_moveit_interfaces): Interfaces for hello_moveit.

## Hardware

- **Robot**: UR5e 6-DOF arm
- **Camera**: Zivid 2+ 3D with ArUco marker detection
- **Grippers** (swappable): Robotiq Hand-E, Robotiq ePick, Pipettor

## Setup

### Prerequisites

- ROS2 Humble
- MoveIt 2
- Zivid SDK (for vision)

### Clone & Build

```bash
git clone https://github.com/bondada-a/erobs.git
cd erobs

# Import external dependencies
vcs import src < src/ros2.repos
vcs import src/end_effectors < src/end_effectors/end_effectors.repos
vcs import src/vision < src/vision/vision.repos

# Build
colcon build && source install/setup.bash
```

See [end_effectors/README.md](./src/end_effectors/README.md) and [vision/README.md](./src/vision/README.md) for SDK requirements.

## Usage

### Launch

```bash
ros2 launch beambot beambot_bringup.launch.py                         # Real hardware
ros2 launch beambot beambot_bringup.launch.py use_fake_hardware:=true # Simulation
```

### GUI

```bash
ros2 run mtc_gui mtc_gui_client
```
