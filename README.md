# Extensible Robotic Beamline Scientist

Autonomous robotic sample handling system for synchrotron beamlines at NSLS-II. Integrates ROS2 robotics with Bluesky experiment orchestration to enable self-driving beamlines.

## Architecture Overview

```
Bluesky RunEngine (experiment orchestration)
            ↓
Ophyd Device (ROS2 Action Client wrapper)
            ↓
MTCOrchestratorActionServer (JSON task dispatcher)
            ↓
Specialized Action Servers (7 types)
├── move_to, pick_place, end_effector
├── tool_exchange, vision, pipettor
└── vision_pick_place
            ↓
MoveIt Task Constructor (motion planning)
            ↓
UR5e Robot + Grippers
```

**Deployment**: Two Docker containers communicating via ROS2 DDS:
- **bsui**: Bluesky/experiment orchestration, sends JSON task goals
- **erobs-common-img**: MTC pipeline servers, MoveIt, Zivid SDK

## Getting External Dependencies

Some packages are imported from external repositories via `vcs`. After cloning, run:

```bash
# End effector drivers (HandE, EPick, Pipettor)
vcs import src/end_effectors < src/end_effectors/end_effectors.repos

# Vision drivers (Zivid, ZED)
vcs import src/vision < src/vision/vision.repos
```

See [end_effectors/README.md](./src/end_effectors/README.md) and [vision/README.md](./src/vision/README.md) for SDK requirements.

## Quick Start

```bash
colcon build && source install/setup.bash
ros2 launch beambot beambot_bringup.launch.py                        # Real hardware
ros2 launch beambot beambot_bringup.launch.py use_fake_hardware:=true # Simulation
ros2 run mtc_gui mtc_gui_client                                       # GUI client
```

## Contents

The majority of the contents in this repository are ROS2 packages with associated container image manifests.
Each manifest in the [docker](./docker) directory is a container image that can be used to run a specific application in the system.

### Source Contents

- [ros2.repos](./src/ros2.repos): ROS2 workspace file for downloading the required external ROS2 dependencies.
- [beambot](./src/beambot): Main ROS2 package providing action servers, MTC stages, and the orchestrator for robot control.
- [beambot_interfaces](./src/beambot_interfaces): ROS2 package defining action interfaces (8 actions: Orchestrator, MoveTo, PickPlace, EndEffector, ToolExchange, Vision, Pipettor, VisionPickPlace).
- [mtc_gui](./src/mtc_gui): GUI client for composing and executing robot tasks.
- [end_effectors](./src/end_effectors): End effector drivers and configuration for grippers and vacuum systems.
  - [end_effectors.repos](./src/end_effectors/end_effectors.repos): VCS file for downloading robotiq_hande_driver, robotiq_hande_description, ros2_epick_gripper, and serial packages.
  - [epick_config](./src/end_effectors/epick_config): Site-specific configuration overlay for EPick vacuum gripper.
- [custom-ur-descriptions](./src/custom-ur-descriptions): Custom UR5e robot arm descriptions with attached grippers.
  - [ur5e_robot_description](./src/custom-ur-descriptions/ur5e_robot_description): UR5e robot arm with Zivid camera mount and gripper attachments.
  - [ur5e_moveit_configs](./src/custom-ur-descriptions/ur5e_moveit_configs): MoveIt configurations for UR5e with each gripper type (HandE, EPick, Pipettor).
- [vision](./src/vision): Vision system packages for Zivid 3D camera and ArUco marker detection.
- [bluesky_ros](./src/bluesky_ros): Python module for integrating Bluesky and ROS2 via Ophyd devices.
- [aruco_pose](./src/aruco_pose): ROS2 package for detecting ArUco markers and calculating their pose.
- [pdf](./src/pdf): PDF beamline specific applications (legacy).
- [cms](./src/cms): CMS beamline specific applications (placeholder).
- [lix](./src/lix): LIX beamline specific applications (placeholder).
- [demos](./src/demos): Demonstration applications.
  - [hello_moveit](./src/demos/hello_moveit): ROS2 package demonstrating simple MoveIt actions.
  - [hello_moveit_interfaces](./src/demos/hello_moveit_interfaces): Interfaces for hello_moveit.

### Docker Contents

We use Podman throughout this work, but have named the container images with Docker in mind.

- [erobs-common-img](./docker/erobs-common-img): Main container image running UR driver, MoveIt, gripper services, and beambot action servers.
- [bsui](./docker/bsui): Container image for running the Bluesky User Interface with mounts at NSLS-II.
- [azure-kinect](./docker/azure-kinect): Container image for running the Azure Kinect ROS2 driver.
- Other auxiliary container images for development and testing:
  - [ursim](./docker/ursim): Container image for running a simulated UR5e robot arm with a teach pendant.
  - [ur-driver](./docker/ur-driver): Container image for running the UR5e robot arm ROS2 driver.
  - [ur-moveit](./docker/ur-moveit): Container image for running MoveIt with the UR5e robot arm.

## Hardware

- **Robot**: UR5e 6-DOF arm
- **Camera**: Zivid 2+ 3D with ArUco marker detection
- **Grippers** (swappable): Robotiq Hand-E, Robotiq ePick, Pipettor

## Task JSON Format

```json
{
  "start_gripper": "hande",
  "tasks": [
    {"task_type": "moveto", "target": "home"},
    {"task_type": "pick_and_place", "pick_approach": "approach", "pick_target": "pick", "place_approach": "approach", "place_target": "place"},
    {"task_type": "vision_moveto", "tag_id": 5}
  ],
  "poses": {"home": [0, -90, 90, -90, -90, 0]}
}
```

Note: Joint poses are in **degrees**, converted to radians internally.

## Using Containers

The complete application uses a 1-node-per-container model. The containers are currently orchestrated by bash scripts detailed in the READMEs of each container image. Specifically, the full application is detailed in [erobs-common-img](./docker/erobs-common-img/README.md).

## Debugging

```bash
ros2 action list                                    # Check action servers
ros2 run tf2_tools view_frames                      # TF tree (vision issues)
ros2 topic echo /joint_states                       # Joint states
ros2 service call /beambot/pause std_srvs/srv/Trigger  # Pause execution
ros2 topic echo /beambot/execution_state            # Monitor state
```

## References

- [Digital Discovery Paper (2025)](https://doi.org/10.1039/d5dd00036j) - Full architecture
- [ICRA 2024 Paper](https://doi.org/10.1109/ICRA57147.2024.10611706) - Bluesky-ROS integration
- [MoveIt Task Constructor](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor.html)

## Notes on VSCode Workspace

VSCode ROS2 Workspace Template Borrowed from @althack.
See [how she develops with vscode and ros2](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for more details.

ROS2-approved formatters are included in the IDE:
- **c++** uncrustify; config from `ament_uncrustify`
- **python** autopep8; vscode settings consistent with the [style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/)
