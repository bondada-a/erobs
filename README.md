# EROBS - Extensible Robotic Beamline Scientist

Autonomous robotic system for sample handling at NSLS-II beamlines using UR5e arms and MoveIt Task Constructor.

## Quick Start

```bash
# Build workspace
colcon build

# Launch system (replace with your robot IP)
source install/setup.bash
ros2 launch mtc_pipeline modular_action_servers.launch.py robot_ip:=192.168.1.10
```

## System Architecture

**Core Components:**
- **mtc_pipeline**: MoveIt Task Constructor orchestrator for motion planning
- **mtc_gui**: GUI for task creation and execution
- **ur5e_moveit_configs**: MoveIt configurations for gripper variants (standalone, HandE, EPick)
- **erobs_planning_scene**: Beamline-specific collision environment

**Hardware Support:**
- UR5e robot arm
- Zivid 2+ 3D camera
- Robotiq HandE gripper
- EPick vacuum gripper

**Vision:**
- AprilTag detection for object localization
- Zivid camera integration for 3D sensing

**Beamline Integration:**
- Bluesky/Ophyd integration for data acquisition workflows

## Key Packages

### mtc_pipeline
Task orchestration using MoveIt Task Constructor pattern with modular action servers:
- pick_place_action_server
- tool_exchange_action_server
- move_to_action_server
- end_effector_action_server
- vision_move_to_action_server

See [src/mtc_pipeline/README.md](src/mtc_pipeline/README.md) for design details.

### end_effectors
Gripper drivers and descriptions. See [src/end_effectors/README.md](src/end_effectors/README.md).

### ur5e_moveit_configs
Three MoveIt configurations, one per gripper type. Launch files handle gripper-specific payload and controllers.


## Testing

```bash
# Build and run tests
colcon build
colcon test --packages-select mtc_pipeline
colcon test-result --verbose
```

## Dependencies

- ROS 2 Humble
- MoveIt 2
- Universal Robots ROS2 driver
- Zivid ROS2 driver (for 3D camera)

## Contributing

This is an active research project. Contact the maintainers before making significant changes.
