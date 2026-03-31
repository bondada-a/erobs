# End Effectors

This directory contains drivers and configuration for robot end effectors like grippers and vacuum systems.

## Getting the Drivers

The actual driver code lives in separate repositories. To download them:

```bash
vcs import src/end_effectors < src/end_effectors/end_effectors.repos
```

This pulls in:
- `serial` - ROS2 serial communication
- `robotiq_hande_driver` - Robotiq HandE gripper driver
- `robotiq_hande_description` - Robotiq HandE URDF models
- `ros2_epick_gripper` - EPick vacuum gripper driver (forked from [PickNikRobotics/ros2_epick_gripper](https://github.com/PickNikRobotics/ros2_epick_gripper) — our fork adds always-present extension link for stable URDF chain across suction cup configurations)

**Note:** The `ros2_epick_gripper` repository includes `epick_moveit_studio` which depends on paywalled MoveIt Studio/MoveIt Pro packages. Since we don't use this package, skip it during build and dependency installation:

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -y --skip-keys moveit_studio_behavior_interface

# Build workspace (skip epick_moveit_studio)
colcon build --packages-skip epick_moveit_studio
```

## EPick Configuration

The `epick_config` package provides an overlay configuration (over ros2_epick_driver) for our specific setup:

- Serial port: `/tmp/ttyUR`
- Custom positioning offsets
- Hardware vs simulation mode
- **Suction cup profiles** (`config/suction_cups.yaml`)

### Suction Cup Profiles

The ePick supports swappable suction cups with different dimensions. Profiles are defined in `epick_config/config/suction_cups.yaml` and control the URDF geometry (extension tube, cup size, tip frame position).

To switch cups:
- **Via MCP**: `set_cup_profile(name="7mm_dia")` — no rebuild needed
- **Via beamline config**: Change `cup_profile` under `grippers.epick` in the beamline YAML

To add a new cup: add an entry to `suction_cups.yaml` with dimensions in meters, then rebuild `epick_config`.