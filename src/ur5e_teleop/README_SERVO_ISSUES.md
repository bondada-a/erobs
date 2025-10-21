# MoveIt Servo Issues in ROS 2 Humble

## Problem

MoveIt Servo (`servo_node_main`) in ROS 2 Humble has a critical bug where it loads hardcoded default parameters (`panda_arm`) and **completely ignores** user-provided parameter files and command-line overrides.

This affects all parameter loading methods:
- `--params-file`
- `-p parameter:=value`
- Launch file `parameters=[...]`

## Evidence

Even after:
1. Creating custom YAML config with `move_group_name: ur_arm`
2. Passing via launch file `parameters=[servo_yaml]`
3. Replacing `/opt/ros/humble/share/moveit_servo/config/panda_simulated_config.yaml`
4. Using command-line override `-p move_group_name:=ur_arm`

Servo **still** loads `panda_arm` and crashes with:
```
[ERROR] [moveit_robot_model.robot_model]: Group 'panda_arm' not found in model 'ur5e'
[ERROR] [moveit_servo.servo_calcs]: Invalid move group name: `panda_arm`
```

## Root Cause

The `servo_node_main` executable appears to have parameters hardcoded in the binary or loads them in a way that makes user parameters irrelevant. This is a known issue in the Humble release.

## Workarounds

### Option 1: Use MoveIt Servo from Source (Recommended for Production)

Build MoveIt Servo from source with parameter loading fixes from newer branches.

### Option 2: Skip Servo - Direct Joint Velocity Control

For this teleop package, we skip servo entirely and use simpler direct control:

**Pros:**
- No servo dependency
- More predictable
- Lower latency

**Cons:**
- No automatic singularity avoidance
- No collision checking
- Must control individual joints instead of Cartesian space

## Usage Without Servo

Just launch the robot and teleop (skip servo entirely):

```bash
# Terminal 1 - Robot
ros2 launch ur_zivid_hande_moveit_config robot_bringup.launch.py

# Terminal 2 - Teleop (publishes directly to velocity controller)
ros2 launch ur5e_teleop teleop.launch.py twist_topic:=/forward_velocity_controller/commands
```

The controller will need to be modified to accept twist commands or you'll need to implement Jacobian-based IK in the teleop node.

## Future Fix

This will be resolved when upgrading to ROS 2 Iron/Jazzy or using MoveIt from source.
