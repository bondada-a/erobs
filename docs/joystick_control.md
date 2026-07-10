# 8BitDo Joystick Control

This setup uses an 8BitDo Ultimate 2C Wireless Controller for base-frame
Cartesian control of the UR arm through MoveIt Servo.

## Software path

```text
8BitDo receiver → Linux xpad → ROS joy → teleop_twist_joy
                 → MoveIt Servo → scaled_joint_trajectory_controller → UR arm
```

MoveIt Servo provides joint-limit, singularity, smoothing, and collision
handling. The UR driver and teach-pendant speed slider remain authoritative for
hardware execution and speed scaling.

## Setup

Use the supplied 2.4 GHz receiver. Linux handles its `2dc8:310a` XInput device
with the kernel `xpad` driver. Install the ROS runtime packages if missing:

```bash
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy \
  ros-jazzy-moveit-servo joystick

ros2 run joy joy_enumerate_devices
```

The expected device name is `8BitDo Ultimate 2C Wireless Controller`.

## Normal MCP launch

```bash
export BEAMBOT_BEAMLINE_CONFIG=$(realpath src/beambot/config/cms_beamline.yaml)
./start_mcp_nobag.sh enable_joystick:=true
```

The orchestrator launches the gripper-specific MoveIt stack lazily. Send the
first GUI or MCP goal with the physically attached gripper before using the
controller. Joystick control is ready after the launch output reports:

```text
Servo initialized successfully
ServoCommandType_Response(success=True)
Robot ready with <gripper> configuration
```

## Controls

The bumpers are deadman switches and must remain held while moving a stick.

- Hold **RB**: left stick moves X/Y; right stick vertical moves Z.
- Hold **LB**: left stick rotates roll/pitch; right stick horizontal rotates yaw.
- Release the bumper: stop.
- Do not hold LB and RB together.

Configured full-stick limits are 0.20 m/s translation and 0.60 rad/s rotation.
The teach-pendant speed slider can reduce both.

## Mock-hardware test

Test mappings without the real robot:

```bash
export BEAMBOT_BEAMLINE_CONFIG=$(realpath src/beambot/config/cms_beamline.yaml)

ros2 launch cms_moveit_config robot_bringup.launch.py \
  use_mock_hardware:=true enable_joystick:=true
```

## Troubleshooting

Verify Linux input first:

```bash
lsusb -d 2dc8:310a
jstest /dev/input/js0
```

With the MoveIt stack running, verify each ROS stage:

```bash
ros2 topic echo /joy
ros2 topic echo /joystick/servo_node/delta_twist_cmds
ros2 topic echo /joystick/servo_node/status
ros2 control list_controllers
```

Expected behavior:

- `/joy` changes when sticks and bumpers move.
- `/joystick/servo_node/delta_twist_cmds` changes only while RB or LB is held.
- `scaled_joint_trajectory_controller` is active.
- Servo status explains collision or singularity slowdowns and stops.

If the receiver is absent, update the Ubuntu kernel rather than installing an
unsigned third-party controller driver.

## Safety

Before real motion, clear the cell, keep the teach-pendant E-stop ready, start
the External Control program, and begin with a low speed-slider setting. The
gamepad deadman is an operational control, not a safety-rated device.
