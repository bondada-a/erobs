# UR5e Teleoperation with 8BitDo Ultimate 2C Controller

This package enables Cartesian velocity control of the UR5e robot using the 8BitDo Ultimate 2C Wireless Controller.

## Features

- **Smooth Cartesian Control**: Move the robot end-effector intuitively using joysticks
- **Gripper Control**: Open/close gripper with shoulder buttons
- **Variable Speed**: Adjust movement speed on-the-fly
- **Safety**: Deadman switch and emergency stop
- **Configurable**: Easy button/axis remapping via YAML config

## Prerequisites

### Hardware
- 8BitDo Ultimate 2C Wireless Controller
- UR5e robot (real or simulated)
- USB dongle for wireless connection (or USB-C cable for wired)

### Software
- ROS 2 Humble
- MoveIt Servo (for Cartesian velocity control)
- Joy package (for joystick input)

Install required ROS packages:
```bash
sudo apt update
sudo apt install ros-humble-joy ros-humble-moveit-servo
```

## Controller Setup

### 1. Pair the Controller

**Wireless (2.4GHz mode - Recommended):**
1. Insert the USB dongle into your computer
2. Turn on the controller (hold the Home button)
3. The controller should auto-pair (LED will stop flashing)

**Bluetooth mode:**
1. Hold Home + Y for 3 seconds to enter pairing mode
2. Pair via your system's Bluetooth settings

**Wired mode:**
1. Connect the controller via USB-C cable

### 2. Verify Controller Connection

Check that the controller is detected:
```bash
ls /dev/input/js*
```

You should see `/dev/input/js0` (or js1, js2, etc.)

Test the controller:
```bash
ros2 run joy joy_node
```

In another terminal:
```bash
ros2 topic echo /joy
```

Move the joysticks and press buttons - you should see values changing.

### 3. Set Controller Mode

The 8BitDo Ultimate 2C supports multiple input modes:
- **X-input mode** (recommended for Linux): Home + X
- **D-input mode**: Home + A
- **Switch mode**: Home + Y

**Use X-input mode for best compatibility.** The button/axis mappings in the config file are set for X-input mode.

## Building the Package

```bash
cd ~/work/github_ws/erobs
colcon build --packages-select ur5e_teleop
source install/setup.bash
```

## Usage

### Basic Usage (with MoveIt Servo)

1. **Start the UR5e with MoveIt:**

```bash
# Launch your UR5e with MoveIt (use your existing launch file)
ros2 launch ur5e_moveit_configs ur_control.launch.py
```

2. **Start MoveIt Servo:**

MoveIt Servo provides real-time Cartesian velocity control. You need to configure and launch it:

```bash
ros2 launch moveit_servo servo.launch.py
```

> **Note:** You may need to create a servo configuration file for your specific setup. See the MoveIt Servo documentation for details.

3. **Launch the teleop node:**

```bash
ros2 launch ur5e_teleop teleop.launch.py
```

4. **Control the robot:**
   - Press **A button** to enable motion (the deadman switch)
   - Use the joysticks to move the robot
   - Press **A** again to disable

### Custom Configuration

Specify a different controller device or config file:

```bash
ros2 launch ur5e_teleop teleop.launch.py joy_dev:=/dev/input/js1 config_file:=custom_config.yaml
```

## Controller Mapping

### 8BitDo Ultimate 2C (X-input mode)

| Input | Function | Description |
|-------|----------|-------------|
| **Left Joystick** | XY Translation | Left/Right: Move TCP left/right<br>Up/Down: Move TCP forward/backward |
| **Right Joystick** | Rotation | Left/Right: Yaw (rotate around Z)<br>Up/Down: Pitch (rotate around Y) |
| **LT (Left Trigger)** | Move Down | Move TCP down (negative Z) |
| **RT (Right Trigger)** | Move Up | Move TCP up (positive Z) |
| **D-Pad Left/Right** | Roll | Rotate around X-axis |
| **A Button** | Enable/Disable | Toggle motion control (deadman switch) |
| **B Button** | Home Position | Return to home (not yet implemented) |
| **X Button** | Decrease Speed | Reduce speed multiplier |
| **Y Button** | Increase Speed | Increase speed multiplier |
| **LB (Left Bumper)** | Close Gripper | Close the gripper |
| **RB (Right Bumper)** | Open Gripper | Open the gripper |
| **Home Button** | Emergency Stop | Immediately disable motion |

### Velocity Scaling

- **Base linear velocity**: 0.05 m/s
- **Base angular velocity**: 0.2 rad/s
- **Speed multiplier range**: 0.1x to 2.0x
- **Step size**: 0.1x per button press

Press **X** to slow down, **Y** to speed up. Current multiplier is logged to console.

## Configuration

Edit `/config/8bitdo_ultimate_2c.yaml` to customize:

- Button/axis mappings
- Velocity scaling factors
- Deadzone threshold
- Publish rate

Example:
```yaml
velocity_scaling:
  linear_base: 0.1      # Increase base speed
  angular_base: 0.3
  max_multiplier: 3.0   # Allow faster speeds

deadzone: 0.15          # Increase deadzone if controller drifts
```

## Troubleshooting

### Controller not detected

- Check USB dongle is inserted
- Verify device exists: `ls /dev/input/js*`
- Try different USB port
- For Bluetooth: ensure controller is in pairing mode

### Wrong button mappings

- Verify controller is in X-input mode (Home + X)
- Run `ros2 topic echo /joy` and press buttons to see indices
- Update `config/8bitdo_ultimate_2c.yaml` with correct indices

### Robot not moving

1. **Check A button is pressed** (motion must be enabled)
2. **Verify MoveIt Servo is running**:
   ```bash
   ros2 topic list | grep servo
   ```
   You should see `/servo_node/delta_twist_cmds`

3. **Check twist messages are published**:
   ```bash
   ros2 topic echo /servo_node/delta_twist_cmds
   ```

4. **Verify robot is not in protective stop or fault state**

### Gripper not responding

- Ensure gripper action server is running:
  ```bash
  ros2 action list | grep gripper
  ```
- Check that your MoveIt config includes the gripper controller
- Verify gripper is powered and connected

### Controller drift

- Increase deadzone in config file
- Calibrate controller (see 8BitDo manual)
- Check battery level

## Alternative: Without MoveIt Servo

If you don't have MoveIt Servo configured, you can modify the node to publish to a different topic or implement your own inverse kinematics to convert Cartesian velocities to joint velocities.

**Option 1**: Use joint velocity control
- Modify the node to compute IK and publish to `/forward_velocity_controller/commands`

**Option 2**: Use position control with small increments
- Convert twist commands to small position changes
- Use the existing MoveIt action servers in this codebase

## Safety Notes

⚠️ **Important Safety Guidelines:**

1. **Always be ready to press the emergency stop** (Home button or physical E-stop)
2. **Start with low speed multipliers** (use X button to slow down)
3. **Test in simulation first** before using on real hardware
4. **Keep clear of the robot workspace** when motion is enabled
5. **The A button is a deadman switch** - release it to stop motion
6. **Monitor the robot** for unexpected movements or collisions
7. **Ensure proper workspace limits** are configured in MoveIt

## Future Enhancements

- [ ] Implement return-to-home functionality (B button)
- [ ] Add haptic feedback for events
- [ ] Support multiple controller profiles
- [ ] Add joint control mode toggle
- [ ] Integrate with existing mtc_pipeline action servers
- [ ] Add visual feedback in RViz
- [ ] Record and playback motion sequences

## License

MIT

## Contributing

Issues and pull requests welcome!
