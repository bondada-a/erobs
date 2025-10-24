# Quick Start Guide: 8BitDo Controller with UR5e

Get your controller working with the UR5e in 5 minutes!

## 1. Install Dependencies

```bash
sudo apt install ros-humble-joy ros-humble-moveit-servo
```

## 2. Connect Controller

- **Plug in the USB dongle** (or connect via USB-C cable)
- **Turn on the controller** (hold Home button)
- **Set to X-input mode** (hold Home + X until LED flashes)

## 3. Verify Controller

```bash
# Check device exists
ls /dev/input/js0

# Test controller (optional)
ros2 run joy joy_node
# In another terminal: ros2 topic echo /joy
```

## 4. Build the Package

```bash
cd ~/work/github_ws/erobs
colcon build --packages-select ur5e_teleop
source install/setup.bash
```

## 5. Launch

You need **3 terminals**:

**Terminal 1** - Start the robot and MoveIt:
```bash
source ~/work/github_ws/erobs/install/setup.bash
# Use your existing MoveIt launch file, for example:
ros2 launch ur5e_moveit_configs ur_control.launch.py
```

**Terminal 2** - Start MoveIt Servo:
```bash
source ~/work/github_ws/erobs/install/setup.bash
ros2 launch ur5e_teleop servo.launch.py
```

**Terminal 3** - Start the teleop node:
```bash
source ~/work/github_ws/erobs/install/setup.bash
ros2 launch ur5e_teleop teleop.launch.py
```

## 6. Control the Robot

1. **Press A button** to enable motion (you'll see "Motion control ENABLED" in the terminal)
2. **Use left joystick** to move the robot end-effector forward/back and left/right
3. **Use triggers** (LT/RT) to move up and down
4. **Use right joystick** to rotate
5. **Press A again** to disable motion

**Speed control:**
- Press **X** to slow down
- Press **Y** to speed up

**Gripper:**
- **LB** (left bumper) to close
- **RB** (right bumper) to open

**Emergency stop:**
- Press **Home button** to immediately disable motion

---

## Advanced: Customizing Servo Behavior

The servo configuration is located at `config/servo/ur5e_servo.yaml`. You can adjust:

- **Speed limits**: Modify `scale.linear` and `scale.rotational` values
- **Collision checking**: Set `check_collisions: false` for testing (not recommended for real robot)
- **Singularity handling**: Adjust threshold values if robot stops near singularities
- **Controller output**: Change `command_out_topic` to match your active controller

After editing, rebuild the package:
```bash
colcon build --packages-select ur5e_teleop && source install/setup.bash
```

---

## Troubleshooting

### "Controller not found"
- Check `ls /dev/input/js0` shows the device
- Try different USB port
- Ensure controller is on and paired

### "Robot doesn't move when I press A"
- Check you see "Motion control ENABLED" in the terminal
- Verify MoveIt Servo is running: `ros2 topic list | grep servo`
- Make sure robot is not in protective stop

### "Wrong buttons do things"
- Ensure controller is in **X-input mode** (Home + X)
- If still wrong, check button indices with `ros2 topic echo /joy`

### "Gripper doesn't work"
- Verify gripper action server is running: `ros2 action list | grep gripper`
- Check your MoveIt config includes the gripper

---

## What Next?

- Read the full README.md for detailed documentation
- Adjust speed and sensitivity in `config/8bitdo_ultimate_2c.yaml`
- Test different controller modes and configurations
- Consider integrating with the existing mtc_pipeline action servers

**Have fun controlling your robot! 🎮🤖**
