# Quick Start: Joint Velocity Control (NO SERVO NEEDED!)

This is the **working** version that bypasses the broken MoveIt Servo.

## What You Get

- **Direct joint control** using your 8BitDo Ultimate 2C controller
- **No MoveIt Servo required** - works immediately
- **Low latency** - direct velocity commands to the robot
- **Same safety features** - deadman switch, speed control, emergency stop

## Launch It

### Step 1: Start Your Robot (Terminal 1)

```bash
cd ~/work/github_ws/erobs
source install/setup.bash
ros2 launch ur_zivid_hande_moveit_config robot_bringup.launch.py
```

Wait for the robot to fully initialize.

### Step 2: Start Teleop (Terminal 2)

```bash
cd ~/work/github_ws/erobs
source install/setup.bash
ros2 launch ur5e_teleop joint_teleop.launch.py
```

You should see:
```
=== CONTROLLER MAPPING ===
A button: Enable/Disable motion
Left stick L/R: Shoulder pan (joint 0)
Left stick U/D: Shoulder lift (joint 1)
...
```

## Controller Mapping

| Input | Controls | Joint Name |
|-------|----------|------------|
| **Left Stick Left/Right** | Shoulder Pan | Joint 0 (base rotation) |
| **Left Stick Up/Down** | Shoulder Lift | Joint 1 (shoulder up/down) |
| **Right Stick Up/Down** | Elbow | Joint 2 (elbow bend) |
| **Right Stick Left/Right** | Wrist 1 | Joint 3 (wrist rotation) |
| **LT/RT Triggers** | Wrist 2 | Joint 4 (wrist bend) |
| **D-Pad Left/Right** | Wrist 3 | Joint 5 (wrist roll) |
| **A Button** | **Enable/Disable** | Deadman switch |
| **X Button** | Decrease Speed | Slower movements |
| **Y Button** | Increase Speed | Faster movements |
| **LB (Left Bumper)** | Close Gripper | - |
| **RB (Right Bumper)** | Open Gripper | - |
| **Home Button** | Emergency Stop | Immediately disables motion |

## Usage

1. **Press and hold A** to enable motion
2. **Move the joysticks** to control individual joints
3. **Press X/Y** to adjust speed (starts at 1.0x)
4. **Release A** or press **Home** to stop

## Safety Notes

⚠️ **IMPORTANT:**
- **A button is the deadman switch** - motion only works while enabled
- **Start with low speed** (press X a few times to slow down)
- **Test each joint individually** before combining movements
- **Keep clear of the robot** when motion is enabled
- **Home button** = immediate stop

## Adjusting Speed

Default max joint velocity: **0.5 rad/s**

To change:
```bash
ros2 launch ur5e_teleop joint_teleop.launch.py max_joint_velocity:=0.3
```

Lower values = safer for testing.

## Troubleshooting

### "No joy messages"
- Check controller is on and paired
- Verify: `ls /dev/input/js0`
- Check joy node: `ros2 topic echo /joy`

### "Robot not moving"
1. Is A button pressed? (Check terminal for "ENABLED" message)
2. Is robot in running state? (not protective stop)
3. Check velocity commands: `ros2 topic echo /forward_velocity_controller/commands`

### "Controller feels slow"
- Press Y to increase speed multiplier
- Or launch with higher max_joint_velocity

### "Controller feels too sensitive"
- Press X to decrease speed multiplier
- Or launch with lower max_joint_velocity
- Increase deadzone in config file

## Advantages vs Servo

✅ **Works immediately** - no servo bugs
✅ **Lower latency** - direct control
✅ **Predictable** - simple velocity mapping
✅ **No dependencies** - just joy + robot driver

## Limitations vs Servo

❌ **No Cartesian control** - must think in joints
❌ **No collision checking** - be careful!
❌ **No singularity avoidance** - can reach limits
❌ **Manual coordination** - all 6 joints independent

## What's Next?

Once you're comfortable with joint control, you could:
1. Build MoveIt Servo from source (fixes the parameter bug)
2. Add simple Cartesian mode with Jacobian IK
3. Add joint limit checking in the teleop node
4. Record and playback joint sequences

**But for now, this works and you can control your robot! 🎮🤖**
