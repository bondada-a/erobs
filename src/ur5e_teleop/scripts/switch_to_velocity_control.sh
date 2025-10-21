#!/bin/bash
# Helper script to switch to velocity control mode for teleoperation

echo "Switching to velocity control mode..."
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller

if [ $? -eq 0 ]; then
    echo "✓ Successfully switched to velocity controller"
    echo "✓ Robot is ready for teleop control"
    echo ""
    echo "Now run: ros2 launch ur5e_teleop joint_teleop.launch.py"
else
    echo "✗ Failed to switch controllers"
    echo "Make sure the robot is running first"
fi
