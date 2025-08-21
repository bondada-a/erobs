#!/bin/bash
set -e

echo "Step 1: Building the project..."
colcon build --packages-skip ur_ikfast apriltag_ros

echo "Step 2: Generating URDF file..."
source install/setup.bash
PKG_PREFIX=$(ros2 pkg prefix ur5e_hande_robot_description)
ros2 run xacro xacro ${PKG_PREFIX}/share/ur5e_hande_robot_description/urdf/ur_with_zivid_epick.xacro ur_type:=ur5e name:=ur initial_positions_file:=${PKG_PREFIX}/share/ur5e_hande_robot_description/config/initial_positions.yaml > ur_with_zivid_epick.urdf

echo "Step 3: Moving URDF file..."
mv ur_with_zivid_epick.urdf src/ur5e_hande_robot_description/urdf/

echo "Step 4: Rebuilding the project..."
colcon build --packages-skip ur_ikfast apriltag_ros

echo "Step 5: Launching MoveIt Setup Assistant..."
source install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py &
