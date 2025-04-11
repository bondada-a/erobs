#!/bin/bash

# Command 1
gnome-terminal --tab --title="UR Robot Driver" -- bash -c "source install/setup.bash && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=\${UR_TYPE} robot_ip:=\${ROBOT_IP} description_package:=\${DESCRIPTION_PKG} description_file:=\${DESCRIPTION_FILE} launch_rviz:=\${LAUNCH_RVIZ} tool_voltage:=24; exec bash"
sleep 2
# Command 2
gnome-terminal --tab --title="UR MoveIt" -- bash -c "source install/setup.bash && ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true description_package:=\"ur5e_hande_robot_description\" launch_servo:=false description_file:=\"ur_with_hande.xacro\" moveit_config_package:=\"ur5e_hande_moveit_config\" moveit_config_file:=\"ur.srdf\"; exec bash"
sleep 2
# Command 3
gnome-terminal --tab --title="ZED Camera" -- bash -c "source install/setup.bash && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm; exec bash"

sleep 2
# Command 4
gnome-terminal --tab --title="RealSense Camera" -- bash -c "source install/setup.bash && ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true; exec bash"

sleep 2
# Command 5
gnome-terminal --tab --title="ZED Calibration" -- bash -c "source install/setup.bash && ros2 launch drylab_calibration camera_pose_zed.launch.py; exec bash"

sleep 2
# Command 6
gnome-terminal --tab --title="D435i Calibration" -- bash -c "source install/setup.bash && ros2 launch drylab_calibration camera_pose_d435i.launch.py; exec bash"

sleep 2

# gnome-terminal --tab --title="Rviz" -- bash -c "source install/setup.bash && ros2 run rviz2 rviz2 -d \"src/drylab_calibration/rviz_config/two_cam_setup.rviz\"; exec bash"

