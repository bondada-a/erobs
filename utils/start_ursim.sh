#!/bin/bash

# Start URSim for UR5e robot
echo "Starting URSim for UR5e..."
ros2 run ur_client_library start_ursim.sh -m ur5e
