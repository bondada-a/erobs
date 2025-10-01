#!/bin/bash

# Build the workspace
colcon build

# Source the setup file
source install/setup.bash

# Run the server with logging (displays in terminal AND saves to file)
ros2 launch mtc_pipeline modular_action_servers.launch.py 2>&1 | tee server.log
