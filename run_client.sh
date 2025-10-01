#!/bin/bash

# Source the setup file
source install/setup.bash

# Run the client with logging (displays in terminal AND saves to file)
ros2 run mtc_pipeline mtc_action_client_example new_test_updated.json 192.168.56.101 2>&1 | tee client.log
