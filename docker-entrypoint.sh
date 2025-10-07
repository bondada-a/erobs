#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Add src to Python path
export PYTHONPATH=/workspace/src:$PYTHONPATH

# Execute command
exec "$@"
