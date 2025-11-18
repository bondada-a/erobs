#!/bin/bash
# Bluesky Interactive Shell Launcher
#
# This script starts an IPython shell with the Bluesky environment
# pre-configured for robot control.

# Change to the bluesky_ros directory
cd "$(dirname "$0")"

# Set robot IP (override with environment variable if needed)
export ROBOT_IP=${ROBOT_IP:-10.68.82.41}

echo "=========================================="
echo "  Starting Bluesky Interactive Shell"
echo "=========================================="
echo ""
echo "Robot IP: $ROBOT_IP"
echo ""
echo "To use a different robot IP:"
echo "  ROBOT_IP=192.168.1.100 ./start_bluesky.sh"
echo ""
echo "Starting IPython..."
echo ""

# Start IPython with startup script
ipython -i bluesky_startup.py

# Note: The -i flag makes IPython interactive after running the script
