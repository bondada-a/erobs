#!/bin/bash

# Script to kill zombie ROS nodes while preserving ursim nodes
# This script identifies and kills ROS processes that are not ursim-related

echo "Searching for zombie ROS nodes..."

# Get all ROS processes, excluding ursim-related ones
# We filter out processes that contain 'ursim' or 'start_ursim' in their command line
zombie_pids=$(ps -ef | grep ros | grep -v grep | grep -v ursim | grep -v start_ursim | awk '{print $2}')

if [ -z "$zombie_pids" ]; then
    echo "No zombie ROS nodes found (excluding ursim nodes)."
    exit 0
fi

echo "Found zombie ROS nodes with PIDs:"
echo "$zombie_pids"

# Show what processes will be killed
echo ""
echo "Processes that will be killed:"
ps -ef | grep ros | grep -v grep | grep -v ursim | grep -v start_ursim

echo ""
echo "Ursim processes that will be preserved:"
ps -ef | grep ros | grep -v grep | grep -E "(ursim|start_ursim)"

echo ""
read -p "Do you want to kill these zombie nodes? (y/N): " -n 1 -r
echo

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Killing zombie ROS nodes..."
    echo "$zombie_pids" | xargs -r kill -9
    echo "Zombie ROS nodes killed successfully."
else
    echo "Operation cancelled."
fi
