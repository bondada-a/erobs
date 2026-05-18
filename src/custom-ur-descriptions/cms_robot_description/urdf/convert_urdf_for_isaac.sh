#!/bin/bash
# Convert URDF package:// URIs to absolute paths for Isaac Sim
#
# Usage: ./convert_urdf_for_isaac.sh input.urdf output.urdf
#
# Isaac Sim's URDF importer cannot resolve package:// URIs because it
# doesn't have access to ROS_PACKAGE_PATH. This script converts them
# to absolute filesystem paths.

set -e

if [ $# -ne 2 ]; then
    echo "Usage: $0 <input.urdf> <output.urdf>"
    echo "Example: $0 ur_with_zivid_hande.urdf ur_with_zivid_hande_isaac.urdf"
    exit 1
fi

INPUT="$1"
OUTPUT="$2"

if [ ! -f "$INPUT" ]; then
    echo "Error: Input file not found: $INPUT"
    exit 1
fi

# Package mappings - add more as needed
sed -e 's|package://ur_description|/opt/ros/humble/share/ur_description|g' \
    -e 's|package://zivid_description|/home/aditya/work/github_ws/experimental/src/vision/zivid-ros/zivid_description|g' \
    -e 's|package://cms_robot_description|/home/aditya/work/github_ws/experimental/src/custom-ur-descriptions/cms_robot_description|g' \
    -e 's|package://robotiq_hande_description|/home/aditya/work/github_ws/experimental/src/end_effectors/robotiq_hande_description|g' \
    -e 's|package://epick_description|/home/aditya/work/github_ws/experimental/src/end_effectors/ros2_epick_gripper/epick_description|g' \
    -e 's|package://pipette_description|/home/aditya/work/github_ws/experimental/src/end_effectors/pipettor/pipette_description|g' \
    "$INPUT" > "$OUTPUT"

# Check for any remaining package:// URIs
REMAINING=$(grep -c "package://" "$OUTPUT" 2>/dev/null || echo "0")

echo "Converted: $INPUT -> $OUTPUT"
if [ "$REMAINING" -gt 0 ]; then
    echo "WARNING: $REMAINING unresolved package:// URIs remain:"
    grep -o 'package://[^/]*' "$OUTPUT" | sort -u
    echo "Add these packages to the script's sed commands."
else
    echo "All package:// URIs converted successfully!"
fi
