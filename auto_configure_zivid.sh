#!/bin/bash
# Wrapper script to run Zivid auto-configuration

echo "======================================"
echo "  Zivid Auto-Configuration"
echo "======================================"
echo

# Check if camera is running
if ! ros2 node list 2>/dev/null | grep -q zivid_camera; then
    echo "ERROR: Zivid camera node not running"
    echo "Start camera first: ros2 run zivid_camera zivid_camera"
    exit 1
fi

# Source workspace
source install/setup.bash

# Run Python script
python3 auto_configure_zivid.py "$@"
EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    echo "Would you like to apply these settings now? (y/n)"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        SETTINGS_FILE="$(pwd)/src/vision/zivid-ros/cam_settings_2d_auto.yml"
        echo "Applying settings..."
        ros2 param set /zivid_camera settings_2d_file_path "$SETTINGS_FILE"

        echo
        echo "Testing capture with new settings..."
        ros2 service call /capture_2d std_srvs/srv/Trigger "{}"

        echo
        echo "✓ Settings applied and tested!"
    fi
fi

exit $EXIT_CODE
