#!/bin/bash
# Test script to capture and save Zivid image

echo "==== Zivid Image Capture Test ===="
echo

# Source workspace
source install/setup.bash

# Trigger capture
echo "1. Triggering camera capture..."
ros2 service call /capture_2d std_srvs/srv/Trigger "{}"

if [ $? -eq 0 ]; then
    echo "   ✓ Capture triggered successfully"
else
    echo "   ✗ Capture failed"
    exit 1
fi

echo
echo "2. Waiting for image to be published..."
sleep 1

# Check if image topic has data
echo "3. Checking image properties..."
timeout 2 ros2 topic info /color/image_color --verbose

echo
echo "4. Saving image to file..."
# Save one image
timeout 5 ros2 run image_view image_saver --ros-args \
    --remap image:=/color/image_color \
    --param filename_format:="zivid_test_%04d.jpg" 2>&1 | head -10 &

SAVER_PID=$!

# Trigger another capture so image_saver catches it
sleep 1
echo "5. Triggering another capture for image_saver..."
ros2 service call /capture_2d std_srvs/srv/Trigger "{}"

# Wait for image_saver
sleep 2
kill $SAVER_PID 2>/dev/null

echo
echo "==== Image saved to current directory ===="
ls -lh zivid_test_*.jpg 2>/dev/null || echo "No image file found - may need to run longer"
