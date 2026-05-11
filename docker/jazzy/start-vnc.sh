#!/bin/bash

# Start Xvfb (virtual framebuffer)
echo "Starting Xvfb on display :1..."
Xvfb :1 -screen 0 1920x1080x24 &
XVFB_PID=$!

# Wait for Xvfb to initialize
sleep 2

# Start x11vnc server
echo "Starting x11vnc on port 5901..."
x11vnc -display :1 -rfbport 5901 -forever -shared -nopw -bg

echo "======================================"
echo "VNC server ready on port 5901"
echo "Connect with: vncviewer localhost:5901"
echo "Xvfb PID: $XVFB_PID"
echo "======================================"
