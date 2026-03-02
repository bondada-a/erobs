#!/bin/bash
# Start rosbridge + beambot for MCP access
# Usage: ./start_mcp.sh [beambot launch args]
# Examples:
#   ./start_mcp.sh
#   ./start_mcp.sh use_fake_hardware:=true
#   ./start_mcp.sh enable_vision:=false

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 + workspace
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || {
    echo "Workspace not built. Run: colcon build && source install/setup.bash"
    exit 1
}

# Cleanup on exit — kill both processes
cleanup() {
    echo ""
    echo "Shutting down..."
    [[ -n "$ROSBRIDGE_PID" ]] && kill "$ROSBRIDGE_PID" 2>/dev/null
    [[ -n "$BEAMBOT_PID" ]] && kill "$BEAMBOT_PID" 2>/dev/null
    wait 2>/dev/null
    echo "Done."
}
trap cleanup EXIT INT TERM

# Start rosbridge in background
echo "Starting rosbridge on port 9090..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

# Wait for rosbridge to be ready
echo "Waiting for rosbridge..."
for i in $(seq 1 15); do
    if timeout 1 bash -c "echo >/dev/tcp/127.0.0.1/9090" 2>/dev/null; then
        echo "rosbridge ready."
        break
    fi
    if ! kill -0 "$ROSBRIDGE_PID" 2>/dev/null; then
        echo "ERROR: rosbridge failed to start."
        exit 1
    fi
    sleep 1
done

# Start beambot with any extra args passed to this script
echo "Starting beambot..."
ros2 launch beambot beambot_bringup.launch.py "$@" &
BEAMBOT_PID=$!

echo ""
echo "=== MCP Ready ==="
echo "  rosbridge:  PID $ROSBRIDGE_PID (port 9090)"
echo "  beambot:    PID $BEAMBOT_PID"
echo "  Press Ctrl+C to stop both"
echo "================="
echo ""

# Wait for either to exit
wait -n "$ROSBRIDGE_PID" "$BEAMBOT_PID" 2>/dev/null
