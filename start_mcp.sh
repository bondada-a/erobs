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

# Cleanup on exit — defined later after all PIDs are known
trap 'cleanup' EXIT INT TERM

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
# Tee output to log file so erobs-mcp-server can read it for get_recent_logs
BEAMBOT_LOG="/tmp/beambot_launch.log"
> "$BEAMBOT_LOG"  # Truncate on start
echo "Starting beambot..."
ros2 launch beambot beambot_bringup.launch.py "$@" 2>&1 | tee "$BEAMBOT_LOG" &
BEAMBOT_PID=$!

# Start rosbag recording for experiment data
BAG_DIR="$SCRIPT_DIR/recorded_bags/testing_2026-04-02"
mkdir -p "$BAG_DIR"
BAG_NAME="experiment_$(date +%Y-%m-%d_%H-%M-%S)"
echo "Starting rosbag recording: $BAG_DIR/$BAG_NAME"
ros2 bag record \
    /joint_states \
    /tf \
    /tf_static \
    /color/image_color \
    /points/xyzrgba \
    /zed/zed_node/rgb/color/rect/image \
    /zed/zed_node/point_cloud/cloud_registered \
    /beambot/current_gripper \
    /beambot/execution_state \
    /object_detection_status \
    /rosout \
    /beambot_execution/_action/send_goal \
    /beambot_execution/_action/get_result \
    /beambot_vision_moveto/_action/send_goal \
    /beambot_vision_moveto/_action/get_result \
    /beambot_vision_moveto/_action/feedback \
    /beambot_moveto/_action/send_goal \
    /beambot_moveto/_action/get_result \
    /beambot_pick_sample/_action/send_goal \
    /beambot_pick_sample/_action/get_result \
    /beambot_pick_sample/_action/feedback \
    /beambot_place_sample/_action/send_goal \
    /beambot_place_sample/_action/get_result \
    /beambot_place_sample/_action/feedback \
    -o "$BAG_DIR/$BAG_NAME" \
    --max-cache-size 0 \
    --include-hidden-topics \
    &
ROSBAG_PID=$!

# Update cleanup to also kill rosbag
cleanup() {
    echo ""
    echo "Shutting down..."
    [[ -n "$ROSBAG_PID" ]] && kill -INT "$ROSBAG_PID" 2>/dev/null && echo "Stopping rosbag..."
    [[ -n "$ROSBRIDGE_PID" ]] && kill "$ROSBRIDGE_PID" 2>/dev/null
    [[ -n "$BEAMBOT_PID" ]] && kill "$BEAMBOT_PID" 2>/dev/null
    wait 2>/dev/null
    echo "Done."
}

echo ""
echo "=== MCP Ready ==="
echo "  rosbridge:  PID $ROSBRIDGE_PID (port 9090)"
echo "  beambot:    PID $BEAMBOT_PID"
echo "  rosbag:     PID $ROSBAG_PID → $BAG_DIR/$BAG_NAME"
echo "  Press Ctrl+C to stop all"
echo "================="
echo ""

# Wait for either to exit
wait -n "$ROSBRIDGE_PID" "$BEAMBOT_PID" 2>/dev/null
