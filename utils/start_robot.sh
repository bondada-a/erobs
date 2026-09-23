#!/bin/bash
# Start beambot (no rosbridge — not using MCP)
# Usage: ./utils/start_robot.sh [beambot launch args]
# Examples:
#   ./utils/start_robot.sh
#   ./utils/start_robot.sh use_mock_hardware:=true enable_vision:=false enable_pipettor:=false
#   ./utils/start_robot.sh enable_vision:=false

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Source ROS2 + workspace
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null || {
    echo "Workspace not built. Run: cd \"$WORKSPACE_DIR\" && colcon build && source install/setup.bash"
    exit 1
}

# Beamline config is the single source of truth for the deployment site.
# We refuse to launch without it — silent CMS-fallback would mask a
# misconfiguration on a different beamline machine.
if [[ -z "${BEAMBOT_BEAMLINE_CONFIG:-}" ]]; then
    echo "ERROR: BEAMBOT_BEAMLINE_CONFIG is not set." >&2
    echo "Export it before launching, e.g.:" >&2
    echo "    export BEAMBOT_BEAMLINE_CONFIG=\"$WORKSPACE_DIR/src/beambot/config/cms_beamline.yaml\"" >&2
    exit 1
fi
if [[ ! -f "$BEAMBOT_BEAMLINE_CONFIG" ]]; then
    echo "ERROR: BEAMBOT_BEAMLINE_CONFIG points at missing file: $BEAMBOT_BEAMLINE_CONFIG" >&2
    exit 1
fi
echo "Beamline config: $BEAMBOT_BEAMLINE_CONFIG"

# Record all topics, including hidden, for experiment data. Background jobs
# start with SIGINT ignored; restore it so Ctrl+C also stops the recorder.
BAG_DIR="$WORKSPACE_DIR/recorded_bags/experiments/$(date +%Y-%m-%d)"
mkdir -p "$BAG_DIR"
BAG_NAME="exp_$(date +%Y-%m-%d_%H-%M-%S)"
echo "Starting rosbag recording: $BAG_DIR/$BAG_NAME"
( trap - INT; exec ros2 bag record \
    --all \
    --include-hidden-topics \
    -o "$BAG_DIR/$BAG_NAME" \
    --max-cache-size 0 ) &
ROSBAG_PID=$!

# Run launch in the foreground so Ctrl+C reaches it directly; launch then
# stops its nodes in order and the orchestrator stops MoveIt.
echo "Starting beambot. Press Ctrl+C to stop all."
ros2 launch beambot beambot_bringup.launch.py "$@" || true

# Ctrl+C already stopped the recorder; stop it if launch exited on its own.
kill -INT "$ROSBAG_PID" 2>/dev/null || true
wait "$ROSBAG_PID" 2>/dev/null || true
