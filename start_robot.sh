#!/bin/bash
# Start beambot (no rosbridge — not using MCP)
# Usage: ./start_robot.sh [beambot launch args]
# Examples:
#   ./start_robot.sh
#   ./start_robot.sh use_fake_hardware:=true
#   ./start_robot.sh enable_vision:=false

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 + workspace
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || {
    echo "Workspace not built. Run: colcon build && source install/setup.bash"
    exit 1
}

# Beamline config is the single source of truth for the deployment site.
# We refuse to launch without it — silent CMS-fallback would mask a
# misconfiguration on a different beamline machine.
if [[ -z "${BEAMBOT_BEAMLINE_CONFIG:-}" ]]; then
    echo "ERROR: BEAMBOT_BEAMLINE_CONFIG is not set." >&2
    echo "Export it before launching, e.g.:" >&2
    echo "    export BEAMBOT_BEAMLINE_CONFIG=$SCRIPT_DIR/src/beambot/config/cms_beamline.yaml" >&2
    exit 1
fi
if [[ ! -f "$BEAMBOT_BEAMLINE_CONFIG" ]]; then
    echo "ERROR: BEAMBOT_BEAMLINE_CONFIG points at missing file: $BEAMBOT_BEAMLINE_CONFIG" >&2
    exit 1
fi
echo "Beamline config: $BEAMBOT_BEAMLINE_CONFIG"

# Cleanup on exit. PIDs are populated as children are spawned below; guard
# against re-entry so a second Ctrl-C while wait is unwinding doesn't confuse
# bash's variable-scope stack (pop_var_context warning).
_cleanup_ran=0
cleanup() {
    (( _cleanup_ran )) && return
    _cleanup_ran=1
    echo ""
    echo "Shutting down..."
    [[ -n "$ROSBAG_PID" ]] && kill -INT "$ROSBAG_PID" 2>/dev/null && echo "Stopping rosbag..."
    [[ -n "$BEAMBOT_PID" ]] && kill "$BEAMBOT_PID" 2>/dev/null
    wait 2>/dev/null
    echo "Done."
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Start beambot with any extra args passed to this script
echo "Starting beambot..."
ros2 launch beambot beambot_bringup.launch.py "$@" &
BEAMBOT_PID=$!

# Start rosbag recording for experiment data — all topics, including hidden
BAG_DIR="$SCRIPT_DIR/recorded_bags/experiments/$(date +%Y-%m-%d)"
mkdir -p "$BAG_DIR"
BAG_NAME="exp_$(date +%Y-%m-%d_%H-%M-%S)"
echo "Starting rosbag recording: $BAG_DIR/$BAG_NAME"
ros2 bag record \
    --all \
    --include-hidden-topics \
    -o "$BAG_DIR/$BAG_NAME" \
    --max-cache-size 0 \
    &
ROSBAG_PID=$!

echo ""
echo "=== Robot Ready ==="
echo "  beambot:    PID $BEAMBOT_PID"
echo "  rosbag:     PID $ROSBAG_PID → $BAG_DIR/$BAG_NAME"
echo "  Press Ctrl+C to stop all"
echo "================="
echo ""

# Wait for beambot to exit
wait -n "$BEAMBOT_PID" 2>/dev/null
