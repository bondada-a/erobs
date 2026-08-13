#!/bin/bash
# Start rosbridge + beambot for MCP access — WITHOUT the experiment rosbag.
#
# This is start_mcp.sh minus the persistent `ros2 bag record`. Use it when you
# want to record bags yourself — e.g. survey mapping, where
# scripts/survey_mapping/run_survey.py spawns its OWN bag of just the survey
# topics. Running the start_mcp.sh recorder at the same time would double-record
# the ~5M-point Zivid clouds into the big experiment bag (wasteful, and it muddies
# which clouds belong to the survey). Everything else — rosbridge + beambot
# bringup — is identical to start_mcp.sh.
#
# Usage: ./start_mcp_nobag.sh [beambot launch args]
# Examples:
#   ./start_mcp_nobag.sh
#   ./start_mcp_nobag.sh use_fake_hardware:=true
#   ./start_mcp_nobag.sh enable_vision:=false

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
    [[ -n "$ROSBRIDGE_PID" ]] && kill "$ROSBRIDGE_PID" 2>/dev/null
    [[ -n "$BEAMBOT_PID" ]] && kill "$BEAMBOT_PID" 2>/dev/null
    wait 2>/dev/null
    echo "Done."
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

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

# NOTE: no `ros2 bag record` here (the difference from start_mcp.sh).
# Record bags yourself — e.g. run_survey.py spawns its own survey bag.

echo ""
echo "=== MCP Ready (no experiment bag) ==="
echo "  rosbridge:  PID $ROSBRIDGE_PID (port 9090)"
echo "  beambot:    PID $BEAMBOT_PID"
echo "  rosbag:     NOT started (use run_survey.py / ros2 bag record yourself)"
echo "  Press Ctrl+C to stop all"
echo "====================================="
echo ""

# Wait for either to exit
wait -n "$ROSBRIDGE_PID" "$BEAMBOT_PID" 2>/dev/null
