#!/bin/bash
# Start beambot (no rosbridge, no rosbag)
# Usage: ./start_robot_nobag.sh [beambot launch args]
# Examples:
#   ./start_robot_nobag.sh
#   ./start_robot_nobag.sh use_fake_hardware:=true
#   ./start_robot_nobag.sh enable_vision:=false

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

# Cleanup on exit. PID is populated as beambot is spawned below; guard
# against re-entry so a second Ctrl-C while wait is unwinding doesn't confuse
# bash's variable-scope stack (pop_var_context warning).
_cleanup_ran=0
cleanup() {
    (( _cleanup_ran )) && return
    _cleanup_ran=1
    echo ""
    echo "Shutting down..."
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

echo ""
echo "=== Robot Ready (no bag) ==="
echo "  beambot:    PID $BEAMBOT_PID"
echo "  Press Ctrl+C to stop"
echo "================="
echo ""

# Wait for beambot to exit
wait -n "$BEAMBOT_PID" 2>/dev/null
