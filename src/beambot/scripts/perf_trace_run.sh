#!/usr/bin/env bash
# Run one traced beambot task end-to-end and print the trace directory.
#
# Usage:
#   scripts/perf_trace_run.sh <task.json> [extra launch args...]
#
# What it does:
#   1. Launches beambot_bringup with enable_tracing:=true in the background.
#   2. Waits for /beambot_execution to appear (orchestrator is ready).
#   3. Runs beambot_client on the task JSON.
#   4. Shuts down the launch cleanly so LTTng flushes + closes the trace.
#   5. Prints the resulting trace directory for analysis.
#
# Analyze the trace:
#   ros2 run tracetools_analysis auto          <trace_dir>   # summary
#   ros2 run tracetools_analysis cb_durations  <trace_dir>   # per-callback
#   ros2 run beambot perf_trace_summarize.py   <trace_dir>   # our custom report
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <task.json> [extra launch args...]" >&2
  exit 2
fi

TASK_JSON="$1"; shift
if [ ! -f "$TASK_JSON" ]; then
  echo "ERROR: task file not found: $TASK_JSON" >&2
  exit 1
fi

SESSION_NAME="${TRACE_SESSION_NAME:-beambot}"
READY_TIMEOUT="${READY_TIMEOUT:-60}"

TRACE_BASE="$HOME/.ros/tracing"
mkdir -p "$TRACE_BASE"
# Snapshot existing session dirs so we can diff after the run.
BEFORE=$(ls -1 "$TRACE_BASE" 2>/dev/null || true)

echo "[perf_trace_run] Launching bringup with tracing enabled..."
# shellcheck disable=SC2086
ros2 launch beambot beambot_bringup.launch.py \
    enable_tracing:=true \
    trace_session_name:="$SESSION_NAME" \
    "$@" &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[perf_trace_run] Shutting down launch (pid $LAUNCH_PID)..."
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    # Give LTTng time to finalize; ros2 launch SIGINT normally propagates.
    for _ in $(seq 1 30); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.5
    done
    kill -KILL "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[perf_trace_run] Waiting up to ${READY_TIMEOUT}s for /beambot_execution..."
deadline=$(( $(date +%s) + READY_TIMEOUT ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  if ros2 action list 2>/dev/null | grep -q "/beambot_execution"; then
    echo "[perf_trace_run] Orchestrator ready."
    break
  fi
  sleep 1
done
if ! ros2 action list 2>/dev/null | grep -q "/beambot_execution"; then
  echo "ERROR: /beambot_execution never appeared within ${READY_TIMEOUT}s" >&2
  exit 1
fi

echo "[perf_trace_run] Running task: $TASK_JSON"
ros2 run beambot beambot_client.py "$TASK_JSON" || TASK_RC=$?
TASK_RC="${TASK_RC:-0}"
echo "[perf_trace_run] Task returned rc=$TASK_RC"

# Shut down via trap, then identify the new trace directory.
cleanup
trap - EXIT

# Give LTTng a moment to finish flushing.
sleep 2

AFTER=$(ls -1 "$TRACE_BASE" 2>/dev/null || true)
NEW_DIRS=$(comm -13 <(echo "$BEFORE" | sort) <(echo "$AFTER" | sort) | grep "^$SESSION_NAME" || true)

if [ -z "$NEW_DIRS" ]; then
  echo "WARNING: no new trace directory detected under $TRACE_BASE" >&2
  exit "$TASK_RC"
fi

# Newest matching session dir
NEWEST=$(echo "$NEW_DIRS" | tail -n 1)
TRACE_DIR="$TRACE_BASE/$NEWEST"

echo ""
echo "=============================================="
echo "Trace directory: $TRACE_DIR"
echo "=============================================="
echo "Analyze with:"
echo "  ros2 run tracetools_analysis auto         $TRACE_DIR"
echo "  ros2 run tracetools_analysis cb_durations $TRACE_DIR"
echo "  ros2 run beambot perf_trace_summarize.py  $TRACE_DIR"

exit "$TASK_RC"
