#!/usr/bin/env bash
# Convenience launcher for the VLM service.
# Usage:
#   ./launch.sh [backend] [port]
#   ./launch.sh stub
#   ./launch.sh molmoact 8765
#   ./launch.sh robobrain25
set -euo pipefail

BACKEND="${1:-stub}"
PORT="${2:-8765}"
HOST="${VLM_HOST:-0.0.0.0}"

cd "$(dirname "$0")/.."

# Activate venv if present.
if [ -d "vlm_service/.venv" ]; then
    # shellcheck disable=SC1091
    source vlm_service/.venv/bin/activate
fi

exec python -m vlm_service.server --backend "$BACKEND" --host "$HOST" --port "$PORT"
