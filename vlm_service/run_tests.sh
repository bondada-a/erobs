#!/usr/bin/env bash
# Run VLM service tests in isolation from ROS pytest plugins.
set -euo pipefail
cd "$(dirname "$0")"
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
    .venv/bin/python -m pytest tests/ "$@"
