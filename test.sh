#!/bin/bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

mode=${1:-unit}
if [ "$#" -gt 0 ]; then shift; fi
case "$mode" in
    unit|integration|all|pure) ;;
    *) echo "Usage: $0 [unit|integration|all|pure] [pytest options]" >&2; exit 2 ;;
esac

if [ "$mode" != pure ]; then
    source /opt/ros/jazzy/setup.bash
    if [ -f install/setup.bash ]; then source install/setup.bash; fi
fi
export PYTHONPATH="$PWD/src/beambot:$PWD/src/beambot_gui${PYTHONPATH:+:$PYTHONPATH}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1
export BEAMBOT_BEAMLINE_CONFIG="$PWD/src/beambot/config/cms_drylab.yaml"
export ROS_LOG_DIR="$PWD/log/tests/ros"
mkdir -p "$ROS_LOG_DIR"

if [ "$mode" = pure ]; then
    timeout 180 /usr/bin/python3 -B -m pytest -q -p no:cacheprovider \
        src/beambot/test/test_task_parser.py src/beambot/test/test_batch_planner.py \
        src/beambot/test/test_trajectory_cache.py src/beambot/test/test_algorithms.py \
        src/beambot/test/test_config_paths.py "$@"
elif [ "$mode" = unit ] || [ "$mode" = all ]; then
    timeout 180 /usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$@"
fi

if [ "$mode" = integration ] || [ "$mode" = all ]; then
    for test_file in src/beambot/test/integration/test_*_launch.py; do
        test_name=$(basename "$test_file" .py)
        /usr/bin/python3 -B src/beambot/test/run_isolated.py \
            "$PWD/log/tests/$test_name.xml" \
            --command timeout --signal=INT --kill-after=10 120 /usr/bin/python3 -B -m launch_testing.launch_test \
            "$test_file" --junit-xml "$PWD/log/tests/$test_name.xml"
    done
fi
