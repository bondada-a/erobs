#!/bin/bash
# Local BSUI launcher for Bluesky/ROS integration
# Adapted from the Docker version for local development

# Colors for output
BOLD=$(tput bold)
UNDERLINE=$(tput smul)
RESET=$(tput sgr0)
BLUE=$(tput setaf 4)
RED=$(tput setaf 1)

# Set up ROS 2 environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 Humble not found at /opt/ros/humble"
    exit 1
fi

# Set up local workspace
WORKSPACE_ROOT="/home/aditya/work/github_ws/erobs"
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
    echo "✓ ROS workspace sourced from $WORKSPACE_ROOT"
else
    echo "Warning: ROS workspace not built yet at $WORKSPACE_ROOT/install"
    echo "You may want to run: cd $WORKSPACE_ROOT && colcon build"
fi

# Set up EPICS environment (optional - only if EPICS is installed)
if [ -d "/home/aditya/EPICS/epics-base" ]; then
    export EPICS_BASE="/home/aditya/EPICS/epics-base"
    export PATH="${EPICS_BASE}/bin/linux-x86_64:${PATH}"
    echo "✓ EPICS base sourced from $EPICS_BASE"
fi

# Set up Conda (optional - only if using conda environment)
if [ -f "/home/aditya/miniconda3/etc/profile.d/conda.sh" ]; then
    source "/home/aditya/miniconda3/etc/profile.d/conda.sh"
    # Optionally activate bluesky environment
    # conda activate bluesky
fi

# Set up Python path for bluesky_ros modules
export PYTHONPATH="$WORKSPACE_ROOT/src:$PYTHONPATH"

# Display version information
cat << EOL

${UNDERLINE}${BOLD}Bluesky/ROS Integration Environment${RESET}

${BOLD}Software Versions:${RESET}

$(python3 -c '
msg = "Not installed"
try:
    import bluesky
    bluesky_version = "v{}".format(bluesky.__version__)
except ImportError:
    bluesky_version = msg
try:
    import ophyd
    ophyd_version = "v{}".format(ophyd.__version__)
except ImportError:
    ophyd_version = msg
try:
    import ophyd_async
    ophyd_async_version = "v{}".format(ophyd_async.__version__)
except ImportError:
    ophyd_async_version = msg
try:
    import tiled
    tiled_version = "v{}".format(tiled.__version__)
except ImportError:
    tiled_version = msg
try:
    import databroker
    databroker_version = "v{}".format(databroker.__version__)
except ImportError:
    databroker_version = msg
try:
    import rclpy
    rclpy_version = "ROS 2 Humble"
except ImportError:
    rclpy_version = msg

print("    - Bluesky      : {}".format(bluesky_version))
print("    - Ophyd-Async  : {}".format(ophyd_async_version))
print("    - Ophyd        : {}".format(ophyd_version))
print("    - Tiled        : {}".format(tiled_version))
print("    - Databroker   : {}".format(databroker_version))
print("    - ROS 2        : {}".format(rclpy_version))
')

${UNDERLINE}${BOLD}Documentation Links:${RESET}

    - ${BLUE}https://blueskyproject.io/bluesky/main/index.html${RESET}
    - ${BLUE}https://blueskyproject.io/tiled/${RESET}
    - ${BLUE}https://docs.ros.org/en/humble/index.html${RESET}

${BOLD}Environment:${RESET}
    - WORKSPACE: ${WORKSPACE_ROOT}
    - PYTHONPATH: ${PYTHONPATH}

${BOLD}Available Example Scripts:${RESET}
    - simple_mtc_bluesky.py    : Simple MTC task execution with Bluesky
    - mtc_bluesky_example.py   : Full Ophyd device integration example

${UNDERLINE}${BOLD}Quick Start:${RESET}

1. Run a simple MTC task:
   ${BLUE}python3 src/bluesky_ros/simple_mtc_bluesky.py task_sequences/complete_sequence.json${RESET}

2. Start interactive IPython session:
   ${BLUE}ipython${RESET}

   Then in IPython:
   >>> import rclpy
   >>> from bluesky import RunEngine
   >>> from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
   >>> rclpy.init()
   >>> RE = RunEngine({})
   >>> # Your Bluesky plans here

3. For interactive exploration with IPython profile:
   ${BLUE}ipython --profile=default${RESET}

EOL

# Parse command line arguments
if [ "$1" == "--ipython" ] || [ "$1" == "-i" ]; then
    # Start IPython interactive session
    echo "${BOLD}Starting IPython...${RESET}"
    ipython "${@:2}"
elif [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    echo "Usage: $0 [--ipython|-i] [args]"
    echo ""
    echo "Options:"
    echo "  --ipython, -i    Start IPython interactive session"
    echo "  --help, -h       Show this help message"
    echo ""
    echo "Without options, shows environment information"
elif [ -n "$1" ]; then
    # Execute the provided command
    echo "${BOLD}Executing: $@${RESET}"
    exec "$@"
else
    # Just show the environment info (already displayed above)
    echo "${BOLD}Environment ready!${RESET}"
    echo "Run '$0 --ipython' to start an interactive session"
fi
