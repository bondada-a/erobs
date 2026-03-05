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
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Error: ROS 2 Jazzy not found at /opt/ros/jazzy"
    exit 1
fi

# Set up local workspace (priority: EROBS_WORKSPACE env var, then auto-detect)
if [ -n "$EROBS_WORKSPACE" ]; then
    WORKSPACE_ROOT="$EROBS_WORKSPACE"
    echo "✓ Using EROBS_WORKSPACE: $WORKSPACE_ROOT"
else
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    # Navigate up from src/bluesky_ros/ to workspace root
    WORKSPACE_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
fi

if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
    echo "✓ ROS workspace sourced from $WORKSPACE_ROOT"
else
    echo "Warning: ROS workspace not built yet at $WORKSPACE_ROOT/install"
    echo "You may want to run: cd $WORKSPACE_ROOT && colcon build"
fi

# Set up EPICS environment (optional - only if EPICS is installed)
if [ -d "$HOME/EPICS/epics-base" ]; then
    export EPICS_BASE="$HOME/EPICS/epics-base"
    export PATH="${EPICS_BASE}/bin/linux-x86_64:${PATH}"
    echo "✓ EPICS base sourced from $EPICS_BASE"
fi

# Set up Conda (optional - only if using conda environment)
if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
    # Optionally activate bluesky environment
    # conda activate bluesky
elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/anaconda3/etc/profile.d/conda.sh"
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
    rclpy_version = "ROS 2 Jazzy"
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

${BOLD}Backend:${RESET}
    - beambot (Python MTC implementation)
    - Action server: beambot_execution

${BOLD}Available Example Scripts:${RESET}
    - simple_mtc_bluesky.py    : Simple MTC task execution with Bluesky
    - mtc_bluesky_example.py   : Full Ophyd device integration example

${UNDERLINE}${BOLD}Quick Start:${RESET}

1. Run a simple MTC task:
   ${BLUE}python3 src/bluesky_ros/simple_mtc_bluesky.py src/cms/tasks/complete_sequence.json${RESET}

2. Start interactive IPython session:
   ${BLUE}ipython${RESET}

   Then in IPython:
   >>> import rclpy
   >>> from bluesky import RunEngine
   >>> from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
   >>> rclpy.init()
   >>> RE = RunEngine({})
   >>> robot = MTCExecutionDevice(name="ur5e_robot")
   >>> # Use RE(your_plan(robot, "task.json"))

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
