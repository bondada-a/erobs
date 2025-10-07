#!/bin/bash
# WARNING : This file is managed by ansible scripts.
# Any changes to this are periodically overwritten.

. /opt/conda/etc/profile.d/conda.sh

# tack on additional PYTHONPATH information
if [ ! -z "$BS_PYTHONPATH" ]; then
    if [ ! -z "$PYTHONPATH" ]; then
        export PYTHONPATH=$PYTHONPATH:$BS_PYTHONPATH
    else
        export PYTHONPATH=$BS_PYTHONPATH
    fi
fi

# Set up the path for "new" overlay only if PYTHONUSERBASE not otherwise
# set.
if [ -z "$PYTHONUSERBASE" ]; then
    export PYTHONUSERBASE=$BS_PYTHONUSERBASE_PATH/`basename $BS_ENV`
fi


# If the above has not defined BS_AN_ENV and BS_AN_PROFILE we will error out
# violently on `conda activate` below.
conda_cmd="conda activate $BS_ENV"
$conda_cmd || exit 1

# Conditionally invoke LD_PRELOAD workaround for 2020-2 profiles.
if [[ "${BS_ENV}" == *"2020-2"* ]]; then
    echo "Adding LD_PRELOAD"
    export LD_PRELOAD=/opt/conda_envs/${BS_ENV}/lib/libgomp.so
fi

# setup the command we will use to start IPython below
ipython_cmd="ipython --profile=$BS_PROFILE"


if [[ "$BS_PROFILE_DIR" ]]; then
    ipython_cmd="$ipython_cmd --ipython-dir=$BS_PROFILE_DIR"
fi


args=$(python -c 'import sys; print(" ".join([x if " " not in x else repr(x) for x in sys.argv[1:]]))' "$@")

cat << EOL

$(tput smul; tput bold)Versions of DSSI software:$(tput sgr0)

$(python -c '\
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

print("    - Bluesky      : {}".format(bluesky_version))
print("    - Ophyd-Async  : {}".format(ophyd_async_version))
print("    - Ophyd        : {}".format(ophyd_version))
print("    - Tiled        : {}".format(tiled_version))
print("    - Databroker   : {}".format(databroker_version))
')

$(tput smul; tput bold)Links to Bluesky and Tiled documentation:$(tput sgr0)

    - $(tput setaf 4)https://blueskyproject.io/bluesky/main/index.html$(tput sgr0)
    - $(tput setaf 4)https://blueskyproject.io/tiled/$(tput sgr0)

If you need help, feel free to reach out to DSSI in the program's support SLACK channel

    $(tput bold; tput setaf 1)https://nsls2.slack.com$(tput sgr0)

or, register with the bluesky mattermost server to discuss with other bluesky users and maintainers.

    $(tput bold; tput setaf 1)https://blueskyproject.io/mattermost/$(tput sgr0)

$(tput bold)bsui$(tput sgr0) is running these commands now to start an interactive computing environment for data acquisition:
    $ ${conda_cmd}
    $ PYTHONPATH=${PYTHONPATH}
    $ PYTHONUSERBASE=${PYTHONUSERBASE}
    $ ${ipython_cmd} ${args}

EOL

# fix permissions on XDG runtime directory
if [[ -n "$XDG_RUNTIME_DIR" ]] && [[ -d "$XDG_RUNTIME_DIR" ]]; then
    chmod g-rwx "$XDG_RUNTIME_DIR"
fi

# hide X11 session manager from Qt
env -u SESSION_MANAGER $ipython_cmd "$@"