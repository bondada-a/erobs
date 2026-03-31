# Minimal working example for running BSUI on a beamline srv
FROM osrf/ros:jazzy-desktop-full
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget \
  ros-${ROS_DISTRO}-rclpy \
  python3-colcon-common-extensions \
  ros-dev-tools

# Pin numpy<2 for cv_bridge compatibility (cv_bridge compiled against NumPy 1.x)
RUN pip3 install 'numpy<2' rosdep && \
  rosdep update

# This is an exit hatch for testing when conda envs not avail
RUN pip3 install \
  bluesky \
  ophyd \
  nslsii \
  ipython

# Install Epics (before CACHEBUST so it stays cached)
RUN mkdir $HOME/EPICS && cd $HOME/EPICS && \
  git clone --recursive https://github.com/epics-base/epics-base.git && \
  cd epics-base && \
  make

ENV EPICS_BASE=/root/EPICS/epics-base
# Set host arch manually
ENV PATH=${EPICS_BASE}/bin/linux-x86_64:${PATH}

# Add our custom bsui
COPY docker/bsui/bsui.bash /bin/bsui
RUN chmod +x /bin/bsui

RUN apt-get update && apt-get install -y fontconfig
RUN mkdir /root/.ipython

RUN mkdir /root/ws -p
WORKDIR /root/ws

# Cache-bust arg: use --build-arg CACHEBUST=$(date +%s) to force fresh clone
ARG CACHEBUST=1
RUN git clone https://github.com/bondada-a/erobs.git -b jazzy_dev

WORKDIR /root/ws/erobs/
# Import all repos including vision and end_effectors
RUN /bin/bash -c "set -e && \
  vcs import < src/ros2.repos src && \
  vcs import < src/end_effectors/end_effectors.repos src/end_effectors && \
  vcs import < src/vision/vision.repos src/vision && \
  apt-get update && \
  rosdep update && \
  rosdep install --from-paths src --ignore-src -y --skip-keys 'moveit_studio_behavior_interface zivid_description pytest ament_python'"

WORKDIR /root/ws/
# Build interface packages first so other packages can find them
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && cd erobs && colcon build --packages-select zivid_interfaces pipette_driver"
# Build remaining packages (skip zivid_camera since no Zivid SDK in this container)
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && cd erobs && source install/setup.bash && colcon build --packages-skip zed_components zed_ros2 zed_wrapper epick_moveit_studio zivid_camera zivid_samples"

# Install Conda AFTER ROS builds to avoid Python environment conflicts
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh && \
  chmod +x miniconda.sh && \
  ./miniconda.sh -b -p /opt/conda && \
  rm miniconda.sh

ENV PATH="/opt/conda/bin:$PATH"

# Clean up
# TODO (maffettone): this image is rather large, there may be some source and build files we can clean up here. 
RUN apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*

# Bluesky environment variables
ENV BS_ENV="/nsls2/conda/envs/2025-2.2-py312-tiled"
ENV BS_PROFILE="collection"

# Start a shell session with BSUI when we run the container
CMD ["/bin/bash", "-c", ".  /opt/ros/${ROS_DISTRO}/setup.bash &&  . /root/ws/erobs/install/setup.bash && bsui"]
