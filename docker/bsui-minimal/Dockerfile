# beambot_bsui_minimal - Lightweight ROS2 Client for Bluesky Integration
#   - rclpy (ROS2 Python client library)
#   - beambot_interfaces (action message definitions only)
#   - bluesky_ros (Ophyd device wrapper)
#   - cms/tasks (pre-saved task JSONs)
#   - bluesky, ophyd
#   - EPICS base
#   - Miniconda 

FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rclpy \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    python3-pip \
    python3-setuptools \
    fontconfig \
    && rm -rf /var/lib/apt/lists/*

# EPICS installation
RUN mkdir -p $HOME/EPICS && cd $HOME/EPICS && \
    git clone --recursive https://github.com/epics-base/epics-base.git && \
    cd epics-base && \
    make

ENV EPICS_BASE=/root/EPICS/epics-base
ENV PATH=${EPICS_BASE}/bin/linux-x86_64:${PATH}

RUN pip3 install --no-cache-dir \
    'numpy<2' \
    bluesky \
    ophyd \
    nslsii \
    ipython


WORKDIR /ros2_ws/src

COPY src/beambot_interfaces ./beambot_interfaces

WORKDIR /ros2_ws

# Initialize rosdep and build
RUN /bin/bash -c "\
    rosdep init || true && \
    rosdep update && \
    source /opt/ros/humble/setup.bash && \
    colcon build --packages-select beambot_interfaces"

COPY src/bluesky_ros /ros2_ws/src/bluesky_ros

# Copy pre-saved task JSONs (CMS beamline tasks)
COPY src/cms /ros2_ws/src/cms

ENV PYTHONPATH="/ros2_ws/src"
ENV EROBS_TASKS="/ros2_ws/src/cms/tasks"

#Miniconda installation
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    chmod +x /tmp/miniconda.sh && \
    /tmp/miniconda.sh -b -p /opt/conda && \
    rm /tmp/miniconda.sh

ENV PATH="/opt/conda/bin:$PATH"

# Copy bsui script
COPY docker/bsui/bsui.bash /bin/bsui
RUN chmod +x /bin/bsui

# Create ipython directory
RUN mkdir -p /root/.ipython

# Bluesky environment variables (can be overridden at runtime)
ENV BS_ENV="/nsls2/conda/envs/2025-2.2-py310-tiled"
ENV BS_PROFILE="collection"

WORKDIR /ros2_ws

RUN apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && bsui"]


LABEL maintainer="abondada@bnl.gov"
LABEL description="Minimal Bluesky-ROS2 client container with EPICS and Conda"
LABEL version="1.0"
LABEL ros.distro="humble"
