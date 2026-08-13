FROM osrf/ros:jazzy-desktop-full

# Set display for RViz/GUI tools (VNC/GUI — disabled while VNC is off)
# ENV DISPLAY=:1

# Fix APT sandboxing issues for container environments
RUN echo 'APT::Sandbox::User "root";' > /etc/apt/apt.conf.d/99sandboxdisable

# Install system dependencies.
# ROS packages (rclpy, ur, moveit, MTC, rosbridge) are NOT listed here —
# they are declared in each package.xml and installed by the rosdep call
# below, keeping the workspace's package.xml the single source of truth.
# Only non-rosdep-resolvable system libs are pinned here.
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    curl \
    build-essential \
    cmake \
    git \
    nano \
    net-tools \
    iputils-ping \
    netcat-openbsd \
    iproute2 \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-vcstool \
    gnupg2 \
    ca-certificates \
    socat \
    # OpenCL dependencies
    ocl-icd-libopencl1 \
    opencl-headers \
    ocl-icd-opencl-dev \
    clinfo \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
# VNC deps (x11vnc, xvfb) removed from the install list above — VNC is
# currently disabled. Re-add them to the apt list to bring VNC back.

# Pin numpy<2 for cv_bridge / tf_transformations compatibility (compiled against NumPy 1.x)
RUN pip3 install --break-system-packages 'numpy<2' rosdep && \
    rosdep update

# Install Zivid SDK 2.17.2 for Ubuntu 24.04 (Noble)
RUN mkdir -p /tmp/zivid && cd /tmp/zivid && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u24/amd64/zivid_2.17.2+440b2367-1_amd64.deb && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u24/amd64/zivid-studio_2.17.2+440b2367-1_amd64.deb && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u24/amd64/zivid-tools_2.17.2+440b2367-1_amd64.deb && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u24/amd64/zivid-genicam_2.17.2+440b2367-1_amd64.deb && \
    apt-get update && apt-get install -y /tmp/zivid/*.deb && \
    rm -rf /tmp/zivid && \
    apt-get clean -y && rm -rf /var/lib/apt/lists/*

# Setup NVIDIA OpenCL ICD for GPU mode
# RUN mkdir -p /etc/OpenCL/vendors && \
#     echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

# Build EPICS base (before the clone so it stays cached across source changes)
RUN mkdir $HOME/EPICS && cd $HOME/EPICS && \
    git clone --recursive https://github.com/epics-base/epics-base.git && \
    cd epics-base && \
    make

ENV EPICS_BASE=/root/EPICS/epics-base
# Host arch set manually
ENV PATH=${EPICS_BASE}/bin/linux-x86_64:${PATH}
ENV LD_LIBRARY_PATH=${EPICS_BASE}/lib/linux-x86_64:${LD_LIBRARY_PATH}

RUN mkdir -p /root/ws
WORKDIR /root/ws

# Copy and setup VNC startup script
# COPY docker/jazzy/start-vnc.sh /root/start-vnc.sh
# RUN chmod +x /root/start-vnc.sh

# Clone erobs (epics_bridge branch) and import all repos
ARG CACHEBUST=1
RUN test -n "$CACHEBUST" && git clone https://github.com/bondada-a/erobs.git -b epics_bridge

WORKDIR /root/ws/erobs/
RUN /bin/bash -c "set -e && \
    vcs import < src/epics.repos src && \
    vcs import < src/end_effectors/end_effectors.repos src/end_effectors && \
    vcs import < src/vision/vision.repos src/vision && \
    rm -rf src/vision/zed-ros2-wrapper"

# Install ROS dependencies via rosdep
RUN /bin/bash -c "apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y \
      --skip-keys 'moveit_studio_behavior_interface pytest ament_python pipette_driver zivid_interfaces zivid_description' && \
    apt-get clean -y && rm -rf /var/lib/apt/lists/*"

# Build interface packages first (other packages depend on these)
WORKDIR /root/ws/
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd erobs && \
    colcon build --packages-select zivid_interfaces pipette_driver"

# Build remaining packages (skip ZED and MoveIt Studio)
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd erobs && \
    source install/setup.bash && \
    colcon build --packages-skip \
      zed_components zed_ros2 zed_wrapper zed_debug \
      epick_moveit_studio \
      onrobot_2fg7_driver onrobot_2fg7_description"

# Pre-configure Zivid network camera
# RUN mkdir -p /root/.config/Zivid/API && \
#     printf '__version__: 1\nCameras:\n  NetworkCameras:\n    - NetworkCamera:\n        Host: 10.68.81.25\n' \
#     > /root/.config/Zivid/API/Cameras.yml

# DDS / discovery server configuration (FastRTPS with discovery server on VM)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DISCOVERY_SERVER="10.65.2.151:11811;10.65.2.152:11888"
ENV ROS_DOMAIN_ID=0
ENV ROS_SUPER_CLIENT=TRUE
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ENV UR_TYPE="ur5e"
ENV LAUNCH_RVIZ="false"

# VNC viewer port
# EXPOSE 5901

# Source workspace in all shells (docker exec, interactive, etc.)
RUN echo "source /root/ws/erobs/install/setup.bash" >> /root/.bashrc

# VNC-enabled entrypoint — re-enable the COPY start-vnc.sh lines above to use this:
# CMD ["/bin/bash", "-c", "source /root/ws/erobs/install/setup.bash && /root/start-vnc.sh && exec /bin/bash"]
CMD ["/bin/bash", "-c", "source /root/ws/erobs/install/setup.bash && exec /bin/bash"]
