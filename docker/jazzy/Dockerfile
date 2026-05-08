FROM osrf/ros:jazzy-desktop-full

# Set display for RViz/GUI tools
ENV DISPLAY=:1

# Fix APT sandboxing issues for container environments
RUN echo 'APT::Sandbox::User "root";' > /etc/apt/apt.conf.d/99sandboxdisable

# Install system dependencies
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
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-ur \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-rosbridge-server \
    x11vnc \
    xvfb \
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
RUN mkdir -p /etc/OpenCL/vendors && \
    echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

RUN mkdir -p /root/ws
WORKDIR /root/ws

# Copy and setup VNC startup script
COPY docker/erobs-common-img/start-vnc.sh /root/start-vnc.sh
RUN chmod +x /root/start-vnc.sh

# Clone erobs and import all repos
ARG CACHEBUST=1
RUN git clone https://github.com/bondada-a/erobs.git -b jazzy_dev

WORKDIR /root/ws/erobs/
RUN /bin/bash -c "set -e && \
    vcs import < src/ros2.repos src && \
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

# Build remaining packages (skip ZED and incompatible packages)
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd erobs && \
    source install/setup.bash && \
    colcon build --packages-skip \
      zed_components zed_ros2 zed_wrapper zed_debug \
      epick_moveit_studio \
      epick_driver epick_description epick_controllers epick_hardware_tests epick_moveit_plugin \
      onrobot_2fg7_driver onrobot_2fg7_description"

# Install Claude Code CLI
RUN curl -fsSL https://claude.ai/install.sh | bash
ENV PATH="/root/.claude/local/bin:${PATH}"

# Pre-configure Zivid network camera
RUN mkdir -p /root/.config/Zivid/API && \
    printf '__version__: 1\nCameras:\n  NetworkCameras:\n    - NetworkCamera:\n        Host: 10.68.81.25\n' \
    > /root/.config/Zivid/API/Cameras.yml

# DDS / discovery server configuration (FastRTPS with discovery server on VM)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DISCOVERY_SERVER="10.68.82.42:11811"
ENV ROS_DOMAIN_ID=0

ENV ROBOT_IP=10.68.82.41
ENV UR_TYPE="ur5e"
ENV LAUNCH_RVIZ="false"

# VNC viewer port
EXPOSE 5901

# Source workspace in all shells (docker exec, interactive, etc.)
RUN echo "source /root/ws/erobs/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash", "-c", "source /root/ws/erobs/install/setup.bash && /root/start-vnc.sh && exec /bin/bash"]
