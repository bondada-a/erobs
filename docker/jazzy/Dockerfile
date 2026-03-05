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
    vcs import < src/vision/vision.repos src/vision"

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
      epick_driver epick_description epick_controllers epick_hardware_tests epick_moveit_plugin"

ENV ROBOT_IP=10.69.26.90
ENV REVERSE_IP=10.69.26.42
ENV UR_TYPE="ur5e"
ENV LAUNCH_RVIZ="false"

# VNC viewer port
EXPOSE 5901

CMD ["/bin/bash", "-c", "source /root/ws/erobs/install/setup.bash && /root/start-vnc.sh && exec /bin/bash"]
