FROM osrf/ros:humble-desktop-full

# Set display
ENV DISPLAY=:1

# Step 1: Fix APT sandboxing issues for container environments
RUN echo 'APT::Sandbox::User "root";' > /etc/apt/apt.conf.d/99sandboxdisable

# Step 2: Install basic dependencies
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    build-essential \
    cmake \
    net-tools \
    iputils-ping \
    netcat-openbsd \
    iproute2 \
    python3-pip \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-ur \
    python3-colcon-common-extensions \
    x11vnc \
    xvfb \
    gnupg2 \
    ca-certificates \
    socat

# Step 2b: Install OpenCL dependencies
RUN apt-get install -y \
    ocl-icd-libopencl1 \
    opencl-headers \
    ocl-icd-opencl-dev \
    clinfo

# Step 3: Install Zivid SDK 2.17.2 for Ubuntu 22.04 (all packages)
RUN mkdir -p /tmp/zivid && cd /tmp/zivid && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u22/amd64/zivid_2.17.2+440b2367-1_amd64.deb && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u22/amd64/zivid-studio_2.17.2+440b2367-1_amd64.deb && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u22/amd64/zivid-tools_2.17.2+440b2367-1_amd64.deb && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u22/amd64/zivid-genicam_2.17.2+440b2367-1_amd64.deb && \
    apt-get update && apt-get install -y /tmp/zivid/*.deb && \
    rm -rf /tmp/zivid

# Step 4: Setup NVIDIA OpenCL ICD for GPU mode
RUN mkdir -p /etc/OpenCL/vendors && \
    echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

# Clean up
RUN apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
# Pin numpy<2 for ROS2 Humble compatibility (cv_bridge, tf_transformations compiled against NumPy 1.x)
RUN pip3 install 'numpy<2' rosdep && \
        rosdep update

RUN mkdir /root/ws -p

WORKDIR /root/ws

# Copy and setup VNC startup script
COPY docker/erobs-common-img/start-vnc.sh /root/start-vnc.sh
RUN chmod +x /root/start-vnc.sh

# Cache-bust arg: use --build-arg CACHEBUST=$(date +%s) to force fresh clone
ARG CACHEBUST=1
RUN git clone https://github.com/bondada-a/erobs.git -b humble-experimental

WORKDIR /root/ws/erobs/
RUN /bin/bash -c "set -e && \
    vcs import < src/ros2.repos src && \
    vcs import < src/end_effectors/end_effectors.repos src/end_effectors && \
    vcs import < src/vision/vision.repos src/vision && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys 'moveit_studio_behavior_interface pytest ament_python pipette_driver zivid_interfaces'"

WORKDIR /root/ws/
# Build interface packages first so other packages can find them
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd erobs && colcon build --packages-select zivid_interfaces pipette_driver"
# Build remaining packages
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd erobs && source install/setup.bash && colcon build --packages-skip zed_components zed_ros2 zed_wrapper zed_debug epick_moveit_studio"

ENV ROBOT_IP=10.69.26.90
ENV REVERSE_IP=10.69.26.42
ENV UR_TYPE="ur5e"
ENV LAUNCH_RVIZ="false"

# VNC viewer
EXPOSE 5901

# Auto-start VNC server and source ROS workspace
CMD ["/bin/bash", "-c", "source /root/ws/erobs/install/setup.bash && /root/start-vnc.sh && exec /bin/bash"]
