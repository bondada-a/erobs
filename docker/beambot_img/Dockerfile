FROM osrf/ros:jazzy-desktop-full

ENV DISPLAY=:1

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-ur \
    python3-colcon-common-extensions \
    x11vnc \
    xvfb \
    socat

# Clean up
RUN apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*

# Pin numpy<2 for cv_bridge compatibility (cv_bridge compiled against NumPy 1.x)
RUN pip3 install 'numpy<2' rosdep && \
        rosdep update

RUN mkdir /root/ws -p

WORKDIR /root/ws

# Copy and setup VNC startup script
COPY docker/beambot_img/start-vnc.sh /root/start-vnc.sh
RUN chmod +x /root/start-vnc.sh

# Cache-bust arg: use --build-arg CACHEBUST=$(date +%s) to force fresh clone
ARG CACHEBUST=1
RUN git clone https://github.com/bondada-a/erobs.git -b jazzy_dev

WORKDIR /root/ws/erobs/
RUN /bin/bash -c "set -e && \
    vcs import < src/ros2.repos src && \
    vcs import < src/end_effectors/end_effectors.repos src/end_effectors && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys 'moveit_studio_behavior_interface pipette_driver zivid_description zivid_interfaces ament_python pytest'"

WORKDIR /root/ws/
# Build interface packages first so other packages can find them
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && cd erobs && colcon build --packages-select pipette_driver"
# Build remaining packages (skip epick_moveit_studio - requires MoveIt Studio)
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && cd erobs && source install/setup.bash && colcon build --packages-skip epick_moveit_studio"


ENV UR_TYPE="ur5e"
ENV LAUNCH_RVIZ="false"

# VNC viewer
EXPOSE 5901

# Auto-start VNC server and source ROS workspace
CMD ["/bin/bash", "-c", "source /root/ws/erobs/install/setup.bash && /root/start-vnc.sh && exec /bin/bash"]
