FROM osrf/ros:humble-desktop-full

# Set display
ENV DISPLAY :1

RUN apt-get update \
        && apt-get -y install \
        python3-pip \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-ur \
        build-essential \
        python3-colcon-common-extensions
        
RUN apt-get install -y x11vnc xvfb wget

# Install Zivid SDK 2.16.0 for Ubuntu 22.04 (using local files)
COPY docker/erobs-common-img/zivid-debs/*.deb /tmp/zivid/
RUN apt-get update && apt-get install -y /tmp/zivid/*.deb && rm -rf /tmp/zivid

# Clean up
RUN apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
RUN pip3 install rosdep && \
        rosdep update

RUN mkdir /root/ws -p

WORKDIR /root/ws

# Copy and setup VNC startup script
COPY docker/erobs-common-img/start-vnc.sh /root/start-vnc.sh
RUN chmod +x /root/start-vnc.sh

RUN git clone https://github.com/bondada-a/erobs.git -b zivid_integration

WORKDIR /root/ws/erobs/
RUN /bin/bash -c "set -e && \
    vcs import < src/ros2.repos src && \
    vcs import < src/end_effectors/end_effectors.repos src/end_effectors && \
    vcs import < src/vision/vision.repos src/vision && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys 'moveit_studio_behavior_interface zivid_description'"

WORKDIR /root/ws/
# Build interface packages first so other packages can find them
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd erobs && colcon build --packages-select zivid_interfaces pipette_driver"
# Build remaining packages
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd erobs && source install/setup.bash && colcon build --packages-skip zed_components zed_ros2 zed_wrapper epick_moveit_studio"

ENV ROBOT_IP=10.69.26.90
ENV REVERSE_IP=10.69.26.42
ENV UR_TYPE="ur5e"
ENV LAUNCH_RVIZ="false"

# VNC viewer
EXPOSE 5901

# Auto-start VNC server and source ROS workspace
CMD ["/bin/bash", "-c", "source /root/ws/erobs/install/setup.bash && /root/start-vnc.sh && exec /bin/bash"]
