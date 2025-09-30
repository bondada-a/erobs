FROM osrf/ros:humble-desktop-full

# Set display
ENV DISPLAY=:0

RUN apt-get update \
        && apt-get -y install \
        python3-pip \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-ur \
        build-essential \
        python3-colcon-common-extensions
RUN apt-get install -y x11vnc xvfb 
# Clean up
RUN apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
RUN pip3 install rosdep && \
        rosdep update


# X11 environment configuration
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/tmp/.docker.xauth
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV QT_X11_NO_DPMS=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

RUN mkdir /root/ws/src -p

WORKDIR /root/ws/src

RUN git clone https://github.com/bondada-a/erobs.git -b ur5e

WORKDIR /root/ws/

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
        colcon build --symlink-install --packages-select ur5e_robot_description && \
        colcon build --symlink-install --packages-select ur5e_hande_moveit_config"

WORKDIR /root/ws/

ENV ROBOT_IP=10.69.26.90
ENV REVERSE_IP=192.168.56.102
ENV UR_TYPE="ur5e"
ENV LAUNCH_RVIZ="true"
ENV DESCRIPTION_PKG="ur5e_robot_description"
ENV DESCRIPTION_FILE="ur_with_hande.xacro"
ENV CONFIG_PKG="ur5e_hande_moveit_config"
ENV CONFIG_FILE="ur.srdf"

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# VNC viewer
EXPOSE 5901

ENTRYPOINT [ "/entrypoint.sh" ]