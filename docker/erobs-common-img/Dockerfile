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
        
RUN apt-get install -y x11vnc xvfb 
# Clean up
RUN apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
RUN pip3 install rosdep && \
        rosdep update

RUN mkdir /root/ws -p

WORKDIR /root/ws

# Copy and setup VNC startup script
COPY start-vnc.sh /root/start-vnc.sh
RUN chmod +x /root/start-vnc.sh

RUN git clone https://github.com/bondada-a/erobs.git -b zivid_integration

WORKDIR /root/ws/erobs/
RUN /bin/bash -c ". ./setup.sh"

WORKDIR /root/ws/
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd erobs && colcon build --packages-skip zed_components zed_ros2 zed_wrapper epick_moveit_studio"

ENV ROBOT_IP=10.69.26.90
ENV REVERSE_IP=10.69.26.42
ENV UR_TYPE="ur5e"
ENV LAUNCH_RVIZ="false"

# VNC viewer
EXPOSE 5901

# Auto-start VNC server and source ROS workspace
CMD ["/bin/bash", "-c", "source /root/ws/erobs/install/setup.bash && /root/start-vnc.sh && exec /bin/bash"]
