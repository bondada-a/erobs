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

RUN git clone https://github.com/bondada-a/erobs.git -b embedded_actions

WORKDIR /root/ws/erobs/
RUN /bin/bash -c ". ./setup.sh"

WORKDIR /root/ws/
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd erobs && colcon build --packages-skip zed_components zed_wrapper zed_ros2"

ENV ROBOT_IP=10.69.26.90
ENV REVERSE_IP=10.69.26.42
ENV UR_TYPE="ur5e"
ENV LAUNCH_RVIZ="false"
ENV DESCRIPTION_PKG="ur5e_hande_robot_description"
ENV DESCRIPTION_FILE="ur_with_hande.xacro"
ENV CONFIG_PKG="ur5e_hande_moveit_config"
ENV CONFIG_FILE="ur.srdf"

# VNC viewer
EXPOSE 5901
