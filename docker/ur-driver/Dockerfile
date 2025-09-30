FROM osrf/ros:humble-desktop-full


RUN apt-get update \
        && apt-get -y install \
        python3-pip \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-ur \
        build-essential \
        python3-colcon-common-extensions \
        #
        # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
RUN pip3 install rosdep && \
        rosdep update

RUN mkdir /root/ws/src -p

WORKDIR /root/ws/src

RUN git clone https://github.com/bondada-a/erobs.git -b ur5e

WORKDIR /root/ws/

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
        colcon build --symlink-install --packages-select ur5e_robot_description"

ENV ROBOT_IP=10.69.26.90
ENV REVERSE_IP=10.69.26.42
ENV UR_TYPE="ur5e"
ENV DESCRIPTION_PKG="ur5e_robot_description"
ENV DESCRIPTION_FILE="ur_with_hande.xacro"

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]
