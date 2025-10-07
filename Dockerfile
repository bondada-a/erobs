FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=/workspace

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for Bluesky
RUN pip3 install --no-cache-dir \
    bluesky \
    ophyd \
    ipython \
    numpy

# Create workspace
WORKDIR ${WORKSPACE}

# Copy source code
COPY . ${WORKSPACE}/

# Install ROS dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source workspace in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> ~/.bashrc && \
    echo "export PYTHONPATH=${WORKSPACE}/src:\$PYTHONPATH" >> ~/.bashrc

# Set up entrypoint
COPY docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
