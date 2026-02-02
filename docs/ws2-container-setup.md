# WS2 Container/Conda Setup for Robot Control

## Overview

Guide for controlling the robot from WS2 using either a Podman container or conda-forge ROS2 environment.

**Network Info:**
- Robot IP: `10.68.82.41`
- WS2 IP: `10.68.80.222` or `10.68.83.222`
- VM IP: `10.68.82.42`

> **Note:** WS2 currently cannot ping the robot IP — likely a subnet mismatch (WS2 on 10.68.80.x/83.x, robot on 10.68.82.x). Verify port/network config before proceeding.

---

## Option 1: Conda Environment (robostack)

### Create Environment

```bash
conda create -n ros2_client python=3.10
conda activate ros2_client
```

### Install ROS2 Packages

```bash
conda install robotstack-staging::ros-humble-rclpy
conda install conda-forge::colcon-common-extensions
conda install robostack-staging::ros-humble-rosidl-default-generators
conda install robostack-staging::ros-humble-geometry-msgs
conda install robostack-staging::ros-humble-ros2cli
conda install robostack-staging::ros-humble-ros2action
```

### Set Python Environment Variables

```bash
# Set ALL Python-related variables
export Python3_EXECUTABLE=$CONDA_PREFIX/bin/python
export PYTHON_EXECUTABLE=$CONDA_PREFIX/bin/python
export PYTHON3_EXECUTABLE=$CONDA_PREFIX/bin/python

# Also set library paths
export Python3_LIBRARY=$CONDA_PREFIX/lib/libpython3.10.so
export Python3_INCLUDE_DIR=$CONDA_PREFIX/include/python3.10
```

### Build beambot_interfaces

```bash
colcon build --packages-select beambot_interfaces \
    --cmake-args \
    -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python \
    -DPython_EXECUTABLE=$CONDA_PREFIX/bin/python \
    -DPYTHON_EXECUTABLE=$CONDA_PREFIX/bin/python \
    -DPython3_LIBRARY=$CONDA_PREFIX/lib/libpython3.10.so \
    -DPython3_INCLUDE_DIR=$CONDA_PREFIX/include/python3.10
```

---

## Option 2: Podman Container

Spin up a Podman container with ROS2 Humble, editing the robot IP to the correct address.

*(Container setup details TBD)*

---

## Sending Action Goals

Once the environment is set up and network connectivity is confirmed:

```bash
ros2 action send_goal --feedback /beambot_execution beambot_interfaces/action/MTCExecution \
    "{full_json: '$(cat /nsls2/users/abondada/bdi/erobs/src/cms/tasks/beamtime/spincoat_to_hotplate.json | tr -d '\n')'}"
```

---

## Troubleshooting

### Network Connectivity
- Verify WS2 can reach the robot subnet (10.68.82.x)
- Check if connected to correct network port
- Test ping from VM (10.68.82.42) to robot (10.68.82.41) to isolate issue

### Build Issues
- Ensure all Python environment variables are set before building
- Use the full cmake-args as shown above for colcon build
