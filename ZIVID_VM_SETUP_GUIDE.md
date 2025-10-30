# Zivid Camera Setup Guide for VM Environment (Ubuntu 22.04 / ROS 2 Humble)

## Overview
This document provides a comprehensive guide for setting up and running a Zivid camera in a virtual machine environment without GPU access, using CPU-only OpenCL implementation. This setup was tested with a Zivid 2+ MR60 camera connected via Ethernet.

## System Configuration
- **VM Environment**: Ubuntu 22.04 (Jammy)
- **ROS Version**: ROS 2 Humble
- **Camera**: Zivid 2+ MR60
- **Connection**: Ethernet (GigE Vision)
- **CPU**: Intel Xeon Gold 6252 CPU @ 2.10GHz
- **OpenCL**: Intel CPU Runtime (no GPU)

---

## Issues Encountered and Solutions

### Issue 1: APT Privilege Errors in Container/VM
**Error:**
```
E: setgroups 65534 failed - setgroups (22: Invalid argument)
E: setegid 65534 failed - setegid (22: Invalid argument)
```

**Solution:**
Disable APT sandboxing:
```bash
echo 'APT::Sandbox::User "root";' > /etc/apt/apt.conf.d/99sandboxdisable
apt-get update
```

### Issue 2: Zivid SDK Not Found During Build
**Error:**
```
CMake Error: Could not find a package configuration file provided by "Zivid"
```

**Solution:**
Install Zivid SDK manually (not available via rosdep).

### Issue 3: OpenCL Platform Not Found
**Error:**
```
Error: An OpenCL error occurred: Failed to get platforms [CL_PLATFORM_NOT_FOUND_KHR]
```

**Solution:**
Install Intel OpenCL runtime for CPU (not PoCL which causes segfaults).

### Issue 4: Camera Discovery Failure
**Problem:**
Camera at known IP (10.69.26.41) not discovered via multicast in VM.

**Solution:**
Create Cameras.yml configuration file to specify camera IP directly.

### Issue 5: Settings Version Mismatch
**Error:**
```
Invalid Settings2D version. Found version 26, required version is 7 or older
```

**Solution:**
Use correct settings format version (7) for the SDK version.

---

## Complete Installation Guide

### Step 1: Fix APT Issues (if in container/VM)
```bash
# Fix APT sandboxing issues
echo 'APT::Sandbox::User "root";' > /etc/apt/apt.conf.d/99sandboxdisable
apt-get update
```

### Step 2: Install Basic Dependencies
```bash
# Install essential tools
apt-get install -y \
    wget \
    curl \
    build-essential \
    cmake \
    net-tools \
    iputils-ping \
    netcat-openbsd \
    iproute2

# Install OpenCL dependencies
apt-get install -y \
    ocl-icd-libopencl1 \
    opencl-headers \
    clinfo
```

### Step 3: Install Zivid SDK
```bash
# Create working directory
mkdir -p ~/zivid_install && cd ~/zivid_install

# Download Zivid SDK packages (version 2.16.0)
wget https://downloads.zivid.com/sdk/releases/2.16.0+46cdaba6-1/u22/amd64/zivid_2.16.0+46cdaba6-1_amd64.deb
wget https://downloads.zivid.com/sdk/releases/2.16.0+46cdaba6-1/u22/amd64/zivid-studio_2.16.0+46cdaba6-1_amd64.deb
wget https://downloads.zivid.com/sdk/releases/2.16.0+46cdaba6-1/u22/amd64/zivid-tools_2.16.0+46cdaba6-1_amd64.deb
wget https://downloads.zivid.com/sdk/releases/2.16.0+46cdaba6-1/u22/amd64/zivid-genicam_2.16.0+46cdaba6-1_amd64.deb

# Install packages
apt-get install -y ./zivid*.deb

# Verify installation
which ZividListCameras
```

### Step 4: Setup Intel OpenCL Runtime for CPU
```bash
# Remove PoCL if installed (causes segfaults with Zivid)
apt-get remove -y pocl-opencl-icd

# Add Intel oneAPI repository
wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | \
    gpg --dearmor | tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null

echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | \
    tee /etc/apt/sources.list.d/oneAPI.list

# Update and install Intel OpenCL runtime
apt-get update
apt-get install -y intel-oneapi-runtime-opencl intel-oneapi-runtime-compilers

# Verify OpenCL platform
clinfo -l
# Should show: Platform #0: Intel(R) OpenCL
#              Device #0: Intel(R) Xeon(R) Gold...
```

### Step 5: Configure Zivid for CPU Mode
```bash
# Create Zivid configuration directory
mkdir -p ~/.config/Zivid/API

# Create CPU configuration
cat > ~/.config/Zivid/API/Config.yml << 'EOF'
__version__:
  serializer: 1
  data: 19
Configuration:
  ComputeDevice:
    Type: CPU
    AllowUnsupported: yes
EOF
```

### Step 6: Configure Camera Network Discovery
```bash
# Create camera configuration with static IP
# (Required because multicast discovery often fails in VMs)
cat > ~/.config/Zivid/API/Cameras.yml << 'EOF'
__version__: 1
Cameras:
  NetworkCameras:
    - NetworkCamera:
        Host: 10.68.81.52  # Replace with your camera's IP
EOF

# Verify camera connectivity
ping 10.69.81.52 -c 3

# Test camera discovery
ZividListCameras
# Should show: { Serial Number: XXXXXXXX, Model: zivid2PlusMR60, IP Address: 10.69.26.41 }
```

### Step 7: Build ROS 2 Workspace
```bash
cd ~/ws/erobs  # Your workspace path

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build

# Source workspace
source install/setup.bash
```

### Step 8: Create Default 2D Settings File
```bash
# Create settings directory
mkdir -p ~/zivid_config

# Create default 2D settings (version 7 for compatibility)
cat > ~/zivid_config/default_2d_settings.yml << 'EOF'
__version__:
  serializer: 1
  data: 7
Settings2D:
  Acquisitions:
    - Acquisition:
        Aperture: 5.66
        Brightness: 1.0
        ExposureTime: 10000
        Gain: 1.0
  Processing:
    Color:
      Balance:
        Blue: 1.0
        Green: 1.0
        Red: 1.0
      Gamma: 1.0
EOF
```

### Step 9: Run Zivid Camera Node
```bash
# Source workspace
cd ~/ws/erobs
source install/setup.bash

# Option 1: Run with settings file
ros2 run zivid_camera zivid_camera --ros-args \
    -p settings_2d_file_path:=/root/zivid_config/default_2d_settings.yml

# Option 2: Run in background and trigger capture
ros2 run zivid_camera zivid_camera --ros-args \
    -p settings_2d_file_path:=/root/zivid_config/default_2d_settings.yml &

# Wait for connection
sleep 5

# Trigger 2D capture
ros2 service call /capture_2d std_srvs/srv/Trigger

# Check available topics
ros2 topic list | grep image
```

---

## Troubleshooting Commands

### Check System Status
```bash
# Verify OpenCL platforms
clinfo -l

# Test camera discovery
ZividListCameras

# Check camera connectivity
ping <camera_ip> -c 3

# Check running processes (kill if stuck)
ps aux | grep zivid
pkill -f zivid_camera  # If needed
```

### Verify Configuration Files
```bash
# Check Zivid configurations
ls -la ~/.config/Zivid/API/
cat ~/.config/Zivid/API/Config.yml
cat ~/.config/Zivid/API/Cameras.yml
```

### Network Diagnostics
```bash
# Check network interfaces
ip addr show

# Verify you're on same subnet as camera
# Camera: 10.69.26.41
# VM should be: 10.69.26.x/24
```

---

## Important Notes

### CPU Mode Limitations
- **Performance**: CPU mode is significantly slower than GPU mode
- **Support**: CPU mode is marked as "unsupported" by Zivid but works for testing
- **Use Case**: Suitable for development/testing, not production

### VM-Specific Considerations
- **Multicast Discovery**: Often fails in VMs, hence the need for Cameras.yml
- **Display**: RViz will fail without X11 forwarding or display setup
- **Performance**: Expect slower capture and processing times

### Required Network Configuration
- Camera and VM must be on the same subnet
- Firewall must allow Zivid traffic
- GigE Vision protocol requires proper MTU settings (usually 1500)

### Package Versions
- Zivid SDK: 2.16.0
- Settings format version: 7 (not 26)
- ROS 2: Humble
- Ubuntu: 22.04

---

## Quick Test Procedure

```bash
# 1. Check OpenCL
clinfo -l

# 2. List cameras
ZividListCameras

# 3. Start camera node
ros2 run zivid_camera zivid_camera --ros-args \
    -p settings_2d_file_path:=/root/zivid_config/default_2d_settings.yml &

# 4. Trigger capture
sleep 5
ros2 service call /capture_2d std_srvs/srv/Trigger

# 5. Verify success in response message
```

---

## Environment Setup Script

Save this as `setup_zivid_env.sh`:

```bash
#!/bin/bash

# Source Intel oneAPI if available
if [ -f /opt/intel/oneapi/setvars.sh ]; then
    source /opt/intel/oneapi/setvars.sh
fi

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace
source ~/ws/erobs/install/setup.bash

# Set Zivid environment
export ZIVID_LOG_LEVEL=info

echo "Zivid environment ready!"
echo "Run: ZividListCameras to test camera discovery"
```

---

## Conclusion

This setup allows running Zivid cameras in a CPU-only VM environment, though with performance limitations. The key challenges are:
1. Installing the correct OpenCL runtime (Intel, not PoCL)
2. Configuring static IP discovery via Cameras.yml
3. Using compatible settings file versions
4. Managing process conflicts (pkill when needed)

For production use, a system with proper GPU support is strongly recommended.