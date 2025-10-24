#!/bin/bash

# Automated Zivid VM Setup Script
# For Ubuntu 22.04 with ROS 2 Humble

set -e  # Exit on error

echo "========================================="
echo "Zivid Camera VM Setup Script"
echo "========================================="

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (use sudo)"
    exit 1
fi

# Step 1: Fix APT sandboxing
echo "[1/9] Fixing APT sandboxing issues..."
echo 'APT::Sandbox::User "root";' > /etc/apt/apt.conf.d/99sandboxdisable
apt-get update -qq

# Step 2: Install dependencies
echo "[2/9] Installing basic dependencies..."
apt-get install -y -qq \
    wget curl build-essential cmake \
    net-tools iputils-ping netcat-openbsd iproute2 \
    ocl-icd-libopencl1 opencl-headers clinfo

# Step 3: Download and install Zivid SDK
echo "[3/9] Downloading Zivid SDK..."
mkdir -p /tmp/zivid_install && cd /tmp/zivid_install

wget -q https://downloads.zivid.com/sdk/releases/2.16.0+46cdaba6-1/u22/amd64/zivid_2.16.0+46cdaba6-1_amd64.deb
wget -q https://downloads.zivid.com/sdk/releases/2.16.0+46cdaba6-1/u22/amd64/zivid-tools_2.16.0+46cdaba6-1_amd64.deb

echo "[4/9] Installing Zivid SDK..."
apt-get install -y -qq ./zivid*.deb

# Step 4: Setup Intel OpenCL
echo "[5/9] Setting up Intel OpenCL runtime..."
apt-get remove -y -qq pocl-opencl-icd 2>/dev/null || true

wget -qO- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | \
    gpg --dearmor | tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null

echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | \
    tee /etc/apt/sources.list.d/oneAPI.list > /dev/null

apt-get update -qq
apt-get install -y -qq intel-oneapi-runtime-opencl intel-oneapi-runtime-compilers

# Step 5: Configure Zivid for CPU
echo "[6/9] Configuring Zivid for CPU mode..."
mkdir -p ~/.config/Zivid/API

cat > ~/.config/Zivid/API/Config.yml << 'EOF'
__version__:
  serializer: 1
  data: 19
Configuration:
  ComputeDevice:
    Type: CPU
    AllowUnsupported: yes
EOF

# Step 6: Create default settings
echo "[7/9] Creating default 2D settings..."
mkdir -p ~/zivid_config

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

# Step 7: Prompt for camera IP
echo "[8/9] Camera network configuration..."
read -p "Enter your Zivid camera IP address (e.g., 10.69.26.41): " CAMERA_IP

if [ ! -z "$CAMERA_IP" ]; then
    cat > ~/.config/Zivid/API/Cameras.yml << EOF
__version__: 1
Cameras:
  NetworkCameras:
    - NetworkCamera:
        Host: $CAMERA_IP
EOF
    echo "Camera IP configured: $CAMERA_IP"
else
    echo "Skipping camera IP configuration (manual setup required)"
fi

# Step 8: Test installation
echo "[9/9] Testing installation..."
echo "----------------------------------------"
echo "OpenCL platforms:"
clinfo -l

echo ""
echo "Testing camera discovery..."
if [ ! -z "$CAMERA_IP" ]; then
    ping -c 1 $CAMERA_IP > /dev/null 2>&1 && echo "Camera ping: SUCCESS" || echo "Camera ping: FAILED"
fi

ZividListCameras 2>/dev/null || echo "Note: Camera discovery may require network configuration"

# Create helper script
cat > ~/setup_zivid_env.sh << 'EOF'
#!/bin/bash
# Source this file to setup Zivid environment

# Source Intel oneAPI if available
if [ -f /opt/intel/oneapi/setvars.sh ]; then
    source /opt/intel/oneapi/setvars.sh
fi

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace (adjust path as needed)
if [ -f ~/ws/erobs/install/setup.bash ]; then
    source ~/ws/erobs/install/setup.bash
fi

export ZIVID_LOG_LEVEL=info

echo "Zivid environment ready!"
echo "Commands:"
echo "  ZividListCameras - List available cameras"
echo "  ros2 run zivid_camera zivid_camera - Start camera node"
EOF

chmod +x ~/setup_zivid_env.sh

echo ""
echo "========================================="
echo "Setup Complete!"
echo "========================================="
echo "To use Zivid:"
echo "1. Source the environment: source ~/setup_zivid_env.sh"
echo "2. Test camera: ZividListCameras"
echo "3. Run ROS node: ros2 run zivid_camera zivid_camera --ros-args -p settings_2d_file_path:=$HOME/zivid_config/default_2d_settings.yml"
echo ""
echo "Configuration files created:"
echo "  ~/.config/Zivid/API/Config.yml      - CPU mode configuration"
echo "  ~/.config/Zivid/API/Cameras.yml     - Camera IP configuration"
echo "  ~/zivid_config/default_2d_settings.yml - Default capture settings"
echo ""