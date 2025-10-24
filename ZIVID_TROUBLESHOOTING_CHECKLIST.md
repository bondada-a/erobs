# Zivid VM Troubleshooting Quick Reference

## Quick Diagnostics

```bash
# Run this diagnostic script to check your setup
#!/bin/bash

echo "=== Zivid VM Diagnostic Check ==="
echo ""

# 1. Check OpenCL
echo "[1] OpenCL Status:"
clinfo -l 2>/dev/null || echo "  ❌ OpenCL not working"
echo ""

# 2. Check Zivid SDK
echo "[2] Zivid SDK:"
which ZividListCameras && echo "  ✓ SDK installed" || echo "  ❌ SDK not found"
echo ""

# 3. Check config files
echo "[3] Configuration Files:"
[ -f ~/.config/Zivid/API/Config.yml ] && echo "  ✓ CPU config exists" || echo "  ❌ CPU config missing"
[ -f ~/.config/Zivid/API/Cameras.yml ] && echo "  ✓ Camera IP config exists" || echo "  ❌ Camera IP config missing"
echo ""

# 4. Test camera discovery
echo "[4] Camera Discovery:"
ZividListCameras 2>/dev/null || echo "  ⚠ Camera not found (check network)"
```

## Common Issues & Quick Fixes

### 🔴 "No OpenCL platforms found"
```bash
# Fix: Install Intel OpenCL (NOT PoCL)
apt-get remove -y pocl-opencl-icd
apt-get install -y intel-oneapi-runtime-opencl
clinfo -l  # Should show Intel platform
```

### 🔴 "Segmentation fault" on ZividListCameras
```bash
# Fix: Wrong OpenCL or missing CPU config
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

### 🔴 "No cameras found" (but camera is pingable)
```bash
# Fix: Add camera IP to config
cat > ~/.config/Zivid/API/Cameras.yml << 'EOF'
__version__: 1
Cameras:
  NetworkCameras:
    - NetworkCamera:
        Host: YOUR_CAMERA_IP_HERE
EOF
```

### 🔴 "Both settings parameters are empty"
```bash
# Fix: Specify settings file when running
ros2 run zivid_camera zivid_camera --ros-args \
    -p settings_2d_file_path:=/root/zivid_config/default_2d_settings.yml
```

### 🔴 "Invalid Settings2D version"
```bash
# Fix: Use version 7, not 26
# Edit your settings file and change:
#   data: 26  → data: 7
```

### 🔴 Camera not responding after failed attempt
```bash
# Fix: Kill stuck processes
pkill -f zivid_camera
sleep 2
ZividListCameras
```

### 🔴 APT errors in container
```bash
# Fix: Disable sandboxing
echo 'APT::Sandbox::User "root";' > /etc/apt/apt.conf.d/99sandboxdisable
```

## Essential Commands

### Test Sequence
```bash
# 1. Test OpenCL
clinfo -l

# 2. Test camera discovery
ZividListCameras

# 3. Test ROS node
ros2 run zivid_camera zivid_camera --ros-args \
    -p settings_2d_file_path:=$HOME/zivid_config/default_2d_settings.yml &

# 4. Trigger capture
sleep 5
ros2 service call /capture_2d std_srvs/srv/Trigger
```

### Process Management
```bash
# View Zivid processes
ps aux | grep -i zivid

# Kill stuck camera node
pkill -f zivid_camera

# Kill all Zivid processes
pkill -f -i zivid
```

### Network Checks
```bash
# Check your IP
ip addr show | grep "inet 10"

# Ping camera
ping CAMERA_IP -c 3

# Check if on same subnet
# Camera: 10.69.26.41 → You need: 10.69.26.x/24
```

## File Locations

| File | Path | Purpose |
|------|------|---------|
| CPU Config | `~/.config/Zivid/API/Config.yml` | Forces CPU mode |
| Camera IPs | `~/.config/Zivid/API/Cameras.yml` | Static IP discovery |
| 2D Settings | `~/zivid_config/default_2d_settings.yml` | Capture parameters |
| Intel OpenCL | `/opt/intel/oneapi/setvars.sh` | Environment setup |

## Quick Health Check

Run this to verify everything:
```bash
echo "System ready:" && \
clinfo -l | grep -q Intel && \
which ZividListCameras > /dev/null && \
[ -f ~/.config/Zivid/API/Config.yml ] && \
[ -f ~/.config/Zivid/API/Cameras.yml ] && \
echo "✅ All systems go!" || echo "❌ Check setup"
```

## Nuclear Options

If nothing works:
```bash
# 1. Clean everything
apt-get remove -y '*zivid*' '*opencl*' '*pocl*'
rm -rf ~/.config/Zivid ~/.cache/Zivid ~/.local/share/Zivid

# 2. Run the setup script
chmod +x setup_zivid_vm.sh
sudo ./setup_zivid_vm.sh

# 3. Test
source ~/setup_zivid_env.sh
ZividListCameras
```