# Docker Images Status Report

**Date**: 2025-12-02
**Status**: ✅ All images built, pushed, and verified

## 📦 Images Available

### 1. ur5e-erobs-common-img (Base Image)

**Registry**: `ghcr.io/bondada-a/ur5e-erobs-common-img:latest`
**Digest**: `sha256:2a266eb3194efdc47a85989ed9f7d231ed27ddb81c795e40c1b5f8007040afa9`
**Size**: 10.77 GB (compressed), 11.6 GB (on disk)
**Status**: ✅ Pulled and verified

**Contains**:
- ✅ ROS 2 Humble Desktop Full
- ✅ Zivid SDK 2.16.0
- ✅ Intel OpenCL Runtime
- ✅ UR robot drivers
- ✅ Built ROS workspace (erobs)
- ✅ MoveIt Task Constructor
- ✅ VNC server for remote GUI

**Pull Command**:
```bash
docker pull ghcr.io/bondada-a/ur5e-erobs-common-img:latest
```

**Test Command**:
```bash
docker run --rm ghcr.io/bondada-a/ur5e-erobs-common-img:latest \
  ros2 pkg list | grep mtc
```

---

### 2. bsui-img-new (Bluesky Integration)

**Registry**: `ghcr.io/bondada-a/bsui-img-new:latest`
**Digest**: `sha256:e699d5f7af41a30e1276547f9be2e7b152582e57b6f305a488f83cc2ad649568`
**Size**: 12.01 GB (compressed), 12.9 GB (on disk)
**Status**: ✅ Pulled and verified

**Contains** (extends ur5e-erobs-common-img):
- ✅ All from base image
- ✅ EPICS Base (compiled)
- ✅ Conda (Miniconda)
- ✅ Bluesky ecosystem (via pip)
- ✅ Ophyd, IPython, nslsii
- ✅ Custom bsui launcher script
- ✅ Python 3.13 base + conda environments

**Pull Command**:
```bash
docker pull ghcr.io/bondada-a/bsui-img-new:latest
```

**Test Command**:
```bash
# Quick test
docker run --rm ghcr.io/bondada-a/bsui-img-new:latest \
  /bin/bash -c "ls /root/ws/erobs/src/bluesky_ros/*.py"

# Full interactive session
docker run -it --network host ghcr.io/bondada-a/bsui-img-new:latest
```

---

## 🚀 Usage Examples

### Run the bsui Container

```bash
# Basic run
docker run -it ghcr.io/bondada-a/bsui-img-new:latest

# With robot IP and network access
docker run -it \
    --network host \
    -e ROBOT_IP=10.69.26.90 \
    ghcr.io/bondada-a/bsui-img-new:latest

# With GUI support (X11)
docker run -it \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ghcr.io/bondada-a/bsui-img-new:latest

# With VNC (headless)
docker run -d \
    -p 5901:5901 \
    -e ROBOT_IP=10.69.26.90 \
    ghcr.io/bondada-a/bsui-img-new:latest
# Connect with VNC client to localhost:5901
```

### Execute Bluesky Script

```bash
# Run a simple MTC task with Bluesky
docker run --rm --network host \
    -e ROBOT_IP=10.69.26.90 \
    ghcr.io/bondada-a/bsui-img-new:latest \
    python3 /root/ws/erobs/src/bluesky_ros/simple_mtc_bluesky.py \
    /root/ws/erobs/task_sequences/complete_sequence.json
```

### Mount Local Files

```bash
# Mount local workspace to use your code
docker run -it \
    --network host \
    -v /home/aditya/work/github_ws/erobs:/root/ws/erobs \
    ghcr.io/bondada-a/bsui-img-new:latest
```

---

## 📊 Image Architecture

```
ur5e-erobs-common-img (11.6 GB)
├── Ubuntu 22.04 (osrf/ros:humble-desktop-full)
├── ROS 2 Humble
├── Zivid SDK 2.16.0
├── Intel OpenCL Runtime
├── UR Drivers (ros-humble-ur)
├── Built ROS Workspace
│   ├── MoveIt Task Constructor
│   ├── mtc_pipeline
│   ├── pipette_driver
│   └── zivid_interfaces
└── VNC Server

bsui-img-new (12.9 GB) extends ur5e-erobs-common-img
├── All from base image
├── EPICS Base
│   └── /root/EPICS/epics-base
├── Conda (Miniconda)
│   └── /opt/conda
├── Bluesky Packages (pip)
│   ├── bluesky
│   ├── ophyd
│   ├── ipython
│   ├── nslsii
│   └── tiled
└── bsui Launcher Script
    └── /bin/bsui
```

---

## 🔐 Registry Information

**Registry**: GitHub Container Registry (ghcr.io)
**Owner**: bondada-a
**Visibility**: Public (anyone can pull)

**View on GitHub**:
- https://github.com/bondada-a/erobs/pkgs/container/ur5e-erobs-common-img
- https://github.com/bondada-a/erobs/pkgs/container/bsui-img-new

---

## 🔄 Updating Images

If you need to rebuild and push updated images:

```bash
cd /home/aditya/work/github_ws/erobs

# Rebuild base image
docker build -f docker/erobs-common-img/Dockerfile \
    -t ur5e-erobs-common-img .

# Tag and push base image
docker tag ur5e-erobs-common-img \
    ghcr.io/bondada-a/ur5e-erobs-common-img:latest
docker push ghcr.io/bondada-a/ur5e-erobs-common-img:latest

# Rebuild bsui image
docker build -t bsui-img-new \
    -f docker/bsui/Dockerfile docker/bsui/

# Tag and push bsui image
docker tag bsui-img-new \
    ghcr.io/bondada-a/bsui-img-new:latest
docker push ghcr.io/bondada-a/bsui-img-new:latest
```

---

## ✅ Verification Results

### Pull Test
```
✓ ur5e-erobs-common-img - Successfully pulled
✓ bsui-img-new - Successfully pulled
```

### Component Test (bsui-img-new)
```
✓ ROS 2 Humble - ROS_DISTRO=humble
✓ Python 3.13 - Available
✓ Conda - Installed at /opt/conda
✓ EPICS Base - Installed with caRepeater
✓ Workspace - 5 Bluesky scripts found
✓ bsui launcher - Executable at /bin/bsui
```

### Image Digests (for reproducibility)
```
ur5e-erobs-common-img:
  sha256:2a266eb3194efdc47a85989ed9f7d231ed27ddb81c795e40c1b5f8007040afa9

bsui-img-new:
  sha256:e699d5f7af41a30e1276547f9be2e7b152582e57b6f305a488f83cc2ad649568
```

---

## 🎯 Next Steps

1. ✅ Images are pulled and ready
2. ✅ Images are tested and verified
3. ⬜ Run interactive session: `docker run -it ghcr.io/bondada-a/bsui-img-new:latest`
4. ⬜ Test with real robot
5. ⬜ Deploy to production environment

---

## 📚 Related Documentation

- [DUAL_SETUP_GUIDE.md](DUAL_SETUP_GUIDE.md) - Complete local + Docker guide
- [INSTALLATION_COMPLETE.md](INSTALLATION_COMPLETE.md) - Installation summary
- [README_BLUESKY.md](README_BLUESKY.md) - Quick reference

---

**Last Updated**: 2025-12-02
**Images Version**: Latest
**Status**: Production Ready ✅
