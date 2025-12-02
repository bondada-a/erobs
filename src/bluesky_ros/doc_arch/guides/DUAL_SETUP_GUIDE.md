# Dual Setup Guide: Local + Docker

This guide covers both **local** and **Docker** installations of the Bluesky/ROS system, allowing you to work with either approach based on your needs.

## 📦 What's Installed

### Local Installation ✓

- ✅ **ROS 2 Humble** - `/opt/ros/humble`
- ✅ **Python 3.10** with Bluesky packages (pip)
- ✅ **EPICS Base** - `/home/aditya/EPICS/epics-base`
- ✅ **Miniconda** - `/home/aditya/miniconda3`
- ✅ **Conda Environment "bluesky"** - Python 3.10 with Bluesky packages
- ✅ **ROS Workspace** - `/home/aditya/work/github_ws/erobs`

### Docker Images ✓

- ✅ **ur5e-erobs-common-img** - Base image with Zivid SDK, ROS packages, workspace
  - `ghcr.io/bondada-a/ur5e-erobs-common-img:latest`
  - Digest: `sha256:2a266eb3194efdc47a85989ed9f7d231ed27ddb81c795e40c1b5f8007040afa9`

- ✅ **bsui-img-new** - Bluesky integration with EPICS and Conda
  - `ghcr.io/bondada-a/bsui-img-new:latest`
  - Digest: `sha256:e699d5f7af41a30e1276547f9be2e7b152582e57b6f305a488f83cc2ad649568`

## 🚀 Quick Start

### Option 1: Local Setup

```bash
cd /home/aditya/work/github_ws/erobs

# Source the environment
./local_bsui.sh

# Run a test
python3 test_bluesky_local.py

# Execute an MTC task with Bluesky
python3 src/bluesky_ros/simple_mtc_bluesky.py task_sequences/complete_sequence.json
```

### Option 2: Docker Setup

Pull and run the Docker images:

```bash
# Pull the images
docker pull ghcr.io/bondada-a/ur5e-erobs-common-img:latest
docker pull ghcr.io/bondada-a/bsui-img-new:latest

# Run the bsui container
docker run -it \
    --network host \
    -e ROBOT_IP=10.69.26.90 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ghcr.io/bondada-a/bsui-img-new:latest

# Inside the container, you'll have the full Bluesky/EPICS/ROS environment
```

## 🔧 Detailed Usage

### Local: Using System Python

Best for quick testing and development:

```bash
# Source environment
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"

# Run Bluesky script
python3 src/bluesky_ros/simple_mtc_bluesky.py <task.json>
```

### Local: Using Conda Environment

Best for matching the Docker environment exactly:

```bash
# Activate conda environment
source /home/aditya/miniconda3/etc/profile.d/conda.sh
conda activate bluesky

# Source ROS
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run with conda's Python
python src/bluesky_ros/simple_mtc_bluesky.py <task.json>
```

### Local: Using the Launcher Script

Easiest method - handles everything automatically:

```bash
./local_bsui.sh                    # Show environment info
./local_bsui.sh --ipython          # Start IPython session
./local_bsui.sh python3 script.py  # Run a script
```

### Docker: Basic Usage

```bash
# Run interactively
docker run -it ghcr.io/bondada-a/bsui-img-new:latest

# Run with custom robot IP
docker run -it \
    -e ROBOT_IP=192.168.1.100 \
    ghcr.io/bondada-a/bsui-img-new:latest

# Run with VNC access (port 5901)
docker run -d \
    -p 5901:5901 \
    -e ROBOT_IP=10.69.26.90 \
    ghcr.io/bondada-a/bsui-img-new:latest
```

### Docker: Using Docker Compose

```bash
cd /home/aditya/work/github_ws/erobs/docker

# Check the docker-compose.yml
cat docker-compose.yml

# Start services
docker-compose up -d

# View logs
docker-compose logs -f

# Stop services
docker-compose down
```

## 📊 Comparison: Local vs Docker

| Feature | Local | Docker |
|---------|-------|--------|
| **Setup Time** | Slower (build EPICS, install packages) | Fast (pull pre-built images) |
| **Disk Space** | Moderate (~2-3 GB) | Large (~5-8 GB per image) |
| **Performance** | Native (fastest) | Near-native (slight overhead) |
| **Isolation** | No isolation | Full isolation |
| **Development** | Easy to modify code | Need to rebuild for changes |
| **Portability** | Machine-specific | Works anywhere |
| **Zivid Camera** | Requires local SDK install | Included in image |
| **GPU Access** | Direct | Needs --gpus flag |
| **Best For** | Development, debugging | Deployment, reproducibility |

## 🏗️ Architecture

### Local Architecture

```
┌─────────────────────────────────────────┐
│  Host System (Ubuntu 22.04)             │
│  ┌───────────────────────────────────┐  │
│  │ ROS 2 Humble                      │  │
│  │ /opt/ros/humble                   │  │
│  └───────────────────────────────────┘  │
│  ┌───────────────────────────────────┐  │
│  │ EPICS Base                        │  │
│  │ ~/EPICS/epics-base                │  │
│  └───────────────────────────────────┘  │
│  ┌───────────────────────────────────┐  │
│  │ Python Packages (pip)             │  │
│  │ - bluesky, ophyd, ipython         │  │
│  └───────────────────────────────────┘  │
│  ┌───────────────────────────────────┐  │
│  │ Conda Environment                 │  │
│  │ ~/miniconda3/envs/bluesky         │  │
│  └───────────────────────────────────┘  │
│  ┌───────────────────────────────────┐  │
│  │ Workspace                         │  │
│  │ ~/work/github_ws/erobs            │  │
│  └───────────────────────────────────┘  │
└─────────────────────────────────────────┘
```

### Docker Architecture

```
┌─────────────────────────────────────────┐
│  Host System                             │
│                                          │
│  ┌────────────────────────────────────┐ │
│  │ Docker Container (bsui-img-new)   │ │
│  │  ┌──────────────────────────────┐ │ │
│  │  │ ROS 2 Humble                 │ │ │
│  │  └──────────────────────────────┘ │ │
│  │  ┌──────────────────────────────┐ │ │
│  │  │ EPICS Base                   │ │ │
│  │  └──────────────────────────────┘ │ │
│  │  ┌──────────────────────────────┐ │ │
│  │  │ Conda Environment            │ │ │
│  │  │ /opt/conda/envs/...          │ │ │
│  │  └──────────────────────────────┘ │ │
│  │  ┌──────────────────────────────┐ │ │
│  │  │ Zivid SDK                    │ │ │
│  │  └──────────────────────────────┘ │ │
│  │  ┌──────────────────────────────┐ │ │
│  │  │ Workspace                    │ │ │
│  │  │ /root/ws/erobs               │ │ │
│  │  └──────────────────────────────┘ │ │
│  └────────────────────────────────────┘ │
└─────────────────────────────────────────┘
```

## 🔄 Workflow: When to Use Each

### Use Local When:

1. **Active Development** - You're modifying code frequently
2. **Debugging** - You need direct access to files and processes
3. **Performance Critical** - Running compute-intensive tasks
4. **Testing Hardware** - Direct access to USB/network devices

### Use Docker When:

1. **Deployment** - Running on beamline or production system
2. **Reproducibility** - Need exact environment match
3. **Sharing** - Multiple users need same setup
4. **Isolation** - Testing without affecting host system
5. **CI/CD** - Automated testing and deployment

## 🛠️ Advanced: Switching Between Setups

### Test Locally, Deploy with Docker

Typical workflow:

```bash
# 1. Develop and test locally
./local_bsui.sh
python3 src/bluesky_ros/simple_mtc_bluesky.py test.json

# 2. Commit changes
git add .
git commit -m "Add new feature"
git push

# 3. Rebuild Docker image (if needed)
docker build -f docker/bsui/Dockerfile -t bsui-img-new docker/bsui/
docker tag bsui-img-new ghcr.io/bondada-a/bsui-img-new:latest
docker push ghcr.io/bondada-a/bsui-img-new:latest

# 4. Deploy with Docker
docker pull ghcr.io/bondada-a/bsui-img-new:latest
docker run -it ghcr.io/bondada-a/bsui-img-new:latest
```

### Share Files Between Local and Docker

Mount your local workspace into the container:

```bash
docker run -it \
    -v /home/aditya/work/github_ws/erobs:/root/ws/erobs \
    ghcr.io/bondada-a/bsui-img-new:latest
```

**Warning**: This will use your local code instead of the container's built-in code. You may need to rebuild inside the container.

## 📝 Environment Variables

### Local

Set in your shell or in `~/.bashrc`:

```bash
export ROBOT_IP=10.69.26.90
export REVERSE_IP=10.69.26.42
export UR_TYPE="ur5e"
export EPICS_BASE=/home/aditya/EPICS/epics-base
export PYTHONPATH=/home/aditya/work/github_ws/erobs/src:$PYTHONPATH
```

### Docker

Pass via `-e` flag or docker-compose:

```bash
docker run -it \
    -e ROBOT_IP=10.69.26.90 \
    -e REVERSE_IP=10.69.26.42 \
    -e UR_TYPE="ur5e" \
    ghcr.io/bondada-a/bsui-img-new:latest
```

## 🐛 Troubleshooting

### Issue: "Cannot connect to robot"

**Local**: Check firewall, network settings, robot IP
**Docker**: Use `--network host` to access host network

### Issue: "Module not found"

**Local**: Run `./local_bsui.sh` to set PYTHONPATH
**Docker**: Rebuild image if you added new packages

### Issue: "EPICS tools not found"

**Local**: Check `echo $EPICS_BASE` and `which caRepeater`
**Docker**: EPICS is pre-installed, check $PATH

### Issue: "Display/GUI not working"

**Local**: Should work automatically
**Docker**: Use `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` or VNC

### Issue: "Conda environment conflicts"

**Local**: Use `conda activate bluesky` for isolated environment
**Docker**: Use the container's pre-configured environment

## 📚 File Locations

### Local Setup

```
/home/aditya/
├── EPICS/
│   └── epics-base/          # EPICS installation
├── miniconda3/
│   └── envs/bluesky/        # Conda environment
└── work/github_ws/erobs/
    ├── src/                 # Source code
    ├── install/             # Built packages
    ├── local_bsui.sh        # Launcher script
    ├── test_bluesky_local.py
    └── task_sequences/      # JSON task files
```

### Docker Container

```
/root/
├── EPICS/
│   └── epics-base/          # EPICS installation
├── ws/erobs/
│   ├── src/                 # Source code
│   ├── install/             # Built packages
│   └── task_sequences/
├── zivid_config/            # Zivid settings
└── .config/Zivid/           # Zivid configuration

/opt/conda/                  # Conda installation
```

## 🎯 Quick Commands Reference

### Local

```bash
# Setup
./local_bsui.sh

# Test
python3 test_bluesky_local.py

# Run
python3 src/bluesky_ros/simple_mtc_bluesky.py <task.json>

# Interactive
./local_bsui.sh --ipython
```

### Docker

```bash
# Pull
docker pull ghcr.io/bondada-a/bsui-img-new:latest

# Run
docker run -it --network host ghcr.io/bondada-a/bsui-img-new:latest

# Check images
docker images | grep -E "(erobs|bsui)"

# Clean up
docker system prune -a
```

## 🔐 GitHub Container Registry

The Docker images are hosted at:
- https://github.com/bondada-a/erobs/pkgs/container/ur5e-erobs-common-img
- https://github.com/bondada-a/erobs/pkgs/container/bsui-img-new

To update images:

```bash
# Login (if needed)
echo $GITHUB_TOKEN | docker login ghcr.io -u bondada-a --password-stdin

# Build
docker build -f docker/bsui/Dockerfile -t bsui-img-new docker/bsui/

# Tag
docker tag bsui-img-new ghcr.io/bondada-a/bsui-img-new:latest

# Push
docker push ghcr.io/bondada-a/bsui-img-new:latest
```

## 📖 Additional Documentation

- **Local Setup Only**: [BLUESKY_LOCAL_SETUP.md](BLUESKY_LOCAL_SETUP.md)
- **Quick Reference**: [BLUESKY_QUICKSTART.md](BLUESKY_QUICKSTART.md)
- **Docker README**: [docker/bsui/README.md](docker/bsui/README.md)
- **Common Image README**: [docker/erobs-common-img/README.md](docker/erobs-common-img/README.md)

---

**Last Updated**: 2025-12-02
**Images Published**: 2025-12-02
**Maintained by**: bondada-a
