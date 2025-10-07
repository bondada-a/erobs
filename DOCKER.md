# Docker Setup for EROBS

## Building Locally

```bash
# Build the image
docker build -t erobs:local .

# Run interactive shell
docker run -it --rm erobs:local
```

## Using from GHCR

After pushing to GitHub, the image will be available at:

```bash
# Pull the image
docker pull ghcr.io/<your-username>/erobs:latest

# Run with IPython for Bluesky
docker run -it --rm ghcr.io/<your-username>/erobs:latest ipython

# Run with ROS2 tools
docker run -it --rm ghcr.io/<your-username>/erobs:latest ros2 topic list
```

## Interactive Bluesky Session

```bash
docker run -it --rm ghcr.io/<your-username>/erobs:latest ipython

# In IPython:
import rclpy
from bluesky import RunEngine
import bluesky.plan_stubs as bps
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice

rclpy.init()
RE = RunEngine({})
mtc = MTCExecutionDevice(name="mtc_executor", robot_ip="192.168.56.101")

# Run tasks
RE(bps.abs_set(mtc, "/workspace/new_test_updated.json", wait=True))
```

## GitHub Actions Setup

The workflow automatically:
- Builds on push to main/zivid_integration branches
- Tags with version on git tags (v*)
- Pushes to ghcr.io

Make sure to enable GitHub Actions and set repository visibility to allow GHCR access.
