# docker/ — Container images for EROBS

Index of what lives here. For usage (pulls, run commands, architecture),
see [`docs/Container_documentation.md`](../docs/Container_documentation.md).

| Directory | Purpose | Status |
|---|---|---|
| `jazzy/` | Full ROS 2 Jazzy stack: MoveIt, UR driver, Zivid SDK, all erobs action servers, VNC for RViz. Published as `ghcr.io/bondada-a/erobs-jazzy`. | **Current production image.** Built by the `build-erobs-jazzy` job in [`.github/workflows/docker-publish.yml`](../.github/workflows/docker-publish.yml). |
| `bsui/` | Bluesky PoC — ROS Jazzy + EPICS + Miniconda + bluesky/ophyd + full erobs workspace. Published as `ghcr.io/bondada-a/beambot_bsui`. | Reference only. Not the current deployment path; rebuild on demand via the CI workflow dropdown. |
| `bsui-minimal/` | Lightweight Bluesky client — only `beambot_interfaces`, `bluesky_ros`, `cms/tasks`, plus bluesky/ophyd/EPICS. Designed to talk to `erobs-jazzy` over ROS 2 DDS. | Reference only. Not built by CI; build locally via `img_build.sh beambot_bsui_minimal`. |

## Building

```bash
# Production image (preferred: use the GitHub Actions workflow).
# For a local build:
docker build -f docker/jazzy/Dockerfile -t erobs-jazzy:latest .

# Reference Bluesky images (local):
./docker/img_build.sh beambot_bsui
./docker/img_build.sh beambot_bsui_minimal
```

The `docker/jazzy/Dockerfile` clones `erobs` from GitHub at build time
(branch: `jazzy_dev`). To force a fresh clone that picks up new commits:

```bash
docker build -f docker/jazzy/Dockerfile \
  --build-arg CACHEBUST=$(date +%s) \
  -t erobs-jazzy:latest .
```

## Smoke test

After building, run [`scripts/jazzy-smoke-test.sh`](../scripts/jazzy-smoke-test.sh)
to verify the image has a working ROS 2 environment and all erobs packages.
