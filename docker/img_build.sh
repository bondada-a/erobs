#!/bin/bash
# Build and push beambot Docker images to GHCR
# Usage: ./img_build.sh [beambot_img|beambot_bsui|both]

set -e

REGISTRY="ghcr.io/bondada-a"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$REPO_ROOT"

build_beambot_img() {
    echo "=========================================="
    echo "Building beambot_img (ROS/robotics)..."
    echo "=========================================="
    docker build --no-cache -f docker/erobs-common-img/Dockerfile -t beambot_img .
    docker tag beambot_img "$REGISTRY/beambot_img:latest"
    echo ""
    echo "Pushing beambot_img to GHCR..."
    docker push "$REGISTRY/beambot_img:latest"
    echo "✓ beambot_img pushed to $REGISTRY/beambot_img:latest"
}

build_beambot_bsui() {
    echo "=========================================="
    echo "Building beambot_bsui (Bluesky)..."
    echo "=========================================="
    docker build --no-cache -f docker/bsui/Dockerfile -t beambot_bsui .
    docker tag beambot_bsui "$REGISTRY/beambot_bsui:latest"
    echo ""
    echo "Pushing beambot_bsui to GHCR..."
    docker push "$REGISTRY/beambot_bsui:latest"
    echo "✓ beambot_bsui pushed to $REGISTRY/beambot_bsui:latest"
}

case "${1:-both}" in
    beambot_img)
        build_beambot_img
        ;;
    beambot_bsui)
        build_beambot_bsui
        ;;
    both)
        build_beambot_img
        build_beambot_bsui
        ;;
    *)
        echo "Usage: $0 [beambot_img|beambot_bsui|both]"
        echo ""
        echo "  beambot_img   - Build ROS/robotics image (erobs-common-img)"
        echo "  beambot_bsui  - Build Bluesky image"
        echo "  both          - Build both images (default)"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "Done!"
echo "=========================================="
