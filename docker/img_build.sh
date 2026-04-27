#!/bin/bash
# Build and push beambot Docker images to GHCR
# Usage: ./img_build.sh [beambot_img|beambot_bsui|beambot_bsui_minimal|all]

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

build_beambot_img_v2() {
    echo "=========================================="
    echo "Building beambot_img_v2 (ROS/robotics)..."
    echo "=========================================="
    docker build --no-cache -f docker/erobs-common-img/Dockerfile -t beambot_img_v2 .
    docker tag beambot_img_v2 "$REGISTRY/beambot_img_v2:latest"
    echo ""
    echo "Pushing beambot_img_v2 to GHCR..."
    docker push "$REGISTRY/beambot_img_v2:latest"
    echo "✓ beambot_img_v2 pushed to $REGISTRY/beambot_img_v2:latest"
}

build_beambot_bsui() {
    echo "=========================================="
    echo "Building beambot_bsui (Bluesky - full)..."
    echo "=========================================="
    docker build --no-cache -f docker/bsui/Dockerfile -t beambot_bsui .
    docker tag beambot_bsui "$REGISTRY/beambot_bsui:latest"
    echo ""
    echo "Pushing beambot_bsui to GHCR..."
    docker push "$REGISTRY/beambot_bsui:latest"
    echo "✓ beambot_bsui pushed to $REGISTRY/beambot_bsui:latest"
}

build_beambot_bsui_minimal() {
    echo "=========================================="
    echo "Building beambot_bsui_minimal (Bluesky - lightweight)..."
    echo "=========================================="
    docker build --no-cache -f docker/bsui-minimal/Dockerfile -t beambot_bsui_minimal .
    docker tag beambot_bsui_minimal "$REGISTRY/beambot_bsui_minimal:latest"
    echo ""
    echo "Pushing beambot_bsui_minimal to GHCR..."
    docker push "$REGISTRY/beambot_bsui_minimal:latest"
    echo "✓ beambot_bsui_minimal pushed to $REGISTRY/beambot_bsui_minimal:latest"
}

case "${1:-all}" in
    beambot_img)
        build_beambot_img
        ;;
    beambot_img_v2)
        build_beambot_img_v2
        ;;
    beambot_bsui)
        build_beambot_bsui
        ;;
    beambot_bsui_minimal)
        build_beambot_bsui_minimal
        ;;
    all)
        build_beambot_img
        build_beambot_img_v2
        build_beambot_bsui
        build_beambot_bsui_minimal
        ;;
    *)
        echo "Usage: $0 [beambot_img|beambot_img_v2|beambot_bsui|beambot_bsui_minimal|all]"
        echo ""
        echo "  beambot_img          - Build ROS/robotics image (erobs-common-img, legacy)"
        echo "  beambot_img_v2       - Build ROS/robotics image (erobs-common-img, latest)"
        echo "  beambot_bsui         - Build Bluesky image (full, ~5GB)"
        echo "  beambot_bsui_minimal - Build Bluesky image (lightweight, ~1.5GB)"
        echo "  all                  - Build all images (default)"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "Done!"
echo "=========================================="
