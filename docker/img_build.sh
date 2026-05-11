#!/bin/bash
# Build and push beambot reference Docker images to GHCR.
# For the production ROS image (erobs-jazzy), use the docker-publish.yml
# workflow in GitHub Actions.
# Usage: ./img_build.sh [beambot_bsui|beambot_bsui_minimal|all]

set -e

REGISTRY="ghcr.io/bondada-a"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$REPO_ROOT"

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
    beambot_bsui)
        build_beambot_bsui
        ;;
    beambot_bsui_minimal)
        build_beambot_bsui_minimal
        ;;
    all)
        build_beambot_bsui
        build_beambot_bsui_minimal
        ;;
    *)
        echo "Usage: $0 [beambot_bsui|beambot_bsui_minimal|all]"
        echo ""
        echo "  beambot_bsui         - Build Bluesky image (full, ~5GB)"
        echo "  beambot_bsui_minimal - Build Bluesky image (lightweight, ~1.5GB)"
        echo "  all                  - Build all images (default)"
        echo ""
        echo "For erobs-jazzy (production ROS image), use the GitHub Actions"
        echo "workflow 'Build and Push Docker Images to GHCR'."
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "Done!"
echo "=========================================="
