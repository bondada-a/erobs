"""Camera abstraction module for beambot vision.

This module provides a camera-agnostic interface for object detection.
Camera implementations are selected based on beamline configuration.

Supported detection methods:
    - ArUco markers: detect_markers()
    - Circles (Hough): detect_circles()
    - Any shape (contour): detect_contours()

Usage:
    from beambot.camera import get_camera

    # Get camera wrapper based on config
    camera = get_camera("zivid")
    client = camera.create_client(node)

    # Detect ArUco markers
    markers = camera.detect_markers(client, node, marker_ids=[5, 10])

    # Detect circles
    circles = camera.detect_circles(node)

    # Detect any shape by area
    objects = camera.detect_contours(node)
"""

from importlib import import_module
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from types import ModuleType

# Registry of supported camera types
_CAMERA_MODULES = {
    "zivid": "beambot.camera.zivid",
    "zed": "beambot.camera.zed",
}


def get_camera(camera_type: str) -> "ModuleType":
    """Get camera wrapper module by type.

    Args:
        camera_type: Camera type from beamline config (e.g., "zivid")

    Returns:
        Camera module with create_client() and detect_markers() functions

    Raises:
        ValueError: If camera_type is not supported
    """
    if camera_type not in _CAMERA_MODULES:
        supported = ", ".join(_CAMERA_MODULES.keys())
        raise ValueError(
            f"Unsupported camera type: '{camera_type}'. "
            f"Supported types: {supported}"
        )

    return import_module(_CAMERA_MODULES[camera_type])
