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

from dataclasses import dataclass
from importlib import import_module
from typing import TYPE_CHECKING

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose

if TYPE_CHECKING:
    from types import ModuleType


@dataclass
class DetectionResult:
    """Result of marker detection with capture timestamp.

    Shared across camera backends. The capture_stamp represents when the
    frame was captured (used for TF lookups at capture time rather than
    processing time).
    """
    markers: list[tuple[int, Pose]]
    capture_stamp: TimeMsg | None


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
        ImportError: If the camera backend's dependencies are not installed
            (e.g. zivid_interfaces missing on a beamline without Zivid).
    """
    if camera_type not in _CAMERA_MODULES:
        supported = ", ".join(_CAMERA_MODULES.keys())
        raise ValueError(
            f"Unsupported camera type: '{camera_type}'. "
            f"Supported types: {supported}"
        )

    try:
        return import_module(_CAMERA_MODULES[camera_type])
    except ImportError as e:
        raise ImportError(
            f"Camera backend '{camera_type}' is not available: {e}. "
            f"Install the required packages or select a different camera "
            f"in the beamline config."
        ) from e
