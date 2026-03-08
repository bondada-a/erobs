"""Shared detection algorithms for circle, contour, and point cloud lookup.

Pure OpenCV + NumPy — no ROS dependencies.
"""

from beambot.detection.params import CircleDetectionParams, ContourDetectionParams
from beambot.detection.algorithms import (
    detect_hough_circles,
    detect_contours_in_image,
    sort_contours_reading_order,
    get_3d_position,
)

__all__ = [
    "CircleDetectionParams",
    "ContourDetectionParams",
    "detect_hough_circles",
    "detect_contours_in_image",
    "sort_contours_reading_order",
    "get_3d_position",
]
