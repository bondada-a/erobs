"""Shared detection algorithms for circle, contour, YOLO, and point cloud lookup.

Pure OpenCV + NumPy — no ROS dependencies.
YOLO detector requires ultralytics (optional, lazy-loaded).
"""

from beambot.detection.params import CircleDetectionParams, ContourDetectionParams
from beambot.detection.algorithms import (
    detect_hough_circles,
    detect_contours_in_image,
    sort_contours_reading_order,
    get_3d_position,
)
from beambot.detection.yolo_detector import (
    YoloDetector,
    YoloDetectionParams,
    get_detector as get_yolo_detector,
)

__all__ = [
    "CircleDetectionParams",
    "ContourDetectionParams",
    "detect_hough_circles",
    "detect_contours_in_image",
    "sort_contours_reading_order",
    "get_3d_position",
    "YoloDetector",
    "YoloDetectionParams",
    "get_yolo_detector",
]
