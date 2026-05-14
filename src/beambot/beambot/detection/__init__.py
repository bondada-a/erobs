"""Shared detection algorithms for sample ROI, YOLO, and point cloud lookup.

Pure OpenCV + NumPy — no ROS dependencies.
YOLO detector requires ultralytics (optional, lazy-loaded).
"""

from beambot.detection.params import SampleRoiDetectionParams
from beambot.detection.algorithms import (
    detect_sample_in_roi,
    get_3d_position,
    get_3d_position_averaged,
)
from beambot.detection.yolo_detector import (
    YoloDetector,
    YoloDetectionParams,
    get_detector as get_yolo_detector,
)

__all__ = [
    "SampleRoiDetectionParams",
    "detect_sample_in_roi",
    "get_3d_position",
    "get_3d_position_averaged",
    "YoloDetector",
    "YoloDetectionParams",
    "get_yolo_detector",
]
