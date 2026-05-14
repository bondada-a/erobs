"""Detection parameter dataclasses."""

from dataclasses import dataclass


@dataclass
class SampleRoiDetectionParams:
    """Parameters for ArUco-tag-relative ROI sample detection.

    Calibration values derived from physical tag/sample grid layout.
    """
    # ROI geometry relative to tag center (mm)
    roi_offset_x_mm: float = 19.3
    roi_offset_y_mm: float = 0.3
    roi_width_mm: float = 22.1
    roi_height_mm: float = 21.8
    # Contour filtering
    min_area: int = 100
    max_area: int = 15000
    max_aspect_ratio: float = 3.0
    # Physical marker size for px_per_mm calculation
    marker_size_mm: float = 14.9
