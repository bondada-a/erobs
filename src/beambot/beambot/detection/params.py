"""Detection parameter dataclasses."""

from dataclasses import dataclass


@dataclass
class CircleDetectionParams:
    """Parameters for Hough circle detection.

    All radius values are in PIXELS, not mm.
    Typical values for a 10mm wafer:
      - At ~300mm distance: ~50-70 pixels
      - At ~500mm distance: ~30-50 pixels
      - At ~800mm distance: ~20-30 pixels
    """
    min_radius: int = 15       # Min circle radius in pixels
    max_radius: int = 100      # Max circle radius in pixels
    blur_kernel: int = 5       # Gaussian blur kernel size
    param1: int = 50           # Canny edge detection threshold
    param2: int = 25           # Accumulator threshold (lower = more sensitive)
    min_dist: int = 50         # Min distance between detected circles
    search_radius: int = 10    # Pixels to search for valid depth around center


@dataclass
class ContourDetectionParams:
    """Parameters for contour-based object detection.

    Detects ANY shaped object (circles, squares, irregular shapes) by finding
    closed contours and filtering by area. More flexible than Hough circles.

    Area values are in PIXELS², not mm².
    Typical values (depends on camera distance and object size):
      - Small sample (~10mm) at 300mm: ~2000-8000 px²
      - Small sample (~10mm) at 500mm: ~700-3000 px²
      - Adjust based on your setup
    """
    min_area: int = 500          # Min contour area in pixels²
    max_area: int = 50000        # Max contour area in pixels²
    blur_kernel: int = 5         # Gaussian blur kernel size (must be odd)
    canny_low: int = 50          # Canny edge detection lower threshold
    canny_high: int = 150        # Canny edge detection upper threshold
    search_radius: int = 10      # Pixels to search for valid depth around centroid
    approx_epsilon: float = 0.02 # Contour approximation epsilon (fraction of perimeter)
    row_tolerance: int = 50      # Y-pixel tolerance for grouping into rows (for sorting)


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
