"""Tests for beambot.detection.algorithms — pure OpenCV/NumPy functions."""

import math
import struct
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest

from beambot.detection.algorithms import (
    detect_hough_circles,
    detect_contours_in_image,
    sort_contours_reading_order,
    get_3d_position,
)
from beambot.detection.params import CircleDetectionParams, ContourDetectionParams


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def _make_circle_image(
    width=640, height=480,
    center=(320, 240), radius=50,
    bg_color=0, fg_color=255,
) -> np.ndarray:
    """Create a synthetic RGB image with a filled circle."""
    img = np.full((height, width, 3), bg_color, dtype=np.uint8)
    cv2.circle(img, center, radius, (fg_color, fg_color, fg_color), -1)
    return img


def _make_rect_image(
    width=640, height=480,
    rect_center=(320, 240), rect_size=(80, 60),
    bg_color=0, fg_color=255,
) -> np.ndarray:
    """Create a synthetic RGB image with a filled rectangle."""
    img = np.full((height, width, 3), bg_color, dtype=np.uint8)
    x, y = rect_center
    w, h = rect_size
    cv2.rectangle(img, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2),
                  (fg_color, fg_color, fg_color), -1)
    return img


def _make_multi_rect_image(centers, rect_size=(40, 30)) -> np.ndarray:
    """Create image with multiple rectangles at given centers."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    for cx, cy in centers:
        w, h = rect_size
        cv2.rectangle(img, (cx - w // 2, cy - h // 2), (cx + w // 2, cy + h // 2),
                      (255, 255, 255), -1)
    return img


def _make_fake_cloud(width, height, xyz_data=None):
    """Create a minimal fake PointCloud2 message for testing get_3d_position.

    Args:
        width, height: Cloud dimensions
        xyz_data: Dict mapping (u, v) -> (x, y, z). All other points are NaN.
    """
    point_step = 16  # x(4) + y(4) + z(4) + rgba(4)
    row_step = width * point_step
    data = bytearray(height * row_step)

    # Fill with NaN by default
    nan_bytes = struct.pack("<fff", float('nan'), float('nan'), float('nan'))
    for v in range(height):
        for u in range(width):
            offset = v * row_step + u * point_step
            data[offset:offset + 12] = nan_bytes

    # Set specific points
    if xyz_data:
        for (u, v), (x, y, z) in xyz_data.items():
            offset = v * row_step + u * point_step
            data[offset:offset + 12] = struct.pack("<fff", x, y, z)

    cloud = MagicMock()
    cloud.width = width
    cloud.height = height
    cloud.point_step = point_step
    cloud.row_step = row_step
    cloud.data = bytes(data)
    return cloud


# ---------------------------------------------------------------------------
# Circle Detection
# ---------------------------------------------------------------------------

class TestDetectHoughCircles:

    def test_single_circle(self):
        img = _make_circle_image(center=(300, 200), radius=40)
        params = CircleDetectionParams(min_radius=20, max_radius=60, param2=20)
        result = detect_hough_circles(img, params)
        assert result is not None
        assert len(result) >= 1
        cx, cy, r = result[0]
        assert abs(cx - 300) < 15
        assert abs(cy - 200) < 15
        assert 25 < r < 55

    def test_no_circles(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        params = CircleDetectionParams(param2=50)
        result = detect_hough_circles(img, params)
        assert result is None

    def test_circle_too_small(self):
        img = _make_circle_image(radius=5)
        params = CircleDetectionParams(min_radius=20, max_radius=100)
        result = detect_hough_circles(img, params)
        assert result is None


# ---------------------------------------------------------------------------
# Contour Detection
# ---------------------------------------------------------------------------

class TestDetectContoursInImage:

    def test_single_rectangle(self):
        img = _make_rect_image(rect_center=(300, 200), rect_size=(80, 60))
        params = ContourDetectionParams(min_area=100, max_area=50000)
        result = detect_contours_in_image(img, params)
        assert result is not None
        assert len(result) >= 1
        cx, cy, area, vertices = result[0]
        assert abs(cx - 300) < 15
        assert abs(cy - 200) < 15
        assert area > 100

    def test_no_contours(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        params = ContourDetectionParams()
        result = detect_contours_in_image(img, params)
        assert result is None

    def test_area_filter_min(self):
        """Small object should be filtered by min_area."""
        img = _make_rect_image(rect_size=(5, 5))
        params = ContourDetectionParams(min_area=500)
        result = detect_contours_in_image(img, params)
        assert result is None

    def test_area_filter_max(self):
        """Large object should be filtered by max_area."""
        img = _make_rect_image(rect_size=(400, 300))
        params = ContourDetectionParams(max_area=1000)
        result = detect_contours_in_image(img, params)
        assert result is None

    def test_multiple_objects_sorted(self):
        """Multiple objects should be sorted in reading order."""
        centers = [(500, 100), (100, 100), (300, 300)]
        img = _make_multi_rect_image(centers)
        params = ContourDetectionParams(min_area=100, max_area=50000)
        result = detect_contours_in_image(img, params)
        assert result is not None
        assert len(result) >= 3
        # Reading order: top row L→R, then bottom row
        assert result[0][0] < result[1][0]  # First two in top row, sorted by x
        assert result[2][1] > result[0][1]  # Third is in lower row


# ---------------------------------------------------------------------------
# Sort Reading Order
# ---------------------------------------------------------------------------

class TestSortContoursReadingOrder:

    def test_empty(self):
        assert sort_contours_reading_order([]) == []

    def test_single_item(self):
        items = [(100, 200, 500, 4)]
        assert sort_contours_reading_order(items) == items

    def test_same_row(self):
        """Objects at similar Y should sort by X."""
        items = [(300, 100, 500, 4), (100, 105, 500, 4), (500, 98, 500, 4)]
        result = sort_contours_reading_order(items, row_tolerance=50)
        xs = [r[0] for r in result]
        assert xs == [100, 300, 500]

    def test_two_rows(self):
        """Objects in different rows sort top-to-bottom, then left-to-right."""
        items = [
            (400, 300, 500, 4),  # row 2, right
            (100, 300, 500, 4),  # row 2, left
            (300, 100, 500, 4),  # row 1, right
            (100, 100, 500, 4),  # row 1, left
        ]
        result = sort_contours_reading_order(items, row_tolerance=50)
        assert [(r[0], r[1]) for r in result] == [
            (100, 100), (300, 100),  # row 1
            (100, 300), (400, 300),  # row 2
        ]

    def test_row_tolerance(self):
        """Objects within tolerance should be in same row."""
        items = [(200, 100, 500, 4), (100, 130, 500, 4)]
        # tolerance=50: same row
        result = sort_contours_reading_order(items, row_tolerance=50)
        assert result[0][0] == 100  # sorted by X
        # tolerance=10: different rows
        result = sort_contours_reading_order(items, row_tolerance=10)
        assert result[0][1] == 100  # sorted by Y first


# ---------------------------------------------------------------------------
# 3D Position from Point Cloud
# ---------------------------------------------------------------------------

class TestGet3dPosition:

    def test_direct_hit(self):
        cloud = _make_fake_cloud(100, 100, xyz_data={(50, 50): (0.1, 0.2, 0.3)})
        result = get_3d_position(cloud, 50, 50)
        assert result is not None
        assert abs(result[0] - 0.1) < 1e-5
        assert abs(result[1] - 0.2) < 1e-5
        assert abs(result[2] - 0.3) < 1e-5

    def test_nan_returns_none(self):
        cloud = _make_fake_cloud(100, 100)  # All NaN
        result = get_3d_position(cloud, 50, 50, search_radius=0)
        assert result is None

    def test_search_radius_finds_nearby(self):
        """Center is NaN but neighbor has valid data."""
        cloud = _make_fake_cloud(100, 100, xyz_data={(52, 50): (0.5, 0.6, 0.7)})
        result = get_3d_position(cloud, 50, 50, search_radius=5)
        assert result is not None
        assert abs(result[0] - 0.5) < 1e-5

    def test_search_radius_too_small(self):
        """Neighbor is outside search radius."""
        cloud = _make_fake_cloud(100, 100, xyz_data={(60, 50): (0.5, 0.6, 0.7)})
        result = get_3d_position(cloud, 50, 50, search_radius=5)
        assert result is None

    def test_out_of_bounds(self):
        cloud = _make_fake_cloud(100, 100, xyz_data={(50, 50): (0.1, 0.2, 0.3)})
        result = get_3d_position(cloud, 200, 200)
        assert result is None

    def test_zero_point_rejected(self):
        """(0,0,0) points are treated as invalid."""
        cloud = _make_fake_cloud(100, 100, xyz_data={(50, 50): (0.0, 0.0, 0.0)})
        result = get_3d_position(cloud, 50, 50, search_radius=0)
        assert result is None
