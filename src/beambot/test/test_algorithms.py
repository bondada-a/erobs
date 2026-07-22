"""Tests for beambot.detection.algorithms — pure OpenCV/NumPy functions."""

import struct
from unittest.mock import MagicMock

import numpy as np
import pytest

from beambot.detection.algorithms import (
    detect_sample_in_roi,
    get_3d_position,
    sample_roi_pickup_camera_xyz,
)
from beambot.detection.params import SampleRoiDetectionParams


# A unit square marker in pixels: [TL, TR, BR, BL], 100px per edge.
_SQUARE = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=float)
_IDENTITY_Q = (0.0, 0.0, 0.0, 1.0)


def test_pickup_at_marker_center_returns_marker_position():
    # pickup == marker centre -> zero offset -> exactly the marker origin.
    xyz = sample_roi_pickup_camera_xyz(
        (150, 150), _SQUARE, (0.5, -0.1, 0.3), _IDENTITY_Q, px_per_mm=10.0
    )
    assert xyz == pytest.approx((0.5, -0.1, 0.3))


def test_pixel_offset_maps_to_metres_along_marker_x():
    # +20px along the TL->TR axis at 10 px/mm -> 2.0mm -> 0.002m in camera x.
    xyz = sample_roi_pickup_camera_xyz(
        (170, 150), _SQUARE, (0.5, -0.1, 0.3), _IDENTITY_Q, px_per_mm=10.0
    )
    assert xyz == pytest.approx((0.502, -0.1, 0.3))


def test_marker_yaw_rotates_offset_into_camera_frame():
    # 90deg about +z maps the marker x-offset onto camera +y.
    q_z90 = (0.0, 0.0, 0.70710678, 0.70710678)
    xyz = sample_roi_pickup_camera_xyz(
        (170, 150), _SQUARE, (0.5, -0.1, 0.3), q_z90, px_per_mm=10.0
    )
    assert xyz == pytest.approx((0.5, -0.098, 0.3), abs=1e-6)


def test_sample_thickness_offsets_along_marker_normal():
    # thickness lifts the point along -z of the marker frame (identity -> cam -z).
    xyz = sample_roi_pickup_camera_xyz(
        (150, 150), _SQUARE, (0.5, -0.1, 0.3), _IDENTITY_Q,
        px_per_mm=10.0, sample_thickness_mm=5.0,
    )
    assert xyz == pytest.approx((0.5, -0.1, 0.295))


def test_degenerate_input_returns_none():
    assert sample_roi_pickup_camera_xyz(
        (150, 150), _SQUARE[:3], (0, 0, 0), _IDENTITY_Q, px_per_mm=10.0
    ) is None
    assert sample_roi_pickup_camera_xyz(
        (150, 150), _SQUARE, (0, 0, 0), _IDENTITY_Q, px_per_mm=0.0
    ) is None
    assert sample_roi_pickup_camera_xyz(
        (150, 150), _SQUARE, (0, 0, 0), (0, 0, 0, 0), px_per_mm=10.0
    ) is None


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Sample ROI detection input validation and strategy selection
# ---------------------------------------------------------------------------

def _sample_roi_case():
    image = np.zeros((180, 180), dtype=np.uint8)
    image[90:111, 80:121] = 255
    corners = np.array([[20, 20], [40, 20], [40, 40], [20, 40]], dtype=float)
    params = SampleRoiDetectionParams(
        roi_offset_x_mm=70,
        roi_offset_y_mm=70,
        roi_width_mm=120,
        roi_height_mm=120,
        min_area=50,
    )
    return image, corners, params


@pytest.mark.parametrize(
    "corners",
    [
        np.zeros((3, 2)),
        np.full((4, 2), np.nan),
        np.array([[20, 20], [20, 20], [40, 40], [20, 40]]),
        np.array([[20, 20], [40, 20], [40, 40], [20, 20]]),
    ],
)
def test_sample_roi_rejects_invalid_marker_geometry(corners):
    image, _, params = _sample_roi_case()
    assert detect_sample_in_roi(image, corners, 1.0, params=params) is None


@pytest.mark.parametrize(
    "px_per_mm", [0.0, -1.0, float("nan"), float("inf")]
)
def test_sample_roi_rejects_invalid_px_per_mm(px_per_mm):
    image, corners, params = _sample_roi_case()
    assert detect_sample_in_roi(image, corners, px_per_mm, params=params) is None


@pytest.mark.parametrize(
    ("strategy", "edge_inset_mm", "param", "value"),
    [
        ("nearest_edge_typo", 6.5, None, None),
        ("corner", 6.5, None, None),
        ("nearest_edge", -1.0, None, None),
        ("nearest_edge", float("nan"), None, None),
        ("nearest_edge", 6.5, "roi_offset_x_mm", float("inf")),
        ("nearest_edge", 6.5, "roi_width_mm", 0.0),
    ],
)
def test_sample_roi_rejects_invalid_strategy_and_roi_config(
    strategy, edge_inset_mm, param, value
):
    image, corners, params = _sample_roi_case()
    if param is not None:
        setattr(params, param, value)

    assert (
        detect_sample_in_roi(
            image,
            corners,
            1.0,
            strategy=strategy,
            edge_inset_mm=edge_inset_mm,
            params=params,
        )
        is None
    )


def test_sample_roi_supported_strategies_select_the_intended_side():
    image, corners, params = _sample_roi_case()
    detections = {
        strategy: detect_sample_in_roi(
            image,
            corners,
            1.0,
            strategy=strategy,
            edge_inset_mm=0.0,
            params=params,
        )
        for strategy in (
            "center",
            "nearest_edge",
            "farthest_edge",
            "nearest_corner",
            "farthest_corner",
        )
    }

    assert all(detections.values())
    assert detections["center"]["pickup_px"] == detections["center"]["center_px"]
    center = detections["center"]["center_px"]
    assert detections["nearest_edge"]["pickup_px"][0] < center[0]
    assert detections["farthest_edge"]["pickup_px"][0] > center[0]
    assert all(
        near < middle
        for near, middle in zip(detections["nearest_corner"]["pickup_px"], center)
    )
    assert all(
        far > middle
        for far, middle in zip(detections["farthest_corner"]["pickup_px"], center)
    )
