"""Tests for beambot.detection.algorithms — pure OpenCV/NumPy functions."""

import struct
from unittest.mock import MagicMock

import numpy as np

from beambot.detection.algorithms import get_3d_position


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
