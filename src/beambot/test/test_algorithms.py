"""Offline tests for detection geometry, model loading and image handling."""

import ast
import asyncio
import json
import struct
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest

from beambot.vision.perception.image_detection import (
    SampleRoiDetectionParams,
    detect_aruco_markers,
    detect_sample_in_roi,
    get_3d_position,
    sample_roi_pickup_camera_xyz,
)


# A unit square marker in pixels: [TL, TR, BR, BL], 100px per edge.
_SQUARE = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=float)
_IDENTITY_Q = (0.0, 0.0, 0.0, 1.0)


def test_yolo_preserves_constructor_model_and_device(monkeypatch, tmp_path):
    from beambot.vision.perception.yolo_object_detection import YoloModel

    model = MagicMock(return_value=[])
    load_model = MagicMock(return_value=model)
    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(YOLO=load_model))
    model_path = str(tmp_path / "custom.pt")
    detector = YoloModel(model_path, device="cpu")
    image = np.zeros((8, 8, 3), dtype=np.uint8)

    assert detector.detect(image) == []
    assert detector.detect(image) == []
    load_model.assert_called_once_with(model_path)
    model.to.assert_called_once_with("cpu")


def test_mcp_yolo_uses_bgr_images(monkeypatch, tmp_path):
    """Exercise the caller without importing the MCP server or starting ROS."""
    from beambot.vision.perception.yolo_object_detection import YoloDetectionParams

    path = Path(__file__).resolve().parents[1] / "mcp_server" / "beambot_mcp_server.py"
    function = next(
        node for node in ast.parse(path.read_text()).body
        if isinstance(node, ast.AsyncFunctionDef) and node.name == "detect_objects"
    )
    function.decorator_list = []
    rgb = np.full((8, 8, 3), (255, 0, 0), dtype=np.uint8)
    detector = MagicMock()
    detector.detect.return_value = [("sample", 0.9, 4, 4, 2, 2, 6, 6)]
    detector.annotate.side_effect = lambda image, detections: image.copy()
    save = MagicMock(return_value=True)
    monkeypatch.setattr(cv2, "imwrite", save)
    monkeypatch.setattr(Path, "mkdir", MagicMock())
    namespace = {
        "bridge": SimpleNamespace(node=None),
        "_resolve_camera_state": lambda node, camera: (rgb, None, None, "camera"),
        "get_yolo_model": lambda model: detector,
        "YoloDetectionParams": YoloDetectionParams,
        "DEFAULT_ANNOTATED_PATH": str(tmp_path / "annotated.jpg"),
        "cv2": cv2, "json": json, "Path": Path,
    }
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), "exec"), namespace)
    result = json.loads(asyncio.run(namespace["detect_objects"](method="yolo")))

    assert result["count"] == 1
    bgr = np.full_like(rgb, (0, 0, 255))
    np.testing.assert_array_equal(detector.detect.call_args.args[0], bgr)
    np.testing.assert_array_equal(detector.annotate.call_args.args[0], bgr)
    np.testing.assert_array_equal(save.call_args.args[1], bgr)
    np.testing.assert_array_equal(rgb, np.full_like(rgb, (255, 0, 0)))


@pytest.mark.parametrize("method", ["hsv_color", "marker"])
def test_mcp_image_detection_preserves_results_and_annotation(monkeypatch, method):
    path = Path(__file__).resolve().parents[1] / "mcp_server" / "beambot_mcp_server.py"
    functions = [
        node for node in ast.parse(path.read_text()).body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name in {"_annotate_image", "detect_objects"}
    ]
    for function in functions:
        function.decorator_list = []
    rgb = np.zeros((32, 32, 3), dtype=np.uint8)
    corners = [[8, 8], [24, 8], [24, 24], [8, 24]]
    save = MagicMock(return_value=True)
    monkeypatch.setattr(cv2, "imwrite", save)
    monkeypatch.setattr(Path, "mkdir", MagicMock())
    namespace = {
        "bridge": SimpleNamespace(node=None),
        "_resolve_camera_state": lambda node, camera: (rgb, object(), None, "camera"),
        "_detect_hsv_color": lambda image, **kwargs: [(16, 16, 256)],
        "_detect_aruco_markers": lambda image, **kwargs: [(16, 16, 7, corners)],
        "get_3d_position": lambda cloud, x, y: (0.1, 0.2, 0.3),
        "DEFAULT_ANNOTATED_PATH": "/tmp/beambot_detection.jpg",
        "cv2": cv2, "np": np, "json": json, "Path": Path, "Any": Any,
    }
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), "exec"), namespace)
    result = json.loads(asyncio.run(namespace["detect_objects"](
        method=method, transform_to_base=False,
    )))

    assert result["count"] == 1
    detection = result["detections"][0]
    assert (detection["pixel_x"], detection["pixel_y"]) == (16, 16)
    assert detection["camera_xyz"] == [0.1, 0.2, 0.3]
    assert detection["base_xyz"] is None
    if method == "marker":
        assert detection["marker_id"] == 7 and detection["corners"] == corners
    else:
        assert detection["area"] == 256
    save.assert_called_once()
    assert save.call_args.args[0] == result["annotated_image_path"]
    assert save.call_args.args[1].shape == rgb.shape
    assert np.any(save.call_args.args[1]) and not np.any(rgb)


def test_spincoater_model_uses_package_share(monkeypatch, tmp_path):
    """Resolve weights independently of cwd and cache the loaded model."""
    from beambot.vision.perception import spincoater_detection

    share_path = Path(__file__).resolve().parents[1]
    model_path = share_path / "models" / "spincoater_sample_seg.pt"
    lookup = MagicMock(return_value=share_path)
    yolo = MagicMock()
    monkeypatch.setitem(
        sys.modules, "ament_index_python.packages",
        SimpleNamespace(get_package_share_path=lookup),
    )
    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(YOLO=yolo))
    monkeypatch.setattr(spincoater_detection, "_sample_model", None)
    monkeypatch.chdir(tmp_path)

    assert spincoater_detection._get_sample_model() is yolo.return_value
    assert spincoater_detection._get_sample_model() is yolo.return_value
    lookup.assert_called_once_with("beambot")
    yolo.assert_called_once_with(str(model_path))

    monkeypatch.setattr(spincoater_detection, "_sample_model", None)
    lookup.return_value = tmp_path
    with pytest.raises(FileNotFoundError) as error:
        spincoater_detection._get_sample_model()
    assert str(error.value) == (
        f"Spincoater sample model not found at {tmp_path / 'models' / model_path.name}"
    )
    yolo.assert_called_once()


@pytest.mark.parametrize("modern_api", [True, False])
@pytest.mark.parametrize("found", [True, False])
def test_aruco_api_compatibility(monkeypatch, modern_api, found):
    corners = (np.zeros((1, 4, 2), dtype=np.float32),) if found else ()
    ids = np.array([[7]], dtype=np.int32) if found else None
    detect = MagicMock(return_value=(corners, ids, ()))
    dictionary, parameters = object(), object()
    aruco = SimpleNamespace(
        Dictionary_get=MagicMock(return_value=dictionary),
        DetectorParameters_create=MagicMock(return_value=parameters),
        detectMarkers=detect,
    )
    if modern_api:
        aruco.getPredefinedDictionary = MagicMock(return_value=dictionary)
        aruco.DetectorParameters = MagicMock(return_value=parameters)
        aruco.ArucoDetector = MagicMock(
            return_value=SimpleNamespace(detectMarkers=detect)
        )
    monkeypatch.setattr(cv2, "aruco", aruco)
    image = np.full((8, 8, 3), (255, 0, 0), dtype=np.uint8)

    result_corners, result_ids = detect_aruco_markers(image, 42)

    assert result_corners is corners and result_ids is ids
    np.testing.assert_array_equal(detect.call_args.args[0], np.full((8, 8), 76))
    if modern_api:
        aruco.getPredefinedDictionary.assert_called_once_with(42)
        aruco.ArucoDetector.assert_called_once_with(dictionary, parameters)
        aruco.Dictionary_get.assert_not_called()
    else:
        aruco.Dictionary_get.assert_called_once_with(42)
        assert detect.call_args.args[1] is dictionary
        assert detect.call_args.kwargs["parameters"] is parameters


def test_aruco_detects_synthetic_marker():
    dictionary_id = cv2.aruco.DICT_4X4_50
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, 7, 100)
    else:
        marker = cv2.aruco.drawMarker(dictionary, 7, 100)
    image = np.full((160, 160), 255, dtype=np.uint8)
    image[30:130, 30:130] = marker

    corners, ids = detect_aruco_markers(
        cv2.cvtColor(image, cv2.COLOR_GRAY2RGB), dictionary_id
    )

    assert ids.tolist() == [[7]]
    assert len(corners) == 1 and corners[0].shape == (1, 4, 2)


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


def test_rotated_marker_axes_and_tilted_normal_map_to_camera_xyz():
    # Marker +X points down the image and +Y points left. Both pixel offsets
    # are 20 px = 2 mm. A 90-degree X rotation sends +Y to +Z and -Z to +Y.
    corners = np.array([[200, 100], [200, 200], [100, 200], [100, 100]])
    xyz = sample_roi_pickup_camera_xyz(
        (130, 170), corners, (0.5, -0.1, 0.3),
        (0.70710678, 0.0, 0.0, 0.70710678),
        px_per_mm=10.0, sample_thickness_mm=5.0,
    )
    assert xyz == pytest.approx((0.502, -0.095, 0.302))


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

    def test_padded_rows_use_row_stride_and_truncated_data_returns_none(self):
        # Two 12-byte XYZ points per row, followed by eight padding bytes.
        cloud = SimpleNamespace(
            width=2, height=2, point_step=12, row_step=32,
            data=(b"\x00" * 44) + struct.pack("<fff", 0.4, -0.2, 0.7),
        )
        assert get_3d_position(cloud, 1, 1, search_radius=0) == pytest.approx(
            (0.4, -0.2, 0.7)
        )
        cloud.data = cloud.data[:-1]
        assert get_3d_position(cloud, 1, 1, search_radius=0) is None


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


@pytest.mark.parametrize(
    "strategy",
    ["center", "nearest_edge", "farthest_edge", "nearest_corner", "farthest_corner"],
)
def test_sample_roi_inset_moves_toward_center(strategy):
    image, corners, params = _sample_roi_case()
    original = detect_sample_in_roi(
        image, corners, 1.0, strategy=strategy, edge_inset_mm=0.0, params=params
    )
    inset = detect_sample_in_roi(
        image, corners, 1.0, strategy=strategy, edge_inset_mm=3.0, params=params
    )

    assert original is not None and inset is not None
    if strategy == "center":
        assert inset["pickup_px"] == original["pickup_px"]
        assert inset["offset_from_center_mm"] == 0.0
    else:
        assert inset["pickup_px"] != original["pickup_px"]
        assert inset["offset_from_center_mm"] == pytest.approx(
            original["offset_from_center_mm"] - 3.0, abs=0.1
        )
