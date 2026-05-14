"""Pure detection algorithms — OpenCV + NumPy only, no ROS dependencies."""

from __future__ import annotations

import math
import struct
from typing import Any

import cv2
import numpy as np

from beambot.detection.params import SampleRoiDetectionParams


def get_3d_position(
    cloud,
    cx: int,
    cy: int,
    search_radius: int = 10,
) -> tuple[float, float, float] | None:
    """Get 3D position from organized point cloud at pixel (cx, cy).

    The point cloud is organized (same dimensions as image), so we can
    directly index into it using pixel coordinates.

    Args:
        cloud: Organized point cloud (PointCloud2 message)
        cx, cy: Pixel coordinates
        search_radius: Pixels to search for valid depth if center is invalid

    Returns:
        (x, y, z) tuple in meters, or None if no valid depth found
    """
    width = cloud.width
    height = cloud.height
    point_step = cloud.point_step

    def get_xyz_at(u: int, v: int):
        if u < 0 or u >= width or v < 0 or v >= height:
            return None
        offset = v * cloud.row_step + u * point_step
        try:
            x, y, z = struct.unpack_from("<fff", cloud.data, offset)
        except struct.error:
            return None
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            return None
        if x == 0.0 and y == 0.0 and z == 0.0:
            return None
        return (x, y, z)

    xyz = get_xyz_at(cx, cy)
    if xyz is not None:
        return xyz

    for r in range(1, search_radius + 1):
        for du in range(-r, r + 1):
            for dv in range(-r, r + 1):
                if abs(du) == r or abs(dv) == r:
                    xyz = get_xyz_at(cx + du, cy + dv)
                    if xyz is not None:
                        return xyz
    return None


def get_3d_position_averaged(
    cloud,
    cx: int,
    cy: int,
    search_radius: int = 10,
    min_points: int = 3,
) -> tuple[float, float, float] | None:
    """Get averaged 3D position from organized point cloud at pixel (cx, cy).

    Unlike get_3d_position which returns the FIRST valid nearby pixel's XYZ,
    this collects ALL valid pixels within search_radius and averages their XYZ.
    This gives a position estimate centered on the target pixel rather than
    biased toward whichever direction has valid depth first.

    Useful for dark surfaces where the exact pixel may lack depth but
    surrounding pixels have valid data.

    Args:
        cloud: Organized point cloud (PointCloud2 message)
        cx, cy: Target pixel coordinates
        search_radius: Pixels to search around center
        min_points: Minimum valid points required for a reliable average

    Returns:
        (x, y, z) average in meters, or None if fewer than min_points found
    """
    width = cloud.width
    height = cloud.height
    point_step = cloud.point_step

    points = []
    for dv in range(-search_radius, search_radius + 1):
        for du in range(-search_radius, search_radius + 1):
            u = cx + du
            v = cy + dv
            if u < 0 or u >= width or v < 0 or v >= height:
                continue
            offset = v * cloud.row_step + u * point_step
            try:
                x, y, z = struct.unpack_from("<fff", cloud.data, offset)
            except struct.error:
                continue
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue
            if x == 0.0 and y == 0.0 and z == 0.0:
                continue
            points.append((x, y, z))

    if len(points) < min_points:
        return None

    avg_x = sum(p[0] for p in points) / len(points)
    avg_y = sum(p[1] for p in points) / len(points)
    avg_z = sum(p[2] for p in points) / len(points)
    return (avg_x, avg_y, avg_z)


def detect_sample_in_roi(
    rgb_image: np.ndarray,
    marker_corners: np.ndarray,
    px_per_mm: float,
    strategy: str = "farthest_edge",
    edge_inset_mm: float = 4.0,
    params: SampleRoiDetectionParams | None = None,
) -> dict[str, Any] | None:
    """Detect a sample contour in a fixed ROI relative to an ArUco marker.

    Computes an ROI offset from the tag center in the marker's local axes,
    runs edge detection + contour finding in that ROI, then selects a pickup
    point based on the chosen strategy.

    Args:
        rgb_image: Full image (BGR or RGB)
        marker_corners: Shape (4, 2) pixel corners [TL, TR, BR, BL]
        px_per_mm: Pixel-to-mm scale (from known marker size)
        strategy: Pickup strategy — "center", "farthest_edge", "nearest_edge",
                  "farthest_corner", "nearest_corner"
        edge_inset_mm: Distance to move inward from edge toward center (mm)
        params: ROI geometry and CV pipeline parameters (uses defaults if None)

    Returns:
        Dict with pickup_px, center_px, sample_size_mm, sample_angle,
        offset_from_center_mm, roi, strategy, edge_inset_mm — or None if
        no sample found.
    """
    if params is None:
        params = SampleRoiDetectionParams()

    # Marker axes in pixel space
    top_left, top_right = marker_corners[0], marker_corners[1]
    bottom_left = marker_corners[3]
    marker_x = top_right - top_left
    marker_x = marker_x / np.linalg.norm(marker_x)
    marker_y = bottom_left - top_left
    marker_y = marker_y / np.linalg.norm(marker_y)
    tag_center = marker_corners.mean(axis=0)

    # ROI center in pixel space
    roi_center = (
        tag_center
        + marker_x * (params.roi_offset_x_mm * px_per_mm)
        + marker_y * (params.roi_offset_y_mm * px_per_mm)
    )

    half_w = (params.roi_width_mm * px_per_mm) / 2
    half_h = (params.roi_height_mm * px_per_mm) / 2

    h, w = rgb_image.shape[:2]
    roi_x1 = max(0, int(roi_center[0] - half_w))
    roi_y1 = max(0, int(roi_center[1] - half_h))
    roi_x2 = min(w, int(roi_center[0] + half_w))
    roi_y2 = min(h, int(roi_center[1] + half_h))

    roi = rgb_image[roi_y1:roi_y2, roi_x1:roi_x2]
    if roi.size == 0:
        return None

    # Convert to grayscale
    if len(roi.shape) == 3:
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    else:
        roi_gray = roi

    # Edge detection + contour finding
    blurred = cv2.GaussianBlur(roi_gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)
    contours, _ = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    # Filter by area and aspect ratio
    valid = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < params.min_area or area > params.max_area:
            continue
        rect = cv2.minAreaRect(c)
        rw, rh = rect[1]
        if rw == 0 or rh == 0:
            continue
        if max(rw, rh) / min(rw, rh) > params.max_aspect_ratio:
            continue
        valid.append((area, c))

    if not valid:
        return None

    # Select largest contour
    valid.sort(key=lambda x: x[0], reverse=True)
    sample_contour = valid[0][1]

    # Fit rotated rectangle
    rect = cv2.minAreaRect(sample_contour)
    rect_center_roi = rect[0]
    rect_size = rect[1]
    rect_angle = rect[2]
    rect_corners_roi = cv2.boxPoints(rect)

    # Convert to full image coordinates
    rect_center_full = np.array([
        rect_center_roi[0] + roi_x1,
        rect_center_roi[1] + roi_y1,
    ])
    rect_corners_full = rect_corners_roi + np.array([roi_x1, roi_y1])

    # Compute pickup point based on strategy
    tag_pt = tag_center
    center_pt = rect_center_full.copy()
    edge_inset_px = edge_inset_mm * px_per_mm

    if strategy == "center":
        pickup = center_pt.copy()
    elif "corner" in strategy:
        distances = [np.linalg.norm(c - tag_pt) for c in rect_corners_full]
        idx = np.argmax(distances) if "farthest" in strategy else np.argmin(distances)
        pickup = rect_corners_full[idx].copy()
        if edge_inset_px > 0:
            toward = center_pt - pickup
            norm = np.linalg.norm(toward)
            if norm > 0:
                pickup = pickup + toward / norm * edge_inset_px
    elif "edge" in strategy:
        midpoints = []
        for i in range(4):
            midpoints.append(
                (rect_corners_full[i] + rect_corners_full[(i + 1) % 4]) / 2
            )
        distances = [np.linalg.norm(m - tag_pt) for m in midpoints]
        idx = np.argmax(distances) if "farthest" in strategy else np.argmin(distances)
        pickup = midpoints[idx].copy()
        if edge_inset_px > 0:
            toward = center_pt - pickup
            norm = np.linalg.norm(toward)
            if norm > 0:
                pickup = pickup + toward / norm * edge_inset_px
    else:
        pickup = center_pt.copy()

    offset_from_center_mm = np.linalg.norm(pickup - center_pt) / px_per_mm

    return {
        "pickup_px": (int(pickup[0]), int(pickup[1])),
        "center_px": (int(center_pt[0]), int(center_pt[1])),
        "sample_size_mm": (
            round(rect_size[0] / px_per_mm, 1),
            round(rect_size[1] / px_per_mm, 1),
        ),
        "sample_angle": round(rect_angle, 1),
        "sample_area_px": int(cv2.contourArea(sample_contour)),
        "offset_from_center_mm": round(offset_from_center_mm, 1),
        "roi": (roi_x1, roi_y1, roi_x2, roi_y2),
        "strategy": strategy,
        "edge_inset_mm": edge_inset_mm,
    }
