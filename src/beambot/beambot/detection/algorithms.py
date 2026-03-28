"""Pure detection algorithms — OpenCV + NumPy only, no ROS dependencies."""

import math
import struct
from typing import List, Optional, Tuple

import cv2
import numpy as np

from beambot.detection.params import CircleDetectionParams, ContourDetectionParams


def detect_hough_circles(
    rgb_image: np.ndarray,
    params: CircleDetectionParams,
) -> Optional[List[Tuple[int, int, int]]]:
    """Detect circles in image using Hough Transform.

    Args:
        rgb_image: RGB image array
        params: Detection parameters

    Returns:
        List of (center_x, center_y, radius) tuples, or None if no circles found
    """
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (params.blur_kernel, params.blur_kernel), 2)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=params.min_dist,
        param1=params.param1,
        param2=params.param2,
        minRadius=params.min_radius,
        maxRadius=params.max_radius,
    )
    if circles is None:
        return None
    circles = np.uint16(np.around(circles))
    return [(int(c[0]), int(c[1]), int(c[2])) for c in circles[0]]


def sort_contours_reading_order(
    contours_info: List[Tuple[int, int, int, int]],
    row_tolerance: int = 50,
) -> List[Tuple[int, int, int, int]]:
    """Sort contours in reading order: left-to-right, top-to-bottom.

    Groups objects into rows based on Y-coordinate proximity,
    then sorts each row by X-coordinate.

    Args:
        contours_info: List of (cx, cy, area, vertices) tuples
        row_tolerance: Max Y-pixel difference for objects to be in same row

    Returns:
        Sorted list of contour info tuples
    """
    if not contours_info:
        return contours_info

    sorted_by_y = sorted(contours_info, key=lambda d: d[1])
    rows: List[List] = []
    current_row = [sorted_by_y[0]]
    current_row_y = sorted_by_y[0][1]

    for detection in sorted_by_y[1:]:
        cy = detection[1]
        if abs(cy - current_row_y) <= row_tolerance:
            current_row.append(detection)
        else:
            rows.append(current_row)
            current_row = [detection]
            current_row_y = cy
    rows.append(current_row)

    result = []
    for row in rows:
        result.extend(sorted(row, key=lambda d: d[0]))
    return result


def detect_contours_in_image(
    rgb_image: np.ndarray,
    params: ContourDetectionParams,
    logger=None,
) -> Optional[List[Tuple[int, int, int, int]]]:
    """Detect contours in image, filter by area, and sort in reading order.

    Objects are sorted left-to-right, top-to-bottom (reading order) so that
    sample_index=1 is the top-left object, index=2 is the next one to the right, etc.

    Args:
        rgb_image: RGB image array
        params: Detection parameters
        logger: Optional logger for debug output

    Returns:
        Sorted list of (centroid_x, centroid_y, area, num_vertices) tuples,
        or None if no valid contours found.
    """
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (params.blur_kernel, params.blur_kernel), 0)
    edges = cv2.Canny(blurred, params.canny_low, params.canny_high)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if logger:
        logger.debug(f"Found {len(contours)} raw contours")

    result = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < params.min_area or area > params.max_area:
            continue
        M = cv2.moments(contour)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, params.approx_epsilon * perimeter, True)
        result.append((cx, cy, int(area), len(approx)))

    if logger:
        logger.debug(f"Filtered to {len(result)} contours by area [{params.min_area}, {params.max_area}]")

    if not result:
        return None

    result = sort_contours_reading_order(result, params.row_tolerance)

    if logger:
        logger.debug(f"Sorted {len(result)} contours in reading order (row_tolerance={params.row_tolerance}px)")

    return result


def get_3d_position(
    cloud,
    cx: int,
    cy: int,
    search_radius: int = 10,
) -> Optional[Tuple[float, float, float]]:
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
) -> Optional[Tuple[float, float, float]]:
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
