"""Zivid camera wrapper for ArUco marker, circle, and contour detection.

This module provides wrappers around the Zivid camera's services,
exposing a simple interface that matches the beambot camera abstraction.

Interface contract:
    - create_client(node) -> ServiceClient
    - detect_markers(client, node, marker_ids, dictionary, timeout) -> List[Tuple[int, Pose]]
    - detect_circles(node, timeout, params) -> List[Pose]
    - detect_contours(node, timeout, params) -> List[Pose]
"""

import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2

from zivid_interfaces.srv import CaptureAndDetectMarkers


@dataclass
class DetectionResult:
    """Result of marker detection with capture timestamp.

    The capture_stamp is the timestamp from the point cloud message header,
    representing when the Zivid camera actually captured the image.
    This should be used for TF lookups to ensure the transform matches
    the robot pose at capture time, not at processing time.
    """
    markers: List[Tuple[int, Pose]]  # List of (marker_id, pose) tuples
    capture_stamp: Optional[TimeMsg]  # Timestamp when image was captured


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


# Zivid service endpoint
SERVICE_NAME = "/capture_and_detect_markers"

# Topic names for image and point cloud
IMAGE_TOPIC = "/color/image_color"
CLOUD_TOPIC = "/points/xyzrgba"


def create_client(node: Node):
    """Create a Zivid capture service client.

    Args:
        node: ROS2 node to create client on

    Returns:
        Service client for CaptureAndDetectMarkers
    """
    return node.create_client(CaptureAndDetectMarkers, SERVICE_NAME)


def detect_markers(
    client,
    node: Node,
    marker_ids: Optional[List[int]] = None,
    dictionary: str = "aruco4x4_50",
    timeout: float = 45.0,
    settle_time: float = 0.0
) -> DetectionResult:
    """Detect ArUco markers using Zivid's native detection.

    Uses Zivid SDK's built-in ArUco detection which provides 3D poses
    directly from the point cloud.

    TIMESTAMP FIX: The Zivid ROS driver assigns timestamps AFTER capture/processing
    completes (~200-400ms late). We capture the timestamp BEFORE calling the service,
    ensuring TF lookups use the robot pose at actual capture time.

    Args:
        client: Service client from create_client()
        node: ROS2 node for spinning
        marker_ids: List of marker IDs to detect, or None to detect all markers
        dictionary: ArUco dictionary name (default: "aruco4x4_50")
        timeout: Detection timeout in seconds
        settle_time: Seconds to wait before capture for robot to settle (default: 0.0)

    Returns:
        DetectionResult containing:
        - markers: List of (marker_id, pose) tuples for detected markers
        - capture_stamp: Timestamp captured BEFORE service call (for accurate TF lookup)
        Returns empty markers list if detection failed.
    """
    logger = node.get_logger()

    if not client.wait_for_service(timeout_sec=2.0):
        logger.error(f"Zivid service '{SERVICE_NAME}' not available")
        return DetectionResult(markers=[], capture_stamp=None)

    # TIMESTAMP FIX: Capture timestamp BEFORE calling Zivid service
    pre_capture_stamp = node.get_clock().now().to_msg()
    logger.debug(f"Pre-capture timestamp: {pre_capture_stamp.sec}.{pre_capture_stamp.nanosec}")

    # Build and send request (triggers capture + detection)
    # Zivid service requires explicit marker IDs - it doesn't accept empty list
    # If marker_ids is None (detect all), send all possible IDs for the dictionary
    request = CaptureAndDetectMarkers.Request()
    if marker_ids is not None:
        request.marker_ids = marker_ids
    else:
        # ArUco 4x4_50 has markers 0-49, 5x5_100 has 0-99, etc.
        # Default to 0-49 which covers most use cases
        if "4x4_50" in dictionary:
            request.marker_ids = list(range(50))
        elif "5x5_100" in dictionary:
            request.marker_ids = list(range(100))
        elif "5x5_250" in dictionary:
            request.marker_ids = list(range(250))
        elif "6x6_250" in dictionary:
            request.marker_ids = list(range(250))
        else:
            # Fallback: request common range 0-49
            request.marker_ids = list(range(50))
        logger.debug(f"Detecting all markers (IDs 0-{len(request.marker_ids)-1})")
    request.marker_dictionary = dictionary

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)

    if not future.done():
        logger.warning("Zivid detection service timeout")
        return DetectionResult(markers=[], capture_stamp=None)

    result = future.result()
    if not result.success:
        logger.warning(f"Zivid detection failed: {result.message}")
        return DetectionResult(markers=[], capture_stamp=None)

    # Extract detected markers directly from Zivid's result
    # If marker_ids is None, include all detected markers
    detected = []
    for marker in result.detection_result.detected_markers:
        if marker_ids is None or marker.id in marker_ids:
            detected.append((marker.id, marker.pose))
            pos = marker.pose.position
            logger.info(f"Marker {marker.id}: Zivid native → "
                       f"({pos.x*1000:.2f}, {pos.y*1000:.2f}, {pos.z*1000:.2f}) mm")

    if not detected:
        if marker_ids is None:
            logger.warning("No markers detected")
        else:
            logger.warning(f"No markers found for requested IDs: {marker_ids}")

    return DetectionResult(markers=detected, capture_stamp=pre_capture_stamp)


# ============================================================================
# Circle Detection
# ============================================================================


def detect_circles(
    node: Node,
    timeout: float = 45.0,
    params: CircleDetectionParams = None
) -> List[Pose]:
    """Detect circular objects using the Zivid camera.

    Captures an image and point cloud, runs Hough circle detection,
    and returns 3D poses for detected circles.

    Uses the same pattern as ArUco detection: spin_until_future_complete()
    processes both the service call AND subscription callbacks.

    Args:
        node: ROS2 node for subscriptions and service calls
        timeout: Detection timeout in seconds
        params: Circle detection parameters (uses defaults if None)

    Returns:
        List of Pose objects for detected circles (in camera optical frame).
        The pose orientation is identity (flat surface facing camera).
        Empty list if detection failed or no circles found.
    """
    if params is None:
        params = CircleDetectionParams()

    logger = node.get_logger()
    bridge = CvBridge()

    # Storage for received data (using list for mutability in closure)
    received_image: List[Optional[Image]] = [None]
    received_cloud: List[Optional[PointCloud2]] = [None]

    def on_image(msg: Image):
        received_image[0] = msg
        logger.debug("Image callback triggered")

    def on_cloud(msg: PointCloud2):
        received_cloud[0] = msg
        logger.debug("Point cloud callback triggered")

    # Zivid driver uses rmw_qos_profile_default (RELIABLE reliability)
    # We must match this QoS profile for the subscription to work
    # Note: Point cloud is ~40MB and takes 3-4s longer to transmit than image
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

    # Match Zivid's default QoS: RELIABLE, VOLATILE, KEEP_LAST
    zivid_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    image_sub = node.create_subscription(Image, IMAGE_TOPIC, on_image, zivid_qos)
    cloud_sub = node.create_subscription(PointCloud2, CLOUD_TOPIC, on_cloud, zivid_qos)

    # Use the marker detection service - it does capture AND publishes to topics
    marker_client = node.create_client(CaptureAndDetectMarkers, SERVICE_NAME)

    try:
        # Wait for service
        if not marker_client.wait_for_service(timeout_sec=2.0):
            logger.error(f"Zivid service '{SERVICE_NAME}' not available")
            return []

        # CRITICAL: Wait for subscriptions to discover the publisher
        # The test script has persistent subs that are always connected
        # We need to give temporary subs time to connect
        logger.info("Waiting for subscriptions to connect...")
        for i in range(20):  # Up to 2 seconds
            rclpy.spin_once(node, timeout_sec=0.1)
            # Check if we got any data (even stale) - means we're connected
            if received_image[0] is not None or received_cloud[0] is not None:
                logger.info(f"Subscriptions connected after {(i+1)*0.1:.1f}s")
                break

        # Clear any stale data before triggering fresh capture
        received_image[0] = None
        received_cloud[0] = None

        # Trigger capture via marker detection service (we ignore the marker results)
        # This publishes image + point cloud to topics as a side effect
        logger.info("Triggering Zivid capture for circle detection...")
        request = CaptureAndDetectMarkers.Request()
        # Must provide at least one marker ID (Zivid service requires non-empty list)
        # Use ID 999 which likely doesn't exist - we don't care about marker results
        request.marker_ids = [999]
        request.marker_dictionary = "aruco4x4_50"

        future = marker_client.call_async(request)
        logger.info(f"Service call sent, waiting up to {timeout}s for response...")

        # spin_until_future_complete processes BOTH the service call
        # AND our subscription callbacks - same pattern as ArUco detection!
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)

        if not future.done():
            logger.error(f"Zivid capture timed out after {timeout}s - service future not completed")
            return []

        # Check service response
        try:
            response = future.result()
            logger.info(f"Service responded: success={response.success}")
        except Exception as e:
            logger.error(f"Service call exception: {e}")
            return []

        # Wait for FRESH data to arrive via subscriptions
        # CRITICAL: Point cloud (~40MB for 2448x2048x16 bytes) takes 3-4s longer
        # to transmit than the image (~10MB). We must wait long enough!
        logger.info("Waiting for image and point cloud data...")
        max_wait_iterations = 200  # Up to 20 seconds (slower depth engines need more time)
        for i in range(max_wait_iterations):
            rclpy.spin_once(node, timeout_sec=0.1)

            # Log progress every second
            if (i + 1) % 10 == 0:
                img_status = "✓" if received_image[0] else "waiting"
                cloud_status = "✓" if received_cloud[0] else "waiting"
                logger.info(f"  {(i+1)*0.1:.1f}s - image: {img_status}, cloud: {cloud_status}")

            if received_image[0] is not None and received_cloud[0] is not None:
                logger.info(f"Both image and point cloud received after {(i+1)*0.1:.1f}s")
                break

        if received_image[0] is None or received_cloud[0] is None:
            img_status = "received" if received_image[0] else "MISSING"
            cloud_status = "received" if received_cloud[0] else "MISSING"
            logger.error(f"Timeout waiting for data (image: {img_status}, cloud: {cloud_status})")
            logger.error("Point cloud transmission may be slow - try increasing timeout")
            return []

        logger.info("Image and point cloud received")

        # Convert image
        rgb_image = bridge.imgmsg_to_cv2(received_image[0], desired_encoding='rgb8')
        cloud = received_cloud[0]

        # Detect circles
        circles = _detect_hough_circles(rgb_image, params)
        if circles is None or len(circles) == 0:
            logger.warning("No circles detected")
            return []

        logger.info(f"Detected {len(circles)} circle(s)")

        # Convert to 3D poses
        poses = []
        for cx, cy, radius in circles:
            xyz = _get_3d_position(cloud, cx, cy, params.search_radius)
            if xyz is None:
                logger.warning(f"No valid depth at circle ({cx}, {cy})")
                continue

            x, y, z = xyz
            logger.info(f"Circle at ({cx}, {cy}) r={radius}px -> "
                        f"({x:.3f}, {y:.3f}, {z:.3f}) m")

            # Create pose (identity orientation = flat surface facing camera)
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            poses.append(pose)

        return poses

    finally:
        # Clean up subscriptions and client
        node.destroy_subscription(image_sub)
        node.destroy_subscription(cloud_sub)
        node.destroy_client(marker_client)


def _detect_hough_circles(
    rgb_image: np.ndarray,
    params: CircleDetectionParams
) -> Optional[List[Tuple[int, int, int]]]:
    """Detect circles in image using Hough Transform.

    Args:
        rgb_image: RGB image array
        params: Detection parameters

    Returns:
        List of (center_x, center_y, radius) tuples, or None if no circles found
    """
    # Convert to grayscale
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

    # Blur to reduce noise
    blurred = cv2.GaussianBlur(
        gray, (params.blur_kernel, params.blur_kernel), 2
    )

    # Detect circles using Hough Transform
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=params.min_dist,
        param1=params.param1,
        param2=params.param2,
        minRadius=params.min_radius,
        maxRadius=params.max_radius
    )

    if circles is None:
        return None

    # Convert to list of tuples
    circles = np.uint16(np.around(circles))
    result = []
    for c in circles[0]:
        result.append((int(c[0]), int(c[1]), int(c[2])))

    return result


def _get_3d_position(
    cloud: PointCloud2,
    cx: int,
    cy: int,
    search_radius: int = 10
) -> Optional[Tuple[float, float, float]]:
    """Get 3D position from organized point cloud at pixel (cx, cy).

    The point cloud is organized (same dimensions as image), so we can
    directly index into it using pixel coordinates.

    Args:
        cloud: Organized point cloud
        cx, cy: Pixel coordinates
        search_radius: Pixels to search for valid depth if center is invalid

    Returns:
        (x, y, z) tuple in meters, or None if no valid depth found
    """
    width = cloud.width
    height = cloud.height
    point_step = cloud.point_step

    def get_xyz_at(u: int, v: int) -> Optional[Tuple[float, float, float]]:
        """Extract XYZ at pixel (u, v)."""
        if u < 0 or u >= width or v < 0 or v >= height:
            return None

        offset = v * cloud.row_step + u * point_step

        try:
            x, y, z = struct.unpack_from('<fff', cloud.data, offset)
        except struct.error:
            return None

        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            return None

        if x == 0.0 and y == 0.0 and z == 0.0:
            return None

        return (x, y, z)

    # Try center first
    xyz = get_xyz_at(cx, cy)
    if xyz is not None:
        return xyz

    # Search in expanding squares around center
    for r in range(1, search_radius + 1):
        for du in range(-r, r + 1):
            for dv in range(-r, r + 1):
                if abs(du) == r or abs(dv) == r:
                    xyz = get_xyz_at(cx + du, cy + dv)
                    if xyz is not None:
                        return xyz

    return None


# ============================================================================
# Contour Detection (Any Shape)
# ============================================================================

def detect_contours(
    node: Node,
    timeout: float = 45.0,
    params: ContourDetectionParams = None
) -> List[Pose]:
    """Detect objects of ANY shape using contour detection.

    Captures an image and point cloud, runs edge detection + contour finding,
    filters by area, and returns 3D poses for detected objects.

    Unlike Hough circles, this works for squares, triangles, irregular shapes,
    or any closed boundary that meets the area criteria.

    Args:
        node: ROS2 node for subscriptions and service calls
        timeout: Detection timeout in seconds
        params: Contour detection parameters (uses defaults if None)

    Returns:
        List of Pose objects for detected objects (in camera optical frame).
        The pose orientation is identity (flat surface facing camera).
        Empty list if detection failed or no objects found.
    """
    if params is None:
        params = ContourDetectionParams()

    logger = node.get_logger()
    bridge = CvBridge()

    # Storage for received data (using list for mutability in closure)
    received_image: List[Optional[Image]] = [None]
    received_cloud: List[Optional[PointCloud2]] = [None]

    def on_image(msg: Image):
        received_image[0] = msg
        logger.debug("Image callback triggered")

    def on_cloud(msg: PointCloud2):
        received_cloud[0] = msg
        logger.debug("Point cloud callback triggered")

    # Zivid driver uses rmw_qos_profile_default (RELIABLE reliability)
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

    zivid_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    image_sub = node.create_subscription(Image, IMAGE_TOPIC, on_image, zivid_qos)
    cloud_sub = node.create_subscription(PointCloud2, CLOUD_TOPIC, on_cloud, zivid_qos)

    # Use the marker detection service - it does capture AND publishes to topics
    marker_client = node.create_client(CaptureAndDetectMarkers, SERVICE_NAME)

    try:
        # Wait for service
        if not marker_client.wait_for_service(timeout_sec=2.0):
            logger.error(f"Zivid service '{SERVICE_NAME}' not available")
            return []

        # Wait for subscriptions to discover the publisher
        logger.info("Waiting for subscriptions to connect...")
        for i in range(20):  # Up to 2 seconds
            rclpy.spin_once(node, timeout_sec=0.1)
            if received_image[0] is not None or received_cloud[0] is not None:
                logger.info(f"Subscriptions connected after {(i+1)*0.1:.1f}s")
                break

        # Clear any stale data before triggering fresh capture
        received_image[0] = None
        received_cloud[0] = None

        # Trigger capture via marker detection service
        logger.info("Triggering Zivid capture for contour detection...")
        request = CaptureAndDetectMarkers.Request()
        request.marker_ids = [999]  # Dummy ID - we don't care about markers
        request.marker_dictionary = "aruco4x4_50"

        future = marker_client.call_async(request)
        logger.info(f"Service call sent, waiting up to {timeout}s for response...")
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)

        if not future.done():
            logger.error(f"Zivid capture timed out after {timeout}s - service future not completed")
            return []

        # Check service response
        try:
            response = future.result()
            logger.info(f"Service responded: success={response.success}")
        except Exception as e:
            logger.error(f"Service call exception: {e}")
            return []

        # Wait for data to arrive
        logger.info("Waiting for image and point cloud data...")
        max_wait_iterations = 200  # Up to 20 seconds (slower depth engines need more time)
        for i in range(max_wait_iterations):
            rclpy.spin_once(node, timeout_sec=0.1)

            if (i + 1) % 10 == 0:
                img_status = "✓" if received_image[0] else "waiting"
                cloud_status = "✓" if received_cloud[0] else "waiting"
                logger.info(f"  {(i+1)*0.1:.1f}s - image: {img_status}, cloud: {cloud_status}")

            if received_image[0] is not None and received_cloud[0] is not None:
                logger.info(f"Both image and point cloud received after {(i+1)*0.1:.1f}s")
                break

        if received_image[0] is None or received_cloud[0] is None:
            img_status = "received" if received_image[0] else "MISSING"
            cloud_status = "received" if received_cloud[0] else "MISSING"
            logger.error(f"Timeout waiting for data (image: {img_status}, cloud: {cloud_status})")
            return []

        logger.info("Image and point cloud received")

        # Convert image
        rgb_image = bridge.imgmsg_to_cv2(received_image[0], desired_encoding='rgb8')
        cloud = received_cloud[0]

        # Detect contours
        contours_info = _detect_contours_in_image(rgb_image, params, logger)
        if contours_info is None or len(contours_info) == 0:
            logger.warning("No contours detected matching area criteria")
            return []

        logger.info(f"Detected {len(contours_info)} object(s), sorted in reading order")

        # Convert to 3D poses
        poses = []
        for i, (cx, cy, area, vertices) in enumerate(contours_info):
            sample_num = i + 1  # 1-indexed for user-facing sample selection
            xyz = _get_3d_position(cloud, cx, cy, params.search_radius)
            if xyz is None:
                logger.warning(f"Sample #{sample_num}: No valid depth at ({cx}, {cy})")
                continue

            x, y, z = xyz
            logger.info(f"Sample #{sample_num}: ({cx}, {cy}) area={area}px² -> "
                        f"({x:.3f}, {y:.3f}, {z:.3f}) m")

            # Create pose (identity orientation = flat surface facing camera)
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            poses.append(pose)

        return poses

    finally:
        # Clean up subscriptions and client
        node.destroy_subscription(image_sub)
        node.destroy_subscription(cloud_sub)
        node.destroy_client(marker_client)


def _sort_contours_reading_order(
    contours_info: List[Tuple[int, int, int, int]],
    row_tolerance: int = 50
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

    # Sort by Y first to process top-to-bottom
    sorted_by_y = sorted(contours_info, key=lambda d: d[1])  # d[1] = cy

    # Group into rows
    rows = []
    current_row = [sorted_by_y[0]]
    current_row_y = sorted_by_y[0][1]

    for detection in sorted_by_y[1:]:
        cy = detection[1]
        # If Y is close enough to current row, add to same row
        if abs(cy - current_row_y) <= row_tolerance:
            current_row.append(detection)
        else:
            # Start new row
            rows.append(current_row)
            current_row = [detection]
            current_row_y = cy

    # Don't forget the last row
    rows.append(current_row)

    # Sort each row by X (left to right) and flatten
    result = []
    for row in rows:
        row_sorted = sorted(row, key=lambda d: d[0])  # d[0] = cx
        result.extend(row_sorted)

    return result


def _detect_contours_in_image(
    rgb_image: np.ndarray,
    params: ContourDetectionParams,
    logger=None
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
    # Convert to grayscale
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

    # Blur to reduce noise
    blurred = cv2.GaussianBlur(
        gray, (params.blur_kernel, params.blur_kernel), 0
    )

    # Edge detection
    edges = cv2.Canny(blurred, params.canny_low, params.canny_high)

    # Find contours
    contours, _ = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if logger:
        logger.debug(f"Found {len(contours)} raw contours")

    # Filter by area and extract info
    result = []
    for contour in contours:
        area = cv2.contourArea(contour)

        # Filter by area
        if area < params.min_area or area > params.max_area:
            continue

        # Get centroid using moments
        M = cv2.moments(contour)
        if M['m00'] == 0:
            continue  # Skip degenerate contours

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # Approximate contour to count vertices (for shape classification if needed)
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, params.approx_epsilon * perimeter, True)
        num_vertices = len(approx)

        result.append((cx, cy, int(area), num_vertices))

    if logger:
        logger.debug(f"Filtered to {len(result)} contours by area [{params.min_area}, {params.max_area}]")

    if not result:
        return None

    # Sort in reading order (left-to-right, top-to-bottom)
    result = _sort_contours_reading_order(result, params.row_tolerance)

    if logger:
        logger.debug(f"Sorted {len(result)} contours in reading order (row_tolerance={params.row_tolerance}px)")

    return result
