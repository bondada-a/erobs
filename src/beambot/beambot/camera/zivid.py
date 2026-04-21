"""Zivid camera wrapper for ArUco marker, circle, and contour detection.

This module provides wrappers around the Zivid camera's services,
exposing a simple interface that matches the beambot camera abstraction.

Interface contract:
    - create_client(node) -> ServiceClient
    - detect_markers(client, node, marker_ids, dictionary, timeout) -> list[tuple[int, Pose]]
    - detect_circles(node, timeout, params) -> list[Pose]
    - detect_contours(node, timeout, params) -> list[Pose]
"""

import time
from dataclasses import dataclass

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2

from zivid_interfaces.srv import CaptureAndDetectMarkers

from beambot.detection import (
    CircleDetectionParams,
    ContourDetectionParams,
    detect_hough_circles,
    detect_contours_in_image,
    get_3d_position,
)


@dataclass
class DetectionResult:
    """Result of marker detection with capture timestamp.

    The capture_stamp is the timestamp from the point cloud message header,
    representing when the Zivid camera actually captured the image.
    This should be used for TF lookups to ensure the transform matches
    the robot pose at capture time, not at processing time.
    """
    markers: list[tuple[int, Pose]]  # List of (marker_id, pose) tuples
    capture_stamp: TimeMsg | None  # Timestamp when image was captured


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
    marker_ids: list[int] | None = None,
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

    # Settle: wait for robot vibration to dampen before capturing timestamp
    if settle_time > 0:
        logger.debug(f"Settling for {settle_time:.2f}s before capture...")
        time.sleep(settle_time)

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
# Shared Capture Helper
# ============================================================================

# Match Zivid's default QoS: RELIABLE, VOLATILE, KEEP_LAST
_ZIVID_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _capture_image_and_cloud(
    node: Node,
    timeout: float = 45.0,
    label: str = "detection",
) -> tuple | None:
    """Trigger a Zivid capture and wait for RGB image + point cloud.

    Handles the full subscription lifecycle: create temporary subscriptions,
    wait for publisher discovery, trigger capture via the marker-detection
    service (which publishes image + cloud as a side effect), wait for data,
    then clean up.

    Args:
        node: ROS2 node for subscriptions and service calls
        timeout: Capture timeout in seconds
        label: Human-readable label for log messages (e.g. "circle detection")

    Returns:
        (rgb_image, cloud) tuple on success, or None on failure.
        rgb_image is an np.ndarray (RGB8), cloud is a PointCloud2 message.
    """
    logger = node.get_logger()
    bridge = CvBridge()

    # Storage for received data (using list for mutability in closure)
    received_image: list[Image | None] = [None]
    received_cloud: list[PointCloud2 | None] = [None]

    def on_image(msg: Image):
        received_image[0] = msg
        logger.debug("Image callback triggered")

    def on_cloud(msg: PointCloud2):
        received_cloud[0] = msg
        logger.debug("Point cloud callback triggered")

    image_sub = None
    cloud_sub = None
    marker_client = None

    try:
        image_sub = node.create_subscription(Image, IMAGE_TOPIC, on_image, _ZIVID_QOS)
        cloud_sub = node.create_subscription(PointCloud2, CLOUD_TOPIC, on_cloud, _ZIVID_QOS)
        marker_client = node.create_client(CaptureAndDetectMarkers, SERVICE_NAME)

        # Wait for service
        if not marker_client.wait_for_service(timeout_sec=2.0):
            logger.error(f"Zivid service '{SERVICE_NAME}' not available")
            return None

        # Wait for subscriptions to discover the publisher (up to 2s)
        logger.info("Waiting for subscriptions to connect...")
        for i in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
            if received_image[0] is not None or received_cloud[0] is not None:
                logger.info(f"Subscriptions connected after {(i+1)*0.1:.1f}s")
                break

        # Clear any stale data before triggering fresh capture
        received_image[0] = None
        received_cloud[0] = None

        # Trigger capture via marker detection service (we ignore the marker results).
        # This publishes image + point cloud to topics as a side effect.
        logger.info(f"Triggering Zivid capture for {label}...")
        request = CaptureAndDetectMarkers.Request()
        request.marker_ids = [999]  # Dummy ID — we don't care about markers
        request.marker_dictionary = "aruco4x4_50"

        future = marker_client.call_async(request)
        logger.info(f"Service call sent, waiting up to {timeout}s for response...")

        # spin_until_future_complete processes BOTH the service call
        # AND our subscription callbacks.
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)

        if not future.done():
            logger.error(f"Zivid capture timed out after {timeout}s")
            return None

        try:
            response = future.result()
            logger.info(f"Service responded: success={response.success}")
        except Exception as e:
            logger.error(f"Service call exception: {e}")
            return None

        # Wait for FRESH data to arrive via subscriptions.
        # Point cloud (~40MB) takes 3-4s longer to transmit than image (~10MB).
        logger.info("Waiting for image and point cloud data...")
        max_wait_iterations = 200  # Up to 20 seconds
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
            logger.error("Point cloud transmission may be slow - try increasing timeout")
            return None

        logger.info("Image and point cloud received")
        rgb_image = bridge.imgmsg_to_cv2(received_image[0], desired_encoding='rgb8')
        return (rgb_image, received_cloud[0])

    finally:
        if image_sub is not None:
            node.destroy_subscription(image_sub)
        if cloud_sub is not None:
            node.destroy_subscription(cloud_sub)
        if marker_client is not None:
            node.destroy_client(marker_client)


# ============================================================================
# Circle Detection
# ============================================================================


def detect_circles(
    node: Node,
    timeout: float = 45.0,
    params: CircleDetectionParams = None
) -> list[Pose]:
    """Detect circular objects using the Zivid camera.

    Captures an image and point cloud, runs Hough circle detection,
    and returns 3D poses for detected circles.

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

    result = _capture_image_and_cloud(node, timeout, label="circle detection")
    if result is None:
        return []

    rgb_image, cloud = result

    circles = detect_hough_circles(rgb_image, params)
    if circles is None or len(circles) == 0:
        logger.warning("No circles detected")
        return []

    logger.info(f"Detected {len(circles)} circle(s)")

    poses = []
    for cx, cy, radius in circles:
        xyz = get_3d_position(cloud, cx, cy, params.search_radius)
        if xyz is None:
            logger.warning(f"No valid depth at circle ({cx}, {cy})")
            continue

        x, y, z = xyz
        logger.info(f"Circle at ({cx}, {cy}) r={radius}px -> "
                    f"({x:.3f}, {y:.3f}, {z:.3f}) m")

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        poses.append(pose)

    return poses


# ============================================================================
# Contour Detection (Any Shape)
# ============================================================================

def detect_contours(
    node: Node,
    timeout: float = 45.0,
    params: ContourDetectionParams = None
) -> list[Pose]:
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

    result = _capture_image_and_cloud(node, timeout, label="contour detection")
    if result is None:
        return []

    rgb_image, cloud = result

    contours_info = detect_contours_in_image(rgb_image, params, logger)
    if contours_info is None or len(contours_info) == 0:
        logger.warning("No contours detected matching area criteria")
        return []

    logger.info(f"Detected {len(contours_info)} object(s), sorted in reading order")

    poses = []
    for i, (cx, cy, area, vertices) in enumerate(contours_info):
        sample_num = i + 1
        xyz = get_3d_position(cloud, cx, cy, params.search_radius)
        if xyz is None:
            logger.warning(f"Sample #{sample_num}: No valid depth at ({cx}, {cy})")
            continue

        x, y, z = xyz
        logger.info(f"Sample #{sample_num}: ({cx}, {cy}) area={area}px² -> "
                    f"({x:.3f}, {y:.3f}, {z:.3f}) m")

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        poses.append(pose)

    return poses


