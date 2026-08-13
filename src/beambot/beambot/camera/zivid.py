"""Zivid camera wrapper for ArUco marker and sample ROI detection.

This module provides wrappers around the Zivid camera's services,
exposing a simple interface that matches the beambot camera abstraction.

Interface contract:
    - create_client(node) -> ServiceClient
    - detect_markers(client, node, marker_ids, dictionary, timeout) -> list[tuple[int, Pose]]
    - detect_sample_roi(node, tag_id, ...) -> dict | None
"""

import time

from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2

from std_srvs.srv import Trigger
from zivid_interfaces.srv import CaptureAndDetectMarkers

import numpy as np

from beambot.camera import DetectionResult
from beambot.detection import (
    SampleRoiDetectionParams,
    detect_sample_in_roi,
    sample_roi_pickup_camera_xyz,
)


def _wait_for_future(future, timeout: float, poll_interval: float = 0.01) -> bool:
    """Poll future.done() without spinning. Returns True if completed."""
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() >= deadline:
            return False
        time.sleep(poll_interval)
    return True


# Zivid service endpoint
SERVICE_NAME = "/capture_and_detect_markers"
CAPTURE_2D_SERVICE = "/capture_2d"

# Topic names for image and point cloud
IMAGE_TOPIC = "/color/image_color"
CLOUD_TOPIC = "/points/xyzrgba"

# QoS for Zivid topics (volatile, reliable — matches publisher)
_ZIVID_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


def capture_2d(node: Node, timeout: float = 15.0) -> np.ndarray | None:
    """Trigger a 2D flash-lit capture and return the BGR image.

    Uses /capture_2d (no projector, camera LED flash only). This produces
    a color image suitable for color-based detection (e.g., spincoater
    pocket detection) without the structured-light glare of 3D captures.

    Args:
        node: ROS2 node for service calls and subscriptions.
        timeout: Maximum time to wait for the image (seconds).

    Returns:
        BGR numpy array, or None on failure.
    """
    bridge = CvBridge()
    received: list[Image | None] = [None]

    def on_image(msg: Image):
        received[0] = msg

    from rclpy.callback_groups import ReentrantCallbackGroup
    cb_group = ReentrantCallbackGroup()
    sub = node.create_subscription(
        Image, IMAGE_TOPIC, on_image, _ZIVID_QOS, callback_group=cb_group
    )
    client = node.create_client(
        Trigger, CAPTURE_2D_SERVICE, callback_group=cb_group
    )

    try:
        if not client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(
                f"Service '{CAPTURE_2D_SERVICE}' not available"
            )
            return None

        # Clear stale image
        time.sleep(0.5)
        received[0] = None

        future = client.call_async(Trigger.Request())
        if not _wait_for_future(future, timeout):
            node.get_logger().error("2D capture service timed out")
            return None

        result = future.result()
        if not result.success:
            node.get_logger().warning(f"2D capture failed: {result.message}")
            return None

        # Wait for image on topic — poll with short sleeps to allow
        # the executor to deliver the subscription callback
        deadline = time.monotonic() + timeout
        while received[0] is None and time.monotonic() < deadline:
            time.sleep(0.1)

        if received[0] is None:
            node.get_logger().error("No image received after 2D capture")
            return None

        return bridge.imgmsg_to_cv2(received[0], desired_encoding="bgr8")

    finally:
        node.destroy_subscription(sub)
        node.destroy_client(client)


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
    if not _wait_for_future(future, timeout):
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
# Sample ROI Detection (ArUco-anchored)
# ============================================================================


def detect_sample_roi(
    node: Node,
    tag_id: int,
    strategy: str = "farthest_edge",
    edge_inset_mm: float = 0.0,
    dictionary: str = "aruco4x4_50",
    timeout: float = 45.0,
    params: SampleRoiDetectionParams | None = None,
) -> tuple[Pose, object] | None:
    """Detect a sample in an ROI anchored to an ArUco tag and return 3D pickup pose.

    Self-contained: triggers a Zivid capture, detects the specified ArUco tag
    (extracting pixel corners AND the solvePnP pose from the service response),
    subscribes to the image topic, runs ROI-based sample detection, and
    projects the pickup pixel onto the marker's PnP plane (depth-free — the
    sample surface's own point-cloud depth is too sparse to trust).

    Args:
        node: ROS2 node for subscriptions and service calls
        tag_id: ArUco marker ID that anchors the ROI
        strategy: Pickup strategy — "center", "farthest_edge", "nearest_edge",
                  "farthest_corner", "nearest_corner"
        edge_inset_mm: Distance inward from edge toward center (mm)
        dictionary: ArUco dictionary name (default: "aruco4x4_50")
        timeout: Capture/detection timeout in seconds
        params: ROI geometry parameters (uses defaults if None)

    Returns:
        (pickup_pose_in_camera_frame, capture_stamp) on success, or None.
        The pose is in the Zivid optical frame (same as image/cloud).
    """
    if params is None:
        params = SampleRoiDetectionParams()

    logger = node.get_logger()
    bridge = CvBridge()

    # Storage for received data
    received_image: list[Image | None] = [None]
    received_cloud: list[PointCloud2 | None] = [None]

    def on_image(msg: Image):
        received_image[0] = msg

    def on_cloud(msg: PointCloud2):
        received_cloud[0] = msg

    image_sub = None
    cloud_sub = None
    marker_client = None

    try:
        image_sub = node.create_subscription(Image, IMAGE_TOPIC, on_image, _ZIVID_QOS)
        cloud_sub = node.create_subscription(PointCloud2, CLOUD_TOPIC, on_cloud, _ZIVID_QOS)
        marker_client = node.create_client(CaptureAndDetectMarkers, SERVICE_NAME)

        if not marker_client.wait_for_service(timeout_sec=2.0):
            logger.error(f"Zivid service '{SERVICE_NAME}' not available")
            return None

        # Wait for subscriptions to discover the publisher
        for i in range(20):
            time.sleep(0.1)
            if received_image[0] is not None or received_cloud[0] is not None:
                break

        # Clear stale data
        received_image[0] = None
        received_cloud[0] = None

        # Capture timestamp BEFORE service call (for accurate TF lookup)
        pre_capture_stamp = node.get_clock().now().to_msg()

        # Trigger capture with the actual tag_id
        request = CaptureAndDetectMarkers.Request()
        request.marker_ids = [tag_id]
        request.marker_dictionary = dictionary

        logger.info(f"Triggering Zivid capture for sample_roi detection (tag {tag_id})...")
        future = marker_client.call_async(request)

        if not _wait_for_future(future, timeout):
            logger.error(f"Zivid service timeout after {timeout}s")
            return None

        result = future.result()
        if not result.success:
            logger.warning(f"Zivid detection failed: {result.message}")
            return None

        # Find the requested marker in results
        target_marker = None
        for marker in result.detection_result.detected_markers:
            if marker.id == tag_id:
                target_marker = marker
                break

        if target_marker is None:
            logger.warning(f"Tag {tag_id} not detected in image")
            return None

        # Extract pixel corners from MarkerShape message
        marker_corners = np.array(
            [[p.x, p.y] for p in target_marker.corners_in_pixel_coordinates],
            dtype=float,
        )
        if (
            marker_corners.shape != (4, 2)
            or not np.isfinite(marker_corners).all()
        ):
            logger.warning(f"Tag {tag_id} returned invalid pixel geometry")
            return None

        # Compute px_per_mm from marker corner side lengths
        side_lengths = [
            np.linalg.norm(marker_corners[(i + 1) % 4] - marker_corners[i])
            for i in range(4)
        ]
        px_per_mm = np.mean(side_lengths) / params.marker_size_mm

        # Wait for image + cloud from topics
        logger.info("Waiting for image and point cloud data...")
        max_wait = 20.0
        start_wait = time.time()
        while time.time() - start_wait < max_wait:
            time.sleep(0.1)
            if received_image[0] is not None and received_cloud[0] is not None:
                break

        if received_image[0] is None or received_cloud[0] is None:
            img_status = "received" if received_image[0] else "MISSING"
            cloud_status = "received" if received_cloud[0] else "MISSING"
            logger.error(
                f"Timeout waiting for data (image: {img_status}, cloud: {cloud_status})"
            )
            return None

        bgr = bridge.imgmsg_to_cv2(received_image[0], desired_encoding='bgr8')
        # ponytail: sample_roi is now depth-free (marker-pose projection), so the
        # point cloud is unused here. Left the cloud subscription/wait in place;
        # drop it for a latency win if this path ever needs to be faster.

        # Run ROI-based sample detection
        detection = detect_sample_in_roi(
            bgr, marker_corners, px_per_mm,
            strategy=strategy,
            edge_inset_mm=edge_inset_mm,
            params=params,
        )

        if detection is None:
            logger.warning(f"No sample found in ROI near tag {tag_id}")
            return None

        pickup_px = detection["pickup_px"]
        logger.info(
            f"Sample detected: pickup=({pickup_px[0]}, {pickup_px[1]}), "
            f"size={detection['sample_size_mm']}mm, strategy={strategy}"
        )

        # Depth-free 3D: project the pickup pixel onto the marker's PnP plane
        # (target_marker.pose) — the SAME reliable pose the marker-move path
        # uses. The sample surface's own point-cloud depth is intermittent on
        # low-reflectivity wafers, which made the old cloud lookup wander up to
        # ~18mm capture-to-capture; the marker pose does not.
        mp = target_marker.pose
        pickup_xyz = sample_roi_pickup_camera_xyz(
            pickup_px,
            marker_corners,
            (mp.position.x, mp.position.y, mp.position.z),
            (mp.orientation.x, mp.orientation.y, mp.orientation.z, mp.orientation.w),
            px_per_mm,
            sample_thickness_mm=params.sample_thickness_mm,
        )
        if pickup_xyz is None:
            logger.error("Failed to compute pickup point from marker pose")
            return None

        pose = Pose()
        pose.position.x = pickup_xyz[0]
        pose.position.y = pickup_xyz[1]
        pose.position.z = pickup_xyz[2]
        pose.orientation.w = 1.0

        logger.info(
            f"Sample ROI pickup pose: ({pose.position.x:.4f}, "
            f"{pose.position.y:.4f}, {pose.position.z:.4f}) m in camera frame"
        )

        return (pose, pre_capture_stamp)

    finally:
        if image_sub is not None:
            node.destroy_subscription(image_sub)
        if cloud_sub is not None:
            node.destroy_subscription(cloud_sub)
        if marker_client is not None:
            node.destroy_client(marker_client)
