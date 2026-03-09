"""ZED camera wrapper for beambot vision.

Unlike Zivid (single-shot, triggered), ZED streams continuously.
This module subscribes to ZED ROS2 topics and grabs the latest frame.

Interface contract (matches beambot camera abstraction):
    - create_client(node) -> None (no service client needed — streaming)
    - detect_markers(client, node, ...) -> DetectionResult
    - detect_circles(node, ...) -> List[Pose]
    - detect_contours(node, ...) -> List[Pose]

ZED 2i default topics (namespace: /zed/zed_node):
    - /zed/zed_node/rgb/image_rect_color  (sensor_msgs/Image, bgra8)
    - /zed/zed_node/point_cloud/cloud_registered  (sensor_msgs/PointCloud2)
    - /zed/zed_node/depth/depth_registered  (sensor_msgs/Image, 32FC1)
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2

from beambot.detection import (
    CircleDetectionParams,
    ContourDetectionParams,
    detect_hough_circles,
    detect_contours_in_image,
    get_3d_position,
)


@dataclass
class DetectionResult:
    """Result of detection with capture timestamp."""
    markers: List[Tuple[int, Pose]]
    capture_stamp: Optional[TimeMsg]


# Default topic names (with zed2i default namespace)
IMAGE_TOPIC = "/zed/zed_node/rgb/image_rect_color"
CLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered"

# ZED uses default ROS2 QoS
ZED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def create_client(node: Node):
    """No service client needed — ZED streams continuously.

    Returns None. Kept for interface compatibility with Zivid module.
    """
    return None


def _grab_latest(
    node: Node,
    timeout: float = 5.0,
    need_cloud: bool = True,
) -> Tuple[Optional[Image], Optional[PointCloud2]]:
    """Subscribe to ZED topics and grab the latest frame.

    Since ZED streams at ~15fps, we just need to wait for the next message.

    Args:
        node: ROS2 node for subscriptions
        timeout: Max seconds to wait for data
        need_cloud: Whether to also grab point cloud

    Returns:
        (image_msg, cloud_msg) tuple. cloud_msg may be None if not needed.
    """
    received_image: List[Optional[Image]] = [None]
    received_cloud: List[Optional[PointCloud2]] = [None]

    def on_image(msg: Image):
        received_image[0] = msg

    def on_cloud(msg: PointCloud2):
        received_cloud[0] = msg

    image_sub = node.create_subscription(Image, IMAGE_TOPIC, on_image, ZED_QOS)
    cloud_sub = node.create_subscription(PointCloud2, CLOUD_TOPIC, on_cloud, ZED_QOS) if need_cloud else None

    try:
        # ZED streams at 15fps, so we should get data within ~100ms
        max_iterations = int(timeout * 10)
        for i in range(max_iterations):
            rclpy.spin_once(node, timeout_sec=0.1)
            image_ready = received_image[0] is not None
            cloud_ready = received_cloud[0] is not None or not need_cloud
            if image_ready and cloud_ready:
                break

        return received_image[0], received_cloud[0]
    finally:
        node.destroy_subscription(image_sub)
        if cloud_sub is not None:
            node.destroy_subscription(cloud_sub)


def detect_markers(
    client,
    node: Node,
    marker_ids: Optional[List[int]] = None,
    dictionary: str = "aruco4x4_50",
    timeout: float = 10.0,
    settle_time: float = 0.0,
) -> DetectionResult:
    """Detect ArUco markers using OpenCV (no native SDK detection for ZED).

    Grabs the latest ZED frame and runs OpenCV ArUco detection + 3D lookup
    from the point cloud.

    Args:
        client: Unused (None). Kept for interface compatibility.
        node: ROS2 node for subscriptions
        marker_ids: List of marker IDs to detect, or None for all
        dictionary: ArUco dictionary name
        timeout: Detection timeout in seconds
        settle_time: Unused for ZED (streaming camera)

    Returns:
        DetectionResult with detected markers and capture timestamp.
    """
    import cv2
    logger = node.get_logger()
    bridge = CvBridge()

    image_msg, cloud_msg = _grab_latest(node, timeout=timeout, need_cloud=True)

    if image_msg is None:
        logger.error("No image received from ZED")
        return DetectionResult(markers=[], capture_stamp=None)

    capture_stamp = image_msg.header.stamp
    rgb_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')

    # Map dictionary name to OpenCV constant
    dict_map = {
        "aruco4x4_50": cv2.aruco.DICT_4X4_50,
        "aruco5x5_100": cv2.aruco.DICT_5X5_100,
        "aruco6x6_250": cv2.aruco.DICT_6X6_250,
    }
    dict_id = dict_map.get(dictionary, cv2.aruco.DICT_4X4_50)
    aruco_dict = cv2.aruco.Dictionary_get(dict_id)
    aruco_params = cv2.aruco.DetectorParameters_create()

    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    corners_list, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is None or len(ids) == 0:
        if marker_ids is None:
            logger.warn("No markers detected by ZED")
        else:
            logger.warn(f"No markers found for requested IDs: {marker_ids}")
        return DetectionResult(markers=[], capture_stamp=capture_stamp)

    detected = []
    for i, mid in enumerate(ids.flatten()):
        mid = int(mid)
        if marker_ids is not None and mid not in marker_ids:
            continue

        corners = corners_list[i][0]
        cx = int(corners[:, 0].mean())
        cy = int(corners[:, 1].mean())

        pose = Pose()
        if cloud_msg is not None:
            xyz = get_3d_position(cloud_msg, cx, cy)
            if xyz is not None:
                pose.position.x, pose.position.y, pose.position.z = xyz
                logger.info(
                    f"Marker {mid}: ZED OpenCV → "
                    f"({xyz[0]*1000:.2f}, {xyz[1]*1000:.2f}, {xyz[2]*1000:.2f}) mm"
                )
            else:
                logger.warn(f"Marker {mid}: No valid depth at ({cx}, {cy})")
        else:
            logger.warn(f"Marker {mid}: No point cloud — 2D only")

        # Identity orientation (we don't have 3D pose estimation without camera intrinsics solve)
        pose.orientation.w = 1.0
        detected.append((mid, pose))

    return DetectionResult(markers=detected, capture_stamp=capture_stamp)


def detect_circles(
    node: Node,
    timeout: float = 10.0,
    params: CircleDetectionParams = None,
) -> List[Pose]:
    """Detect circular objects using the ZED camera.

    Grabs latest frame, runs Hough circle detection, returns 3D poses.
    """
    if params is None:
        params = CircleDetectionParams()

    logger = node.get_logger()
    bridge = CvBridge()

    image_msg, cloud_msg = _grab_latest(node, timeout=timeout, need_cloud=True)

    if image_msg is None:
        logger.error("No image received from ZED")
        return []

    rgb_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')
    circles = detect_hough_circles(rgb_image, params)

    if circles is None or len(circles) == 0:
        logger.warn("No circles detected")
        return []

    logger.info(f"Detected {len(circles)} circle(s)")
    poses = []
    for cx, cy, radius in circles:
        if cloud_msg is None:
            continue
        xyz = get_3d_position(cloud_msg, cx, cy, params.search_radius)
        if xyz is None:
            logger.warn(f"No valid depth at circle ({cx}, {cy})")
            continue
        x, y, z = xyz
        logger.info(f"Circle at ({cx}, {cy}) r={radius}px -> ({x:.3f}, {y:.3f}, {z:.3f}) m")
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        poses.append(pose)

    return poses


def detect_contours(
    node: Node,
    timeout: float = 10.0,
    params: ContourDetectionParams = None,
) -> List[Pose]:
    """Detect objects of any shape using contour detection on ZED image."""
    if params is None:
        params = ContourDetectionParams()

    logger = node.get_logger()
    bridge = CvBridge()

    image_msg, cloud_msg = _grab_latest(node, timeout=timeout, need_cloud=True)

    if image_msg is None:
        logger.error("No image received from ZED")
        return []

    rgb_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')
    contours_info = detect_contours_in_image(rgb_image, params, logger)

    if contours_info is None or len(contours_info) == 0:
        logger.warn("No contours detected matching area criteria")
        return []

    logger.info(f"Detected {len(contours_info)} object(s)")
    poses = []
    for i, (cx, cy, area, vertices) in enumerate(contours_info):
        if cloud_msg is None:
            continue
        xyz = get_3d_position(cloud_msg, cx, cy, params.search_radius)
        if xyz is None:
            logger.warn(f"Sample #{i+1}: No valid depth at ({cx}, {cy})")
            continue
        x, y, z = xyz
        logger.info(f"Sample #{i+1}: ({cx}, {cy}) area={area}px² -> ({x:.3f}, {y:.3f}, {z:.3f}) m")
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        poses.append(pose)

    return poses
