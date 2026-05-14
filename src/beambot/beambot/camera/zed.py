"""ZED camera wrapper for beambot vision.

Unlike Zivid (single-shot, triggered), ZED streams continuously.
This module subscribes to ZED ROS2 topics and grabs the latest frame.

Interface contract (matches beambot camera abstraction):
    - create_client(node) -> None (no service client needed — streaming)
    - detect_markers(client, node, ...) -> DetectionResult

ZED 2i default topics (namespace: /zed/zed_node, SDK 5.2.1+):
    - /zed/zed_node/rgb/color/rect/image  (sensor_msgs/Image, bgra8)
    - /zed/zed_node/point_cloud/cloud_registered  (sensor_msgs/PointCloud2)
    - /zed/zed_node/depth/depth_registered  (sensor_msgs/Image, 32FC1)
"""

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2

from beambot.camera import DetectionResult
from beambot.detection import get_3d_position


# Default topic names (with zed2i default namespace)
IMAGE_TOPIC = "/zed/zed_node/rgb/color/rect/image"
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
) -> tuple[Image | None, PointCloud2 | None]:
    """Subscribe to ZED topics and grab the latest frame.

    Since ZED streams at ~15fps, we just need to wait for the next message.

    Args:
        node: ROS2 node for subscriptions
        timeout: Max seconds to wait for data
        need_cloud: Whether to also grab point cloud

    Returns:
        (image_msg, cloud_msg) tuple. cloud_msg may be None if not needed.
    """
    received_image: list[Image | None] = [None]
    received_cloud: list[PointCloud2 | None] = [None]

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
    marker_ids: list[int] | None = None,
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
            logger.warning("No markers detected by ZED")
        else:
            logger.warning(f"No markers found for requested IDs: {marker_ids}")
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
                logger.warning(f"Marker {mid}: No valid depth at ({cx}, {cy})")
        else:
            logger.warning(f"Marker {mid}: No point cloud — 2D only")

        # Identity orientation (we don't have 3D pose estimation without camera intrinsics solve)
        pose.orientation.w = 1.0
        detected.append((mid, pose))

    return DetectionResult(markers=detected, capture_stamp=capture_stamp)
