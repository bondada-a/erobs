"""Zivid camera wrapper for ArUco marker detection.

This module provides a thin wrapper around the Zivid camera's
CaptureAndDetectMarkers service, exposing a simple interface
that matches the beambot camera abstraction.

Interface contract:
    - create_client(node) -> ServiceClient
    - detect_markers(client, node, marker_ids, dictionary, timeout) -> List[Tuple[int, Pose]]
"""

from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

from zivid_interfaces.srv import CaptureAndDetectMarkers

# Zivid service endpoint
SERVICE_NAME = "/capture_and_detect_markers"


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
    marker_ids: List[int],
    dictionary: str = "aruco4x4_50",
    timeout: float = 10.0
) -> List[Tuple[int, Pose]]:
    """Detect ArUco markers using the Zivid camera.

    Args:
        client: Service client from create_client()
        node: ROS2 node for spinning
        marker_ids: List of marker IDs to detect
        dictionary: ArUco dictionary name (default: "aruco4x4_50")
        timeout: Detection timeout in seconds

    Returns:
        List of (marker_id, pose) tuples for detected markers.
        Empty list if detection failed or no markers found.
    """
    # Wait for service availability
    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().error(f"Zivid service '{SERVICE_NAME}' not available")
        return []

    # Build request
    request = CaptureAndDetectMarkers.Request()
    request.marker_ids = marker_ids
    request.marker_dictionary = dictionary

    # Call service asynchronously
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)

    # Check result
    if not future.done():
        node.get_logger().warn("Zivid detection service timeout")
        return []

    result = future.result()
    if not result.success:
        node.get_logger().warn(f"Zivid detection failed: {result.message}")
        return []

    # Extract detected markers
    detected = []
    for marker in result.detection_result.detected_markers:
        detected.append((marker.id, marker.pose))

    return detected
