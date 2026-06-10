#!/usr/bin/env python3
"""Repeatedly call /capture_and_detect_markers until all 30 tags are detected,
then save the Zivid 2D image with GUI-identical overlays drawn on it.

Mirrors the GUI path (mtc_gui/camera_panel.py + ros2_bridge.py):
  - detection via /capture_and_detect_markers (ids 0..49, dict aruco4x4_50)
  - image from /color/image_color (published by the detector after each capture)
  - overlay: green quad + green-filled "ID: N" label with black text
"""

import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage
from zivid_interfaces.srv import CaptureAndDetectMarkers

TARGET_COUNT = 30
MAX_TRIES = 10
OUT_PATH = "/home/aditya/work/github_ws/jazzy/marker_sheet_detections.png"


class MarkerSheetCapturer(Node):
    def __init__(self):
        super().__init__("marker_sheet_capturer")
        self._bridge = CvBridge()
        self._latest_image = None
        self.create_subscription(
            RosImage, "/color/image_color", self._on_image, 10
        )
        self._client = self.create_client(
            CaptureAndDetectMarkers, "/capture_and_detect_markers"
        )

    def _on_image(self, msg):
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"image convert failed: {e}")

    def detect_once(self, timeout_sec=15.0):
        """Call the service once; return list of detected_markers (or None)."""
        req = CaptureAndDetectMarkers.Request()
        req.marker_ids = list(range(50))
        req.marker_dictionary = "aruco4x4_50"
        future = self._client.call_async(req)
        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                self.get_logger().warn("service call timed out")
                return None
        resp = future.result()
        if resp is None or not resp.success:
            msg = getattr(resp, "message", "no response")
            self.get_logger().warn(f"detection failed: {msg}")
            return resp.detection_result.detected_markers if resp else None
        return resp.detection_result.detected_markers

    def grab_image_after_detect(self, timeout_sec=5.0):
        """Spin briefly so the post-capture /color/image_color frame arrives."""
        self._latest_image = None
        start = time.time()
        while rclpy.ok() and self._latest_image is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                break
        return self._latest_image


def draw_overlay(image, markers):
    """Reproduce camera_panel.py overlay exactly."""
    display = image.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX
    for marker in markers:
        corners = marker.corners_in_pixel_coordinates
        if len(corners) == 4:
            pts = np.array([[int(c.x), int(c.y)] for c in corners], np.int32)
            cv2.polylines(display, [pts.reshape(-1, 1, 2)], True, (0, 255, 0), 3)
            cx = int(sum(c.x for c in corners) / 4)
            cy = int(sum(c.y for c in corners) / 4)
            text = f"ID: {marker.id}"
            (tw, th), _ = cv2.getTextSize(text, font, 1.5, 3)
            cv2.rectangle(
                display, (cx - 10, cy - th - 10), (cx + tw + 10, cy + 10),
                (0, 255, 0), -1,
            )
            cv2.putText(display, text, (cx, cy), font, 1.5, (0, 0, 0), 3)
    return display


def main():
    rclpy.init()
    node = MarkerSheetCapturer()

    if not node._client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("/capture_and_detect_markers not available")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    best_markers = []
    best_image = None
    success = False

    for attempt in range(1, MAX_TRIES + 1):
        markers = node.detect_once()
        count = len(markers) if markers else 0
        ids = sorted(m.id for m in markers) if markers else []
        missing = [i for i in range(TARGET_COUNT) if i not in ids]
        node.get_logger().info(
            f"attempt {attempt}/{MAX_TRIES}: {count} markers"
            + (f", missing {missing}" if missing else "")
        )

        image = node.grab_image_after_detect()
        if markers and count > len(best_markers) and image is not None:
            best_markers, best_image = markers, image

        if count >= TARGET_COUNT and image is not None:
            best_markers, best_image = markers, image
            success = True
            node.get_logger().info(f"got all {count} markers on attempt {attempt}")
            break
        time.sleep(0.3)

    if best_image is None:
        node.get_logger().error("never received an image to draw on")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    overlay = draw_overlay(best_image, best_markers)
    cv2.imwrite(OUT_PATH, overlay)
    node.get_logger().info(
        f"saved {len(best_markers)}-marker image -> {OUT_PATH}"
        + ("" if success else "  (best effort, target not reached)")
    )

    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 2


if __name__ == "__main__":
    sys.exit(main())
