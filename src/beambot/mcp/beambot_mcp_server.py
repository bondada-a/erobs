#!/usr/bin/env python3
"""Beambot MCP Server — Custom tools for Zivid camera, detection, and TF.

Runs alongside ros-mcp-server to provide beambot-specific tools that handle
Zivid's single-shot capture quirks (QoS timing, large point cloud transfer)
and wrap multi-step vision workflows into single tool calls.

Architecture:
    FastMCP (async, stdio) → ROS2Bridge (background thread) → ROS2 topics/services/TF

The ROS2Bridge runs a persistent node with:
    - RELIABLE+VOLATILE subscriptions to Zivid image/cloud topics
    - A TF buffer that fills continuously
    - Service client for /capture trigger
This avoids the QoS timing race that makes subscribe_once fail with Zivid.
"""

import asyncio
import collections
import json
import logging
import math
import os
import re
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import yaml
import numpy as np
from mcp.server.fastmcp import FastMCP

from beambot.detection import (
    CircleDetectionParams,
    ContourDetectionParams,
    detect_hough_circles,
    detect_contours_in_image,
    get_3d_position,
    get_3d_position_averaged,
    YoloDetectionParams,
    get_yolo_detector,
)

# ROS2 imports — these require a sourced ROS2 environment
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import quaternion_matrix

# Zivid native marker detection (optional — only when zivid_interfaces is available)
try:
    from zivid_interfaces.srv import CaptureAndDetectMarkers
    _ZIVID_MARKER_AVAILABLE = True
except ImportError:
    _ZIVID_MARKER_AVAILABLE = False
from cv_bridge import CvBridge

from epick_msgs.msg import ObjectDetectionStatus

logger = logging.getLogger("beambot-mcp")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
# Zivid topics (single-shot, triggered)
ZIVID_IMAGE_TOPIC = "/color/image_color"
ZIVID_CLOUD_TOPIC = "/points/xyzrgba"
ZIVID_CAPTURE_SERVICE = "/capture"
ZIVID_MARKER_SERVICE = "/capture_and_detect_markers"
ZIVID_FRAME = "zivid_optical_frame"

# ZED topics (continuous streaming)
ZED_IMAGE_TOPIC = "/zed/zed_node/rgb/color/rect/image"
ZED_CLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered"
ZED_FRAME = "zed_left_camera_optical_frame"

ZIVID_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

ZED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Robot state topics
JOINT_STATES_TOPIC = "/joint_states"
CURRENT_GRIPPER_TOPIC = "/beambot/current_gripper"
EXECUTION_STATE_TOPIC = "/beambot/execution_state"
EPICK_STATUS_TOPIC = "/object_detection_status"
BEAMBOT_EXECUTION_ACTION = "/beambot_execution"

# ePick ObjectDetectionStatus integer → human-readable string
EPICK_STATUS_NAMES = {
    0: "UNKNOWN",
    1: "OBJECT_DETECTED_AT_MIN_PRESSURE",
    2: "OBJECT_DETECTED_AT_MAX_PRESSURE",
    3: "NO_OBJECT_DETECTED",
}

# Pose registry
POSES_FILE = os.environ.get(
    "EROBS_POSES_FILE",
    os.path.join(os.path.dirname(__file__), "..", "..", "cms", "poses.yaml"),
)

# Default save locations
DEFAULT_IMAGE_PATH = "/tmp/beambot_capture.jpg"
DEFAULT_ANNOTATED_PATH = "/tmp/beambot_detection.jpg"


def _detect_display() -> Tuple[str, str]:
    """Detect DISPLAY and XAUTHORITY for GUI subprocess.

    Claude Code strips DISPLAY from MCP server environments. We detect
    the active X11 display by checking the environment first, then
    falling back to querying the system.
    """
    display = os.environ.get("DISPLAY", "")
    xauth = os.environ.get("XAUTHORITY", "")

    if not display:
        # Try to find DISPLAY from any running user process
        import subprocess as _sp
        try:
            result = _sp.run(
                ["bash", "-c", "cat /proc/$(pgrep -u $USER -x gnome-shell || pgrep -u $USER -x Xwayland || echo 1)/environ 2>/dev/null | tr '\\0' '\\n' | grep ^DISPLAY= | head -1 | cut -d= -f2"],
                capture_output=True, text=True, timeout=3,
            )
            display = result.stdout.strip() or ":1"
        except Exception:
            display = ":1"

    if not xauth:
        # Common locations
        for candidate in [
            f"/run/user/{os.getuid()}/gdm/Xauthority",
            os.path.expanduser("~/.Xauthority"),
        ]:
            if os.path.exists(candidate):
                xauth = candidate
                break

    return display, xauth


_DETECTED_DISPLAY, _DETECTED_XAUTHORITY = _detect_display()



def _detect_hsv_color(
    rgb_image: np.ndarray,
    hue_low: int = 100,
    hue_high: int = 130,
    sat_min: int = 80,
    val_min: int = 80,
    min_area: int = 200,
) -> Optional[List[Tuple[int, int, int]]]:
    """Detect objects by HSV color range. Returns list of (cx, cy, area)."""
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    lower = np.array([hue_low, sat_min, val_min])
    upper = np.array([hue_high, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    # Morphological open to remove noise
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        M = cv2.moments(contour)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        result.append((cx, cy, int(area)))
    if not result:
        return None
    # Sort by area descending (largest first)
    result.sort(key=lambda x: x[2], reverse=True)
    return result


def _detect_aruco_markers(
    rgb_image: np.ndarray,
    marker_ids: Optional[List[int]] = None,
    dictionary_name: str = "DICT_4X4_50",
) -> Optional[List[Tuple[int, int, int, List]]]:
    """Detect ArUco markers in image. Returns list of (cx, cy, marker_id, corners)."""
    # Map string name to OpenCV constant
    aruco_dicts = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    }
    dict_id = aruco_dicts.get(dictionary_name, cv2.aruco.DICT_4X4_50)
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        corners_list, ids, _ = detector.detectMarkers(gray)
    except AttributeError:
        aruco_dict = cv2.aruco.Dictionary_get(dict_id)
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners_list, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is None or len(ids) == 0:
        return None

    result = []
    for i, mid in enumerate(ids.flatten()):
        if marker_ids is not None and int(mid) not in marker_ids:
            continue
        corners = corners_list[i][0]
        cx = int(np.mean(corners[:, 0]))
        cy = int(np.mean(corners[:, 1]))
        result.append((cx, cy, int(mid), corners.tolist()))

    return result if result else None


def _annotate_image(
    rgb_image: np.ndarray,
    detections: List[Dict[str, Any]],
    method: str,
) -> np.ndarray:
    """Draw detection results on image. Returns annotated BGR image for saving."""
    annotated = cv2.cvtColor(rgb_image.copy(), cv2.COLOR_RGB2BGR)
    for i, det in enumerate(detections):
        px, py = det["pixel_x"], det["pixel_y"]
        label_parts = [f"#{i + 1}"]
        if "marker_id" in det:
            label_parts = [f"ID:{det['marker_id']}"]
        if det.get("base_xyz"):
            bx, by, bz = det["base_xyz"]
            label_parts.append(f"({bx:.3f},{by:.3f},{bz:.3f})")
        elif det.get("camera_xyz"):
            cx, cy, cz = det["camera_xyz"]
            label_parts.append(f"cam({cx:.3f},{cy:.3f},{cz:.3f})")
        label = " ".join(label_parts)

        # Draw marker
        color = (0, 255, 0)
        if method == "circle" and "radius" in det:
            cv2.circle(annotated, (px, py), det["radius"], color, 2)
        else:
            cv2.circle(annotated, (px, py), 8, color, -1)
        cv2.circle(annotated, (px, py), 3, (0, 0, 255), -1)  # Red center dot

        # Label above
        cv2.putText(
            annotated, label, (px - 10, py - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2,
        )
        cv2.putText(
            annotated, label, (px - 10, py - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
        )
    return annotated


# ---------------------------------------------------------------------------
# ROS2 Bridge — persistent node running in background thread
# ---------------------------------------------------------------------------

class ROS2BridgeNode(Node):
    """Persistent ROS2 node with Zivid subscriptions and TF buffer."""

    def __init__(self):
        super().__init__("beambot_mcp_bridge")
        self._cb_group = ReentrantCallbackGroup()
        self._bridge = CvBridge()

        # Zivid subscriptions (persistent — avoids QoS timing race)
        self._image_sub = self.create_subscription(
            Image, ZIVID_IMAGE_TOPIC, self._on_zivid_image, ZIVID_QOS,
            callback_group=self._cb_group,
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2, ZIVID_CLOUD_TOPIC, self._on_zivid_cloud, ZIVID_QOS,
            callback_group=self._cb_group,
        )

        # Zivid capture service client
        self._capture_client = self.create_client(
            Trigger, ZIVID_CAPTURE_SERVICE, callback_group=self._cb_group,
        )

        # Zivid native marker detection client
        self._marker_detect_client = None
        if _ZIVID_MARKER_AVAILABLE:
            self._marker_detect_client = self.create_client(
                CaptureAndDetectMarkers, ZIVID_MARKER_SERVICE,
                callback_group=self._cb_group,
            )

        # ZED subscriptions (streaming — always has latest frame)
        self._zed_image_sub = self.create_subscription(
            Image, ZED_IMAGE_TOPIC, self._on_zed_image, ZED_QOS,
            callback_group=self._cb_group,
        )
        self._zed_cloud_sub = self.create_subscription(
            PointCloud2, ZED_CLOUD_TOPIC, self._on_zed_cloud, ZED_QOS,
            callback_group=self._cb_group,
        )

        # TF buffer (fills continuously via background executor)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Note: /rosout subscription removed — unreliable due to DDS discovery
        # timing with 50+ publishers. get_recent_logs now reads from the launch
        # output file (/tmp/beambot_launch.log) written by start_mcp.sh.

        # Cancel service client for stopping orchestrator goals
        # Uses the action's cancel service directly (cancel-all with empty request)
        self._cancel_client = self.create_client(
            CancelGoal, f"{BEAMBOT_EXECUTION_ACTION}/_action/cancel_goal",
            callback_group=self._cb_group,
        )

        # Robot state subscriptions
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._joint_states_sub = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_states, 10,
            callback_group=self._cb_group,
        )
        self._gripper_sub = self.create_subscription(
            String, CURRENT_GRIPPER_TOPIC, self._on_current_gripper, latched_qos,
            callback_group=self._cb_group,
        )
        self._exec_state_sub = self.create_subscription(
            String, EXECUTION_STATE_TOPIC, self._on_execution_state, 10,
            callback_group=self._cb_group,
        )

        # ePick vacuum object detection status
        self._epick_status_sub = self.create_subscription(
            ObjectDetectionStatus, EPICK_STATUS_TOPIC,
            self._on_epick_status, 10,
            callback_group=self._cb_group,
        )

        # Robot state (updated by callbacks, None = no data received yet)
        self.joint_names: Optional[List[str]] = None
        self.joint_positions: Optional[List[float]] = None
        self.current_gripper: Optional[str] = None
        self.execution_state: Optional[str] = None
        self.epick_status: Optional[int] = None  # Raw status int from ObjectDetectionStatus

        # Zivid state
        self.last_rgb: Optional[np.ndarray] = None
        self.last_cloud: Optional[PointCloud2] = None
        self.last_image_msg: Optional[Image] = None
        self.capture_stamp = None  # Pre-capture timestamp for TF accuracy

        # ZED state (continuously updated by streaming callbacks)
        self.zed_rgb: Optional[np.ndarray] = None
        self.zed_cloud: Optional[PointCloud2] = None
        self.zed_image_msg: Optional[Image] = None
        self.zed_stamp = None

        # Synchronization events (for blocking Zivid capture)
        self._image_event = threading.Event()
        self._cloud_event = threading.Event()
        self._waiting_for_capture = False

        self.get_logger().info("ROS2BridgeNode initialized")

    def cancel_all_goals(self) -> str:
        """Cancel all active goals on the beambot execution action server.

        Sends an empty CancelGoal request which cancels ALL active goals.
        Returns a status message.
        """
        if not self._cancel_client.wait_for_service(timeout_sec=2.0):
            return "Cancel service not available (orchestrator not running?)"
        request = CancelGoal.Request()
        # Empty goal_info = cancel all goals
        future = self._cancel_client.call_async(request)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            time.sleep(0.05)
        if future.done():
            result = future.result()
            if result is not None:
                n = len(result.goals_canceling)
                if n > 0:
                    return f"Cancel accepted — {n} goal(s) being cancelled"
                return "No active goals to cancel"
            return "Cancel sent but no response received"
        return "Cancel request timed out"

    def _on_zivid_image(self, msg: Image):
        self.last_image_msg = msg
        try:
            self.last_rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"Zivid image conversion failed: {e}")
            return
        if self._waiting_for_capture:
            self._image_event.set()

    def _on_zivid_cloud(self, msg: PointCloud2):
        self.last_cloud = msg
        if self._waiting_for_capture:
            self._cloud_event.set()

    def _on_zed_image(self, msg: Image):
        self.zed_image_msg = msg
        self.zed_stamp = msg.header.stamp
        try:
            self.zed_rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"ZED image conversion failed: {e}")

    def _on_zed_cloud(self, msg: PointCloud2):
        self.zed_cloud = msg

    def _on_joint_states(self, msg: JointState):
        self.joint_names = list(msg.name)
        self.joint_positions = list(msg.position)

    def _on_current_gripper(self, msg: String):
        self.current_gripper = msg.data

    def _on_execution_state(self, msg: String):
        self.execution_state = msg.data

    def _on_epick_status(self, msg):
        self.epick_status = int(msg.status)

    def trigger_capture(self, timeout: float = 30.0, need_cloud: bool = True) -> bool:
        """Trigger Zivid capture and wait for data. Called from executor thread.

        Returns True if data received within timeout.
        """
        if not self._capture_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Capture service '{ZIVID_CAPTURE_SERVICE}' not available")
            return False

        # Clear events and mark that we're waiting
        self._image_event.clear()
        self._cloud_event.clear()
        self._waiting_for_capture = True

        # Record pre-capture timestamp for TF accuracy
        self.capture_stamp = self.get_clock().now().to_msg()

        # Send capture request (async — executor processes the response)
        request = Trigger.Request()
        future = self._capture_client.call_async(request)

        # Wait for service response
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not future.done():
            self.get_logger().error("Capture service call timed out")
            self._waiting_for_capture = False
            return False

        result = future.result()
        if not result.success:
            self.get_logger().error(f"Capture failed: {result.message}")
            self._waiting_for_capture = False
            return False

        self.get_logger().info("Capture triggered, waiting for data...")

        # Wait for image
        remaining = max(0.1, deadline - time.monotonic())
        if not self._image_event.wait(timeout=remaining):
            self.get_logger().error("Timeout waiting for image after capture")
            self._waiting_for_capture = False
            return False

        # Wait for point cloud (takes 3-4s longer due to ~40MB transfer)
        if need_cloud:
            remaining = max(0.1, deadline - time.monotonic())
            if not self._cloud_event.wait(timeout=remaining):
                self.get_logger().error("Timeout waiting for point cloud after capture")
                self._waiting_for_capture = False
                return False

        self._waiting_for_capture = False
        self.get_logger().info("Capture complete — image and cloud received")
        return True

    def detect_marker(self, marker_id: int, dictionary: str = "aruco4x4_50",
                      timeout: float = 30.0) -> Optional[Dict]:
        """Detect an ArUco marker using Zivid native detection and transform to base_link.

        Returns dict with 'position' [x,y,z], 'orientation' [x,y,z,w], or None on failure.
        """
        if self._marker_detect_client is None:
            self.get_logger().error("Zivid marker detection not available")
            return None

        # Wait for service with extended timeout (DDS discovery can be slow)
        service_ready = False
        for _ in range(10):
            if self._marker_detect_client.service_is_ready():
                service_ready = True
                break
            time.sleep(0.5)
        if not service_ready:
            self.get_logger().error(
                f"Zivid marker service '{ZIVID_MARKER_SERVICE}' not available after 5s"
            )
            return None

        # Record pre-capture timestamp for TF accuracy
        pre_capture_stamp = self.get_clock().now().to_msg()

        request = CaptureAndDetectMarkers.Request()
        request.marker_ids = [marker_id]
        request.marker_dictionary = dictionary

        self.get_logger().info(f"Calling marker detection for ID {marker_id}...")
        future = self._marker_detect_client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not future.done():
            self.get_logger().error("Marker detection timed out")
            return None

        result = future.result()
        if not result.success or not result.detection_result.detected_markers:
            self.get_logger().warn(f"Marker {marker_id} not detected: {result.message}")
            return None

        marker = result.detection_result.detected_markers[0]
        cam_pos = marker.pose.position
        cam_ori = marker.pose.orientation
        self.get_logger().info(
            f"Marker {marker_id} in camera: ({cam_pos.x*1000:.1f}, "
            f"{cam_pos.y*1000:.1f}, {cam_pos.z*1000:.1f}) mm"
        )

        # Transform to base_link using pre-capture timestamp
        tf = self.lookup_transform("base_link", ZIVID_FRAME, stamp=pre_capture_stamp)
        if tf is None:
            self.get_logger().error("Failed to get TF for marker transform")
            return None

        # Apply transform: rotate position, then translate
        from geometry_msgs.msg import PoseStamped
        from tf2_geometry_msgs import do_transform_pose_stamped

        pose_in = PoseStamped()
        pose_in.header.frame_id = ZIVID_FRAME
        pose_in.header.stamp = pre_capture_stamp
        pose_in.pose = marker.pose

        pose_out = do_transform_pose_stamped(pose_in, tf)

        pos = pose_out.pose.position
        ori = pose_out.pose.orientation
        self.get_logger().info(
            f"Marker {marker_id} in base_link: ({pos.x*1000:.1f}, "
            f"{pos.y*1000:.1f}, {pos.z*1000:.1f}) mm"
        )

        return {
            "position": [pos.x, pos.y, pos.z],
            "orientation": [ori.x, ori.y, ori.z, ori.w],
        }

    def lookup_transform(
        self, target_frame: str, source_frame: str, stamp=None, timeout_sec: float = 2.0,
    ):
        """Look up TF transform. Returns TransformStamped or None."""
        try:
            if stamp is not None:
                lookup_time = rclpy.time.Time.from_msg(stamp)
                self.get_logger().debug(
                    f"TF lookup {source_frame} → {target_frame} at "
                    f"{stamp.sec}.{stamp.nanosec:09d}"
                )
            else:
                lookup_time = rclpy.time.Time()
                self.get_logger().debug(
                    f"TF lookup {source_frame} → {target_frame} (latest)"
                )

            if not self._tf_buffer.can_transform(
                target_frame, source_frame, lookup_time,
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            ):
                self.get_logger().warning(
                    f"can_transform returned False: {source_frame} → {target_frame}"
                )
                return None

            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, lookup_time,
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
        except TransformException as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None


class ROS2Bridge:
    """Manages ROS2 node lifecycle in a background thread.

    Lazy-initialized on first tool call so the MCP server starts fast
    even if ROS2 isn't running yet.
    """

    def __init__(self):
        self._node: Optional[ROS2BridgeNode] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._initialized = False

    def _ensure_initialized(self):
        """Initialize ROS2 + node + executor on first use."""
        if self._initialized:
            return
        with self._lock:
            if self._initialized:
                return
            logger.info("Initializing ROS2 bridge...")
            if not rclpy.ok():
                rclpy.init()
            self._node = ROS2BridgeNode()
            self._executor = MultiThreadedExecutor(num_threads=4)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(
                target=self._spin, daemon=True, name="ros2-bridge",
            )
            self._thread.start()
            self._initialized = True
            logger.info("ROS2 bridge running in background thread")

    def _spin(self):
        """Run executor in background thread."""
        try:
            self._executor.spin()
        except Exception as e:
            logger.error(f"ROS2 executor error: {e}")

    @property
    def node(self) -> ROS2BridgeNode:
        self._ensure_initialized()
        return self._node

    def shutdown(self):
        if self._executor:
            self._executor.shutdown()
        if self._node:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ---------------------------------------------------------------------------
# Camera state resolution
# ---------------------------------------------------------------------------

def _resolve_camera_state(
    node: ROS2BridgeNode,
    camera: str,
) -> Tuple[Optional[np.ndarray], Optional[PointCloud2], Optional[Any], str]:
    """Resolve camera name to (rgb, cloud, stamp, frame) from node state."""
    if camera == "zivid":
        return node.last_rgb, node.last_cloud, node.capture_stamp, ZIVID_FRAME
    elif camera == "zed":
        return node.zed_rgb, node.zed_cloud, node.zed_stamp, ZED_FRAME
    else:
        raise ValueError(f"Unknown camera '{camera}'. Use 'zivid' or 'zed'.")


# ---------------------------------------------------------------------------
# MCP Server
# ---------------------------------------------------------------------------

mcp = FastMCP("beambot")

# Global bridge instance (lazy-initialized)
bridge = ROS2Bridge()


@mcp.tool()
async def ping() -> str:
    """Test connectivity to the EROBS MCP server.

    Returns 'pong' if the server is running. Use this to verify the server
    is reachable before calling other tools.
    """
    return "pong"


@mcp.tool()
async def stop_robot() -> str:
    """Emergency stop: cancel all active goals on the beambot orchestrator.

    Sends a cancel-all request to /beambot_execution. The orchestrator will
    finish the current motion step and then stop (does not interrupt mid-motion).
    Use this when the robot needs to be stopped and you don't have the goal ID.
    """
    node = bridge.node
    result = await asyncio.get_event_loop().run_in_executor(
        None, node.cancel_all_goals
    )
    return json.dumps({"result": result})


@mcp.tool()
async def get_robot_state() -> str:
    """Get the current state of the robot system.

    Returns a JSON object with:
    - system_running: whether the robot system (MoveIt, action servers) is up
    - gripper: currently attached gripper name, or "unknown" if system not running
    - execution_state: IDLE, EXECUTING, or PAUSED (null if system not running)
    - joints_deg: current joint positions in degrees (matches task JSON convention),
      or null if system not running. Keys are joint names.

    Call this BEFORE constructing task JSON to know:
    1. Whether the system is running (if not, your first goal will launch it)
    2. Which gripper is attached (so you can set start_gripper correctly)
    3. Current robot pose (to judge if a move is feasible)
    """
    node = bridge.node

    system_running = node.joint_positions is not None
    gripper = node.current_gripper or "unknown"
    exec_state = node.execution_state

    joints_deg = None
    if node.joint_positions is not None and node.joint_names is not None:
        joints_deg = {
            name: round(math.degrees(pos), 2)
            for name, pos in zip(node.joint_names, node.joint_positions)
        }

    # Include ePick vacuum status when ePick is the active gripper
    vacuum_status = None
    if gripper == "epick" and node.epick_status is not None:
        status_int = node.epick_status
        vacuum_status = {
            "status": EPICK_STATUS_NAMES.get(status_int, f"UNKNOWN({status_int})"),
            "object_detected": status_int in (1, 2),
        }

    result = {
        "system_running": system_running,
        "gripper": gripper,
        "execution_state": exec_state,
        "joints_deg": joints_deg,
    }
    if vacuum_status is not None:
        result["vacuum_status"] = vacuum_status
    return json.dumps(result, indent=2)


@mcp.tool()
async def get_vacuum_status() -> str:
    """Get the ePick vacuum gripper's object detection status.

    Returns a JSON object with:
    - status: one of "UNKNOWN", "OBJECT_DETECTED_AT_MIN_PRESSURE",
      "OBJECT_DETECTED_AT_MAX_PRESSURE", "NO_OBJECT_DETECTED"
    - object_detected: boolean — true if vacuum seal confirms an object is held
    - available: whether the ePick status topic is being published

    Use this AFTER a vacuum pick operation to verify the object was grasped.
    If object_detected is false after closing the vacuum, the pick failed —
    do NOT proceed to transport.

    Status meanings:
    - OBJECT_DETECTED_AT_MIN_PRESSURE: Object held with minimum vacuum (light seal)
    - OBJECT_DETECTED_AT_MAX_PRESSURE: Object held with maximum vacuum (strong seal)
    - NO_OBJECT_DETECTED: No vacuum seal — nothing picked up, or object dropped
    - UNKNOWN: Regulating toward target vacuum, status not yet determined
    """
    node = bridge.node
    gripper = node.current_gripper or "unknown"

    if node.epick_status is None:
        return json.dumps({
            "available": False,
            "gripper": gripper,
            "note": "No data received on /object_detection_status yet. "
                    "The ePick status controller may not be running "
                    "(only active when ePick gripper is loaded).",
        })

    status_int = node.epick_status
    return json.dumps({
        "available": True,
        "gripper": gripper,
        "status": EPICK_STATUS_NAMES.get(status_int, f"UNKNOWN({status_int})"),
        "object_detected": status_int in (1, 2),
    }, indent=2)


def _read_poses_file() -> dict:
    """Read the poses YAML file. Returns empty dict if file doesn't exist."""
    path = os.path.realpath(POSES_FILE)
    if not os.path.exists(path):
        return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return data


def _write_poses_file(poses: dict):
    """Write poses dict to the YAML file atomically.

    Writes to a temp file first, then renames — prevents corruption
    if the process is killed mid-write.
    """
    import tempfile

    path = os.path.realpath(POSES_FILE)
    dir_path = os.path.dirname(path)
    os.makedirs(dir_path, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(dir=dir_path, suffix=".yaml.tmp")
    try:
        with os.fdopen(fd, "w") as f:
            yaml.dump(poses, f, default_flow_style=None, width=200)
        os.replace(tmp_path, path)
    except BaseException:
        os.unlink(tmp_path)
        raise


@mcp.tool()
async def get_saved_poses(filter: str = "") -> str:
    """Get saved robot poses from the pose registry.

    Returns a JSON object mapping pose names to joint angle arrays (degrees).
    These values can be used directly in task JSON "poses" dicts.

    Args:
        filter: Optional substring to filter pose names (case-insensitive).
            Example: filter="hotplate" returns all poses with "hotplate" in the name.
    """
    poses = _read_poses_file()

    if not poses:
        return json.dumps({
            "poses": {},
            "count": 0,
            "message": f"No poses file found at {os.path.realpath(POSES_FILE)}. "
                       "Use save_pose to create one.",
        })

    if filter:
        filter_lower = filter.lower()
        poses = {k: v for k, v in poses.items() if filter_lower in k.lower()}

    return json.dumps({"poses": poses, "count": len(poses)}, indent=2)


@mcp.tool()
async def save_pose(
    name: str,
    joints_deg: list = None,
    description: str = "",
) -> str:
    """Save a robot pose to the pose registry.

    If joints_deg is omitted, saves the robot's current joint positions
    (the robot system must be running).

    Args:
        name: Pose name (e.g., "hotplate", "sample_scan_1").
        joints_deg: 6-element list of joint angles in degrees.
            If omitted, reads current position from the robot.
        description: Optional note — saved as a YAML comment above the entry.
    """
    if joints_deg is not None:
        if len(joints_deg) != 6:
            return json.dumps({"error": f"Expected 6 joint values, got {len(joints_deg)}"})
        values = [round(float(v), 2) for v in joints_deg]
    else:
        node = bridge.node
        if node.joint_positions is None:
            return json.dumps({
                "error": "No joint data available. Robot system not running. "
                         "Provide joints_deg explicitly.",
            })
        values = [round(math.degrees(v), 2) for v in node.joint_positions]

    poses = _read_poses_file()
    overwritten = name in poses
    poses[name] = values
    _write_poses_file(poses)

    return json.dumps({
        "saved": name,
        "joints_deg": values,
        "overwritten": overwritten,
        "total_poses": len(poses),
    })


@mcp.tool()
async def delete_pose(name: str) -> str:
    """Delete a pose from the pose registry.

    Args:
        name: Pose name to delete.
    """
    poses = _read_poses_file()

    if name not in poses:
        return json.dumps({
            "error": f"Pose '{name}' not found.",
            "available": list(poses.keys()),
        })

    del poses[name]
    _write_poses_file(poses)

    return json.dumps({
        "deleted": name,
        "remaining_poses": len(poses),
    })


@mcp.tool()
async def set_cup_profile(name: str) -> str:
    """Set the active ePick suction cup profile.

    Changes which suction cup dimensions are used when MoveIt launches
    for the ePick gripper. Takes effect on the next MoveIt launch (next goal
    with start_gripper="epick", or after a tool exchange to ePick).

    Available profiles are defined in epick_config/config/suction_cups.yaml.

    Args:
        name: Cup profile name (e.g., "pen_vacuum", "7mm_dia", "default").
    """
    import subprocess as _sp

    try:
        result = _sp.run(
            ["ros2", "param", "set", "/beambot_orchestrator", "cup_profile", name],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0:
            return json.dumps({
                "cup_profile": name,
                "message": f"Cup profile set to '{name}'. Takes effect on next ePick MoveIt launch.",
            })
        else:
            return json.dumps({
                "error": f"Failed to set parameter: {result.stderr.strip()}",
                "note": "Is the orchestrator running?",
            })
    except Exception as e:
        return json.dumps({"error": f"Failed to set cup profile: {e}"})


@mcp.tool()
async def capture_image(
    camera: str = "zivid",
    mode: str = "3d",
    save_path: str = DEFAULT_IMAGE_PATH,
    timeout: float = 30.0,
) -> str:
    """Capture an image (and optionally point cloud) from a camera.

    Supports two cameras:
        - "zivid": Eye-in-hand 3D camera. Single-shot triggered capture.
          High accuracy, narrow FOV. Use for precise positioning.
        - "zed": Fixed external ZED 2i stereo camera. Continuous streaming.
          Wide FOV, covers full workspace. Use for scene overview, finding
          objects, and guiding the robot to the right area.

    The image is saved to disk so Claude can view it with the Read tool.

    Args:
        camera: Which camera to use — "zivid" or "zed". Default "zivid".
        mode: "2d" for image only, "3d" for image + point cloud (needed for
              detect_objects 3D positions). Default "3d".
        save_path: Where to save the captured image. Default /tmp/beambot_capture.jpg.
        timeout: Max seconds to wait for capture. Default 30.

    Returns:
        JSON with image_path, width, height, has_pointcloud, camera_frame.
    """
    node = bridge.node

    if camera == "zed":
        # ZED streams continuously — just grab whatever's latest
        rgb = node.zed_rgb
        cloud = node.zed_cloud
        stamp = node.zed_stamp
        frame = ZED_FRAME

        if rgb is None:
            # Wait briefly for first frame if ZED just started
            loop = asyncio.get_event_loop()
            def _wait_for_zed():
                deadline = time.monotonic() + min(timeout, 5.0)
                while node.zed_rgb is None and time.monotonic() < deadline:
                    time.sleep(0.1)
                return node.zed_rgb is not None
            got_frame = await loop.run_in_executor(None, _wait_for_zed)
            if not got_frame:
                return json.dumps({
                    "error": "No ZED image available. Is the ZED camera running? "
                             "Check: ros2 topic hz /zed/zed_node/rgb/color/rect/image",
                })
            rgb = node.zed_rgb
            cloud = node.zed_cloud
            stamp = node.zed_stamp

    elif camera == "zivid":
        # Zivid requires explicit trigger
        need_cloud = mode == "3d"
        loop = asyncio.get_event_loop()
        success = await loop.run_in_executor(
            None, lambda: node.trigger_capture(timeout=timeout, need_cloud=need_cloud),
        )
        if not success:
            return json.dumps({
                "error": "Zivid capture failed. Is the camera connected and driver running? "
                         "Check: ros2 service list | grep capture",
            })
        rgb = node.last_rgb
        cloud = node.last_cloud
        stamp = node.capture_stamp
        frame = ZIVID_FRAME
    else:
        return json.dumps({"error": f"Unknown camera '{camera}'. Use 'zivid' or 'zed'."})

    if rgb is None:
        return json.dumps({"error": f"No image data received from {camera}"})

    # Save image (convert RGB → BGR for OpenCV)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    os.makedirs(os.path.dirname(save_path) or "/tmp", exist_ok=True)
    cv2.imwrite(save_path, bgr)

    h, w = rgb.shape[:2]
    result = {
        "image_path": save_path,
        "camera": camera,
        "camera_frame": frame,
        "width": w,
        "height": h,
        "has_pointcloud": cloud is not None,
        "capture_timestamp": f"{stamp.sec}.{stamp.nanosec:09d}" if stamp else None,
    }
    return json.dumps(result)


@mcp.tool()
async def detect_objects(
    method: str = "hsv_color",
    camera: str = "zivid",
    hue_low: int = 100,
    hue_high: int = 130,
    sat_min: int = 80,
    val_min: int = 80,
    min_area: int = 200,
    min_radius: int = 15,
    max_radius: int = 100,
    circle_param2: int = 25,
    contour_min_area: int = 500,
    contour_max_area: int = 50000,
    marker_ids: str = "",
    yolo_model: str = "yolov8n.pt",
    yolo_confidence: float = 0.25,
    yolo_classes: str = "",
    transform_to_base: bool = True,
    save_path: str = DEFAULT_ANNOTATED_PATH,
) -> str:
    """Detect objects in the last captured image.

    IMPORTANT: Call capture_image() first! This operates on the most
    recent capture data from the specified camera.

    Detection methods:
        - "hsv_color": Find objects by color in HSV space. Good for colored balls,
          samples with distinctive hues. Tune hue_low/hue_high for your target color.
          Common ranges: blue=100-130, red=0-10 or 170-180, green=35-85, yellow=20-35.
        - "circle": Hough circle detection. Good for round samples/wafers.
        - "contour": Edge-based contour detection. Finds any shape.
        - "marker": ArUco marker detection (2D image-based, not Zivid native).
        - "yolo": Deep learning object detection (Ultralytics YOLOv8/v11). Most robust
          method — detects objects by class with bounding boxes. Works well regardless
          of lighting/contrast. Use yolo_model for custom weights, yolo_classes to filter.

    Args:
        method: Detection method — "hsv_color", "circle", "contour", "marker", or "yolo".
        camera: Which camera's data to use — "zivid" or "zed". Default "zivid".
            Must match the camera used in the preceding capture_image() call.
        hue_low: HSV hue lower bound (0-180). Only for hsv_color.
        hue_high: HSV hue upper bound (0-180). Only for hsv_color.
        sat_min: Min saturation (0-255). Only for hsv_color.
        val_min: Min value/brightness (0-255). Only for hsv_color.
        min_area: Min object area in pixels for hsv_color.
        min_radius: Min circle radius in pixels. Only for circle.
        max_radius: Max circle radius in pixels. Only for circle.
        circle_param2: Hough accumulator threshold (lower=more sensitive). Only for circle.
        contour_min_area: Min contour area in pixels². Only for contour.
        contour_max_area: Max contour area in pixels². Only for contour.
        marker_ids: Comma-separated ArUco marker IDs to find (empty=all). Only for marker.
        yolo_model: YOLO model weights — "yolov8n.pt" (nano/fast), "yolov8s.pt" (small),
            "yolov8m.pt" (medium), or path to custom fine-tuned weights. Only for yolo.
        yolo_confidence: Min detection confidence 0-1. Lower = more detections. Only for yolo.
        yolo_classes: Comma-separated class names or IDs to filter (e.g. "book,cell phone"
            or "73,67"). Empty = detect all classes. Only for yolo.
        transform_to_base: If True, transform 3D positions from camera frame to base_link.
        save_path: Where to save annotated image. Default /tmp/beambot_detection.jpg.

    Returns:
        JSON with list of detections, each containing pixel coords, 3D position
        (camera and/or base frame), and an annotated image path.
    """
    node = bridge.node

    try:
        rgb, cloud, stamp, camera_frame = _resolve_camera_state(node, camera)
    except ValueError as e:
        return json.dumps({"error": str(e)})

    if rgb is None:
        return json.dumps({
            "error": f"No image data from {camera}. Call capture_image(camera='{camera}') first.",
        })

    # Run detection
    raw_detections = None
    if method == "hsv_color":
        raw = _detect_hsv_color(
            rgb, hue_low=hue_low, hue_high=hue_high,
            sat_min=sat_min, val_min=val_min, min_area=min_area,
        )
        if raw:
            raw_detections = [{"pixel_x": cx, "pixel_y": cy, "area": a} for cx, cy, a in raw]

    elif method == "circle":
        params = CircleDetectionParams(
            min_radius=min_radius, max_radius=max_radius, param2=circle_param2,
        )
        raw = detect_hough_circles(rgb, params)
        if raw:
            raw_detections = [
                {"pixel_x": cx, "pixel_y": cy, "radius": r} for cx, cy, r in raw
            ]

    elif method == "contour":
        params = ContourDetectionParams(
            min_area=contour_min_area, max_area=contour_max_area,
        )
        raw = detect_contours_in_image(rgb, params)
        if raw:
            raw_detections = [
                {"pixel_x": cx, "pixel_y": cy, "area": a, "vertices": v}
                for cx, cy, a, v in raw
            ]

    elif method == "marker":
        ids_list = None
        if marker_ids.strip():
            ids_list = [int(x.strip()) for x in marker_ids.split(",")]
        raw = _detect_aruco_markers(rgb, marker_ids=ids_list)
        if raw:
            raw_detections = [
                {"pixel_x": cx, "pixel_y": cy, "marker_id": mid, "corners": corners}
                for cx, cy, mid, corners in raw
            ]

    elif method == "yolo":
        # Parse class filter (names or IDs)
        class_ids = None
        class_filter_names = []
        if yolo_classes.strip():
            parts = [c.strip() for c in yolo_classes.split(",")]
            # Check if they're numeric (class IDs) or names
            if all(p.isdigit() for p in parts):
                class_ids = [int(p) for p in parts]
            else:
                class_filter_names = [p.lower() for p in parts]

        params = YoloDetectionParams(
            model_path=yolo_model,
            confidence=yolo_confidence,
            classes=class_ids,
        )
        detector = get_yolo_detector(yolo_model)
        raw = detector.detect(rgb, params)

        # Filter by class name if specified as strings
        if class_filter_names:
            raw = [d for d in raw if d[0].lower() in class_filter_names]

        if raw:
            # Annotate with YOLO-specific visualization
            annotated_yolo = detector.annotate(rgb, raw)
            os.makedirs(os.path.dirname(save_path) or "/tmp", exist_ok=True)
            cv2.imwrite(save_path, annotated_yolo)

            raw_detections = [
                {
                    "pixel_x": cx, "pixel_y": cy,
                    "class": cls_name, "confidence": round(conf, 3),
                    "bbox": [x1, y1, x2, y2],
                }
                for cls_name, conf, cx, cy, x1, y1, x2, y2 in raw
            ]

    else:
        return json.dumps({"error": f"Unknown method '{method}'. Use: hsv_color, circle, contour, marker, yolo"})

    if not raw_detections:
        return json.dumps({
            "detections": [],
            "count": 0,
            "method": method,
            "message": f"No objects detected with method '{method}'",
        })

    # Add 3D positions from point cloud
    for det in raw_detections:
        det["camera_xyz"] = None
        det["base_xyz"] = None

        if cloud is not None:
            xyz = get_3d_position(cloud, det["pixel_x"], det["pixel_y"])
            if xyz is not None:
                det["camera_xyz"] = list(xyz)

                # Transform to base_link if requested
                if transform_to_base:
                    base_xyz = await _transform_point_to_base(
                        node, xyz, camera_frame, stamp,
                    )
                    if base_xyz is not None:
                        det["base_xyz"] = list(base_xyz)

    # Annotate and save image (YOLO handles its own annotation above)
    if method != "yolo":
        annotated = _annotate_image(rgb, raw_detections, method)
        os.makedirs(os.path.dirname(save_path) or "/tmp", exist_ok=True)
        cv2.imwrite(save_path, annotated)

    # Clean up non-serializable fields
    for det in raw_detections:
        if "corners" in det:
            # corners are already lists from _detect_aruco_markers
            pass

    result = {
        "detections": raw_detections,
        "count": len(raw_detections),
        "method": method,
        "annotated_image_path": save_path,
        "camera": camera,
        "coordinate_frame": "base_link" if transform_to_base else camera_frame,
    }
    return json.dumps(result)


async def _transform_point_to_base(
    node: ROS2BridgeNode,
    camera_xyz: Tuple[float, float, float],
    camera_frame: str,
    capture_stamp=None,
) -> Optional[Tuple[float, float, float]]:
    """Transform a 3D point from camera frame to base_link using TF.

    Tries capture_stamp first for accuracy (matches robot pose at capture time),
    then falls back to latest transform if the timestamped lookup fails (e.g.,
    TF buffer hasn't accumulated enough data yet).
    """
    loop = asyncio.get_event_loop()

    def _do_transform():
        # Try timestamped lookup first
        transform = None
        if capture_stamp is not None:
            transform = node.lookup_transform(
                "base_link", camera_frame, stamp=capture_stamp,
            )
            if transform is None:
                logger.warning(
                    "TF lookup at capture_stamp failed, falling back to latest transform"
                )

        # Fallback: latest available transform
        if transform is None:
            transform = node.lookup_transform(
                "base_link", camera_frame, stamp=None,
            )
            if transform is None:
                logger.error(
                    f"TF lookup failed: {camera_frame} → base_link not available "
                    "(even latest). Is the robot driver running?"
                )
                return None

        from tf_transformations import quaternion_matrix

        q = transform.transform.rotation
        t = transform.transform.translation
        mat = quaternion_matrix([q.x, q.y, q.z, q.w])
        mat[0, 3] = t.x
        mat[1, 3] = t.y
        mat[2, 3] = t.z

        point_cam = np.array([camera_xyz[0], camera_xyz[1], camera_xyz[2], 1.0])
        point_base = mat @ point_cam
        return (float(point_base[0]), float(point_base[1]), float(point_base[2]))

    return await loop.run_in_executor(None, _do_transform)


async def _confirm_point_via_gui(
    image_path: str,
    pixel_x: int,
    pixel_y: int,
    timeout: float = 120.0,
) -> Optional[Dict[str, Any]]:
    """Launch the point selector GUI as a subprocess and wait for the result.

    Returns:
        Dict with confirmed point, None if cancelled, or dict with error key.
    """
    gui_script = os.path.join(os.path.dirname(__file__), "point_selector_gui.py")
    if not os.path.exists(gui_script):
        return {"error": f"GUI script not found: {gui_script}"}

    cmd = [
        sys.executable, gui_script, image_path,
        "--x", str(pixel_x), "--y", str(pixel_y),
        "--title", "Confirm Point — Click to adjust, Enter to confirm",
    ]

    # Ensure X11 env vars are available for the GUI subprocess.
    # Claude Code strips DISPLAY when spawning MCP servers, so we detect
    # the active display at launch and inject it here.
    env = os.environ.copy()
    env.setdefault("DISPLAY", _DETECTED_DISPLAY)
    env.setdefault("XAUTHORITY", _DETECTED_XAUTHORITY)

    try:
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            env=env,
        )
        stdout, stderr = await asyncio.wait_for(
            proc.communicate(), timeout=timeout,
        )
    except asyncio.TimeoutError:
        proc.kill()
        return {"error": f"GUI timed out after {timeout}s"}
    except Exception as e:
        return {"error": f"Failed to launch GUI: {e}"}

    stderr_text = stderr.decode().strip() if stderr else ""
    if stderr_text:
        logger.info(f"GUI stderr: {stderr_text}")

    if proc.returncode != 0:
        # Still try to parse stdout — the script outputs error JSON before exit(1)
        try:
            result = json.loads(stdout.decode().strip())
            if stderr_text:
                result["stderr"] = stderr_text
            return result
        except (json.JSONDecodeError, ValueError):
            return {"error": f"GUI exited with code {proc.returncode}: {stderr_text}"}

    try:
        result = json.loads(stdout.decode().strip())
    except (json.JSONDecodeError, ValueError) as e:
        return {"error": f"Failed to parse GUI output: {e}. stderr: {stderr_text}"}

    if not result.get("confirmed", False):
        return None  # User cancelled

    return result


def _detect_sample_in_roi(
    rgb_image: np.ndarray,
    marker_corners: np.ndarray,
    px_per_mm: float,
    roi_offset_x_mm: float = 19.3,
    roi_offset_y_mm: float = 0.3,
    roi_width_mm: float = 22.1,
    roi_height_mm: float = 21.8,
    edge_inset_mm: float = 4.0,
    strategy: str = "farthest_edge",
    min_area: int = 100,
    max_area: int = 15000,
    max_aspect_ratio: float = 3.0,
) -> Optional[Dict[str, Any]]:
    """Detect a sample contour in a fixed ROI relative to an ArUco marker.

    Pure OpenCV — no ROS dependencies. Reusable from scripts and MCP tools.

    Args:
        rgb_image: Full image (BGR or RGB)
        marker_corners: Shape (4, 2) — tag corner pixels [TL, TR, BR, BL]
        px_per_mm: Pixel-to-mm scale
        roi_offset_x_mm: ROI center offset in marker +X direction (mm)
        roi_offset_y_mm: ROI center offset in marker +Y direction (mm)
        roi_width_mm: ROI width in mm
        roi_height_mm: ROI height in mm
        edge_inset_mm: Distance to move inward from edge toward center (mm)
        strategy: Pickup strategy — "center", "farthest_edge", "nearest_edge",
                  "farthest_corner", "nearest_corner"
        min_area: Min contour area (px²)
        max_area: Max contour area (px²)
        max_aspect_ratio: Max width/height ratio for valid sample

    Returns:
        Dict with pickup_px, center_px, sample_size_mm, angle, offset_from_center_mm,
        or None if no sample found.
    """
    # Marker axes in pixel space
    top_left, top_right = marker_corners[0], marker_corners[1]
    bottom_left = marker_corners[3]
    marker_x = top_right - top_left
    marker_x = marker_x / np.linalg.norm(marker_x)
    marker_y = bottom_left - top_left
    marker_y = marker_y / np.linalg.norm(marker_y)
    tag_center = marker_corners.mean(axis=0)

    # ROI center in pixel space
    roi_center = (tag_center
                  + marker_x * (roi_offset_x_mm * px_per_mm)
                  + marker_y * (roi_offset_y_mm * px_per_mm))

    half_w = (roi_width_mm * px_per_mm) / 2
    half_h = (roi_height_mm * px_per_mm) / 2

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
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter by area and aspect ratio
    valid = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue
        rect = cv2.minAreaRect(c)
        rw, rh = rect[1]
        if rw == 0 or rh == 0:
            continue
        if max(rw, rh) / min(rw, rh) > max_aspect_ratio:
            continue
        valid.append((area, c))

    if not valid:
        return None

    # Select largest contour
    valid.sort(key=lambda x: x[0], reverse=True)
    sample_contour = valid[0][1]

    # Fit rectangle
    rect = cv2.minAreaRect(sample_contour)
    rect_center_roi = rect[0]
    rect_size = rect[1]
    rect_angle = rect[2]
    rect_corners_roi = cv2.boxPoints(rect)

    # Convert to full image coordinates
    rect_center_full = np.array([rect_center_roi[0] + roi_x1,
                                  rect_center_roi[1] + roi_y1])
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
            midpoints.append((rect_corners_full[i] + rect_corners_full[(i + 1) % 4]) / 2)
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

    # Compute offset from center to pickup point in mm
    offset_from_center_mm = np.linalg.norm(pickup - center_pt) / px_per_mm

    return {
        "pickup_px": (int(pickup[0]), int(pickup[1])),
        "center_px": (int(center_pt[0]), int(center_pt[1])),
        "sample_size_mm": (round(rect_size[0] / px_per_mm, 1),
                           round(rect_size[1] / px_per_mm, 1)),
        "sample_angle": round(rect_angle, 1),
        "sample_area_px": int(cv2.contourArea(sample_contour)),
        "offset_from_center_mm": round(offset_from_center_mm, 1),
        "roi": (roi_x1, roi_y1, roi_x2, roi_y2),
        "strategy": strategy,
        "edge_inset_mm": edge_inset_mm,
    }


@mcp.tool()
async def detect_sample(
    tag_id: int,
    camera: str = "zivid",
    strategy: str = "farthest_edge",
    edge_inset_mm: float = 4.0,
    save_path: str = "/tmp/sample_detection.jpg",
) -> str:
    """Detect a sample near an ArUco tag and return its 3D pickup point.

    IMPORTANT: Call capture_image(mode='3d') first! This uses the most recent
    capture data for both image analysis and 3D position lookup.

    Pipeline:
    1. Detect ArUco tag in the captured image (pixel corners)
    2. Crop a fixed ROI at 26mm right of the tag
    3. Run contour detection in the ROI to find the sample shape
    4. Fit a rectangle to the sample, compute pickup point on the edge
    5. Look up 3D position from point cloud at the pickup pixel
    6. Transform to base_link

    The pickup point is offset from the sample center based on strategy,
    so the X-ray beam can hit the center while the suction cup grips the edge.

    Args:
        tag_id: ArUco marker ID near the sample
        camera: Camera to use ("zivid" or "zed"). Default "zivid".
        strategy: Where to grip — "center", "farthest_edge" (default),
            "nearest_edge", "farthest_corner", "nearest_corner".
            "farthest_edge" = midpoint of edge farthest from tag, inset by edge_inset_mm.
        edge_inset_mm: How far inward from edge toward center (mm). Default 4.0.
            Only used for edge/corner strategies.
        save_path: Where to save annotated detection image.

    Returns:
        JSON with:
        - pickup_base_xyz: [x, y, z] pickup point in base_link (use as cartesian_target)
        - center_base_xyz: [x, y, z] sample center in base_link
        - offset_from_center_mm: distance from center to pickup (add to placement offset)
        - sample_size_mm: [width, height] of detected sample
        - sample_angle: rotation angle
    """
    node = bridge.node

    # Get camera state (image + point cloud from last capture)
    try:
        rgb, cloud, stamp, camera_frame = _resolve_camera_state(node, camera)
    except ValueError as e:
        return json.dumps({"error": str(e)})

    if rgb is None or cloud is None:
        return json.dumps({
            "error": f"No image/point cloud from {camera}. "
                     f"Call capture_image(camera='{camera}', mode='3d') first."
        })

    # Step 1: Detect ArUco markers in the image
    # Convert RGB to BGR for OpenCV ArUco detection
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    markers = _detect_aruco_markers(rgb, marker_ids=[tag_id])

    if markers is None:
        return json.dumps({
            "error": f"Tag {tag_id} not detected in image. "
                     "Ensure the tag is visible from the current position."
        })

    # Extract corners for target tag
    tag_cx, tag_cy, tag_mid, tag_corners_list = markers[0]
    tag_corners = np.array(tag_corners_list)  # Shape (4, 2)

    # Compute pixel scale from marker size (20mm printed)
    side_lengths = [np.linalg.norm(tag_corners[(i+1)%4] - tag_corners[i]) for i in range(4)]
    MARKER_SIZE_MM = 14.9  # Measured physical marker size (black square)
    px_per_mm = np.mean(side_lengths) / MARKER_SIZE_MM

    # Step 2-4: Detect sample in ROI
    detection = _detect_sample_in_roi(
        bgr, tag_corners, px_per_mm,
        strategy=strategy,
        edge_inset_mm=edge_inset_mm,
    )

    if detection is None:
        return json.dumps({
            "error": f"No sample contour found near tag {tag_id}. "
                     "Check that a sample is placed in the expected ROI area."
        })

    pickup_px = detection["pickup_px"]
    center_px = detection["center_px"]

    # Step 5: Get 3D positions from point cloud
    # The dark sample surface has sparse/missing depth at individual pixels.
    # Solution: average ALL valid pixels within a radius around the target pixel.
    # This centers the estimate on the target rather than biasing toward
    # whichever direction has the first valid depth.
    # Use the tag center's Z (depth) since tag is coplanar and always reliable.

    # Get tag depth (always reliable — white paper)
    tag_xyz = get_3d_position(cloud, tag_cx, tag_cy, search_radius=10)
    if tag_xyz is None:
        return json.dumps({
            "error": f"No valid depth at tag center ({tag_cx}, {tag_cy})."
        })
    tag_z = tag_xyz[2]

    # Look up 3D at pickup pixel (same approach as get_point_3d)
    pickup_xyz = get_3d_position(cloud, pickup_px[0], pickup_px[1], search_radius=20)

    # Look up 3D at center pixel
    center_xyz = get_3d_position(cloud, center_px[0], center_px[1], search_radius=20)
    if center_xyz is None:
        center_xyz = tag_xyz  # fallback to tag position

    # Step 6: Transform to base_link (may be None if depth was missing)
    pickup_base = None
    if pickup_xyz is not None:
        pickup_base = await _transform_point_to_base(node, pickup_xyz, camera_frame, stamp)
    center_base = None
    if center_xyz is not None:
        center_base = await _transform_point_to_base(node, center_xyz, camera_frame, stamp)

    # pickup_base may be None if depth was missing — that's OK,
    # we still return marker_offset_x/y which are pixel-based

    # Annotate image
    annotated = bgr.copy()
    # ROI box (yellow)
    rx1, ry1, rx2, ry2 = detection["roi"]
    cv2.rectangle(annotated, (rx1, ry1), (rx2, ry2), (0, 255, 255), 2)
    # Tag (blue)
    cv2.polylines(annotated, [tag_corners.astype(int)], True, (255, 0, 0), 2)
    # Sample center (cyan)
    cv2.circle(annotated, center_px, 5, (255, 255, 0), -1)
    # Pickup point (red)
    cv2.circle(annotated, pickup_px, 8, (0, 0, 255), -1)
    cv2.circle(annotated, pickup_px, 12, (0, 0, 255), 2)
    cv2.putText(annotated, f"tag{tag_id} {strategy}",
                (pickup_px[0] + 15, pickup_px[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.imwrite(save_path, annotated)

    # Compute marker-frame offset for use with vision_moveto
    # Project pixel offset from tag to pickup onto marker axes
    top_left, top_right = tag_corners[0], tag_corners[1]
    bottom_left = tag_corners[3]
    marker_x_dir = top_right - top_left
    marker_x_dir = marker_x_dir / np.linalg.norm(marker_x_dir)
    marker_y_dir = bottom_left - top_left
    marker_y_dir = marker_y_dir / np.linalg.norm(marker_y_dir)
    tag_center_px = np.array([tag_cx, tag_cy], dtype=float)
    offset_px = np.array(pickup_px, dtype=float) - tag_center_px
    marker_offset_x_m = np.dot(offset_px, marker_x_dir) / px_per_mm / 1000.0
    marker_offset_y_m = np.dot(offset_px, marker_y_dir) / px_per_mm / 1000.0

    result = {
        "pickup_base_xyz": [round(v, 6) for v in pickup_base] if pickup_base else None,
        "center_base_xyz": [round(v, 6) for v in center_base] if center_base else None,
        "marker_offset_x": round(marker_offset_x_m, 5),
        "marker_offset_y": round(marker_offset_y_m, 5),
        "offset_from_center_mm": detection["offset_from_center_mm"],
        "sample_size_mm": detection["sample_size_mm"],
        "sample_angle": detection["sample_angle"],
        "pickup_pixel": pickup_px,
        "center_pixel": center_px,
        "strategy": strategy,
        "edge_inset_mm": edge_inset_mm,
        "tag_id": tag_id,
        "annotated_image_path": save_path,
    }

    return json.dumps(result)


@mcp.tool()
async def get_point_3d(
    pixel_x: int,
    pixel_y: int,
    camera: str = "zivid",
    transform_to_base: bool = True,
    search_radius: int = 10,
    confirm: bool = True,
    save_path: str = "/tmp/beambot_point3d.jpg",
) -> str:
    """Get the 3D position of a pixel from the last captured point cloud.

    IMPORTANT: Call capture_image() first! This uses the most recent
    point cloud data from the specified camera.

    Use this when you can see something in the camera image and want to know
    its real-world 3D position — e.g., "what are the 3D coordinates of the
    object at pixel (500, 300)?" This is useful for planning robot moves to
    arbitrary points visible in the image without needing a specific detector.

    Args:
        pixel_x: X coordinate (column) in the image.
        pixel_y: Y coordinate (row) in the image.
        camera: Which camera's data to use — "zivid" or "zed". Default "zivid".
            Must match the camera used in the preceding capture_image() call.
        transform_to_base: If True, return position in base_link frame.
            If False, return in the camera's optical frame.
        search_radius: If the exact pixel has no depth, search nearby pixels
            within this radius. Default 10.
        confirm: If True, open a GUI window showing the image with the
            suggested point. The user can click to adjust the position,
            then press Enter to confirm or Esc to cancel. Default True.
        save_path: Where to save annotated image showing the queried point.

    Returns:
        JSON with camera_xyz, base_xyz (if transform_to_base), and an
        annotated image showing the queried point.
    """
    node = bridge.node

    try:
        rgb, cloud, stamp, camera_frame = _resolve_camera_state(node, camera)
    except ValueError as e:
        return json.dumps({"error": str(e)})

    if cloud is None:
        return json.dumps({
            "error": f"No point cloud data from {camera}. Call capture_image(camera='{camera}', mode='3d') first.",
        })

    if rgb is None:
        return json.dumps({
            "error": f"No image data from {camera}. Call capture_image(camera='{camera}', mode='3d') first.",
        })

    # Bounds check
    h, w = rgb.shape[:2]
    if pixel_x < 0 or pixel_x >= w or pixel_y < 0 or pixel_y >= h:
        return json.dumps({
            "error": f"Pixel ({pixel_x}, {pixel_y}) out of bounds. "
                     f"Image size: {w}x{h}.",
        })

    # GUI confirmation step
    if confirm:
        # Use the saved capture image, or save current frame as fallback
        image_path = DEFAULT_IMAGE_PATH
        if not os.path.exists(image_path):
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.imwrite(image_path, bgr)

        gui_result = await _confirm_point_via_gui(image_path, pixel_x, pixel_y)

        if gui_result is None:
            return json.dumps({
                "cancelled": True,
                "message": "User cancelled point selection.",
            })

        if "error" in gui_result:
            return json.dumps(gui_result)

        # Override with user's confirmed coordinates
        pixel_x = gui_result["pixel_x"]
        pixel_y = gui_result["pixel_y"]

        # Re-check bounds after user adjustment
        if pixel_x < 0 or pixel_x >= w or pixel_y < 0 or pixel_y >= h:
            return json.dumps({
                "error": f"User-selected pixel ({pixel_x}, {pixel_y}) out of bounds. "
                         f"Image size: {w}x{h}.",
            })

    # Look up 3D position from point cloud
    xyz = get_3d_position(cloud, pixel_x, pixel_y, search_radius)

    if xyz is None:
        return json.dumps({
            "error": f"No valid depth at pixel ({pixel_x}, {pixel_y}) "
                     f"within search_radius={search_radius}. "
                     "The point may be on a reflective surface or out of range.",
        })

    result = {
        "pixel_x": pixel_x,
        "pixel_y": pixel_y,
        "camera_xyz": [round(v, 6) for v in xyz],
        "camera": camera,
        "camera_frame": camera_frame,
        "base_xyz": None,
        "base_frame": "base_link",
    }

    # Transform to base_link
    if transform_to_base:
        base_xyz = await _transform_point_to_base(
            node, xyz, camera_frame, stamp,
        )
        if base_xyz is not None:
            result["base_xyz"] = [round(v, 6) for v in base_xyz]

    # Annotate image with the queried point
    annotated = cv2.cvtColor(rgb.copy(), cv2.COLOR_RGB2BGR)
    cv2.drawMarker(
        annotated, (pixel_x, pixel_y), (0, 0, 255),
        cv2.MARKER_CROSS, 20, 2,
    )
    label_parts = [f"({pixel_x}, {pixel_y})"]
    if result["base_xyz"]:
        bx, by, bz = result["base_xyz"]
        label_parts.append(f"base: ({bx:.3f}, {by:.3f}, {bz:.3f})")
    else:
        cx, cy, cz = result["camera_xyz"]
        label_parts.append(f"cam: ({cx:.3f}, {cy:.3f}, {cz:.3f})")
    label = " ".join(label_parts)
    cv2.putText(
        annotated, label, (pixel_x + 15, pixel_y - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2,
    )
    cv2.putText(
        annotated, label, (pixel_x + 15, pixel_y - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1,
    )
    os.makedirs(os.path.dirname(save_path) or "/tmp", exist_ok=True)
    cv2.imwrite(save_path, annotated)
    result["annotated_image_path"] = save_path

    return json.dumps(result)


@mcp.tool()
async def get_tf_transform(
    source_frame: str = "flange",
    target_frame: str = "base_link",
    timeout: float = 2.0,
) -> str:
    """Look up TF transform between two frames.

    Uses the persistent TF buffer that fills continuously in the background.
    Common frames: base_link, flange, tool0, zivid_optical_frame, world.

    IMPORTANT: Default is "flange" (MoveIt/ROS convention), NOT "tool0" (UR convention).
    flange and tool0 are at the same position but rotated by (-90°, -90°, 0°).
    Use "flange" when reading orientation for cartesian_target goals (MoveIt uses flange).
    Use "tool0" when comparing with UR teach pendant values.

    Args:
        source_frame: The frame to transform FROM. Default "flange" (MoveIt convention).
        target_frame: The frame to transform TO (e.g., "base_link").
        timeout: Max seconds to wait for transform availability.

    Returns:
        JSON with translation (xyz), rotation (quaternion + RPY in degrees),
        and the full 4x4 homogeneous transform matrix.
    """
    loop = asyncio.get_event_loop()

    def _lookup():
        node = bridge.node
        transform = node.lookup_transform(target_frame, source_frame, timeout_sec=timeout)
        if transform is None:
            return None
        return transform

    transform = await loop.run_in_executor(None, _lookup)

    if transform is None:
        return json.dumps({
            "error": f"Transform {source_frame} → {target_frame} not available. "
                     "Is the robot driver running?",
        })

    t = transform.transform.translation
    q = transform.transform.rotation

    # Compute RPY from quaternion
    from tf_transformations import euler_from_quaternion, quaternion_matrix

    rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
    rpy_deg = [math.degrees(a) for a in rpy]

    # Full 4x4 matrix
    mat = quaternion_matrix([q.x, q.y, q.z, q.w])
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z

    result = {
        "source_frame": source_frame,
        "target_frame": target_frame,
        "translation": {"x": round(t.x, 6), "y": round(t.y, 6), "z": round(t.z, 6)},
        "rotation_quaternion": {
            "x": round(q.x, 6), "y": round(q.y, 6),
            "z": round(q.z, 6), "w": round(q.w, 6),
        },
        "rotation_rpy_degrees": {
            "roll": round(rpy_deg[0], 4),
            "pitch": round(rpy_deg[1], 4),
            "yaw": round(rpy_deg[2], 4),
        },
        "matrix_4x4": [[round(float(mat[r, c]), 6) for c in range(4)] for r in range(4)],
    }
    return json.dumps(result)


BEAMBOT_LOG_FILE = "/tmp/beambot_launch.log"

# Regex to parse ROS2 launch log lines:
# [process_name-N] [LEVEL] [timestamp] [logger_name]: message
_LOG_LINE_RE = re.compile(
    r'^\[([^\]]+)\]\s+'          # [process_name-N]
    r'\[(\w+)\]\s+'              # [LEVEL]
    r'\[[\d.]+\]\s+'             # [timestamp]
    r'\[([^\]]+)\]:\s+'          # [logger_name]
    r'(.+)$'                     # message
)

_SEVERITY_ORDER = {"DEBUG": 10, "INFO": 20, "WARN": 30, "ERROR": 40, "FATAL": 50}


@mcp.tool()
async def get_recent_logs(
    severity: str = "ERROR",
    logger: str = "",
    count: int = 30,
) -> str:
    """Get recent ROS2 log messages from the beambot launch output.

    Reads from /tmp/beambot_launch.log (written by start_mcp.sh).
    Use this after a failure to understand what went wrong.

    Args:
        severity: Minimum severity level — "DEBUG", "INFO", "WARN", "ERROR", "FATAL".
            Default "ERROR" (shows ERROR and FATAL only).
        logger: Filter by logger name prefix (e.g., "beambot", "move_group",
            "moveit"). Empty string = all loggers.
        count: Maximum number of messages to return (most recent first).
            Default 30.

    Returns:
        JSON with list of log entries, each containing process, logger,
        severity, and message.
    """
    min_level = _SEVERITY_ORDER.get(severity.upper(), 40)

    if not os.path.exists(BEAMBOT_LOG_FILE):
        return json.dumps({
            "error": f"Log file not found: {BEAMBOT_LOG_FILE}. "
                     "Make sure beambot was started with start_mcp.sh.",
            "logs": [],
            "count": 0,
        })

    # Read last ~2000 lines (enough for recent activity without loading entire file)
    loop = asyncio.get_event_loop()
    def _read_tail():
        try:
            with open(BEAMBOT_LOG_FILE, 'r', errors='replace') as f:
                return collections.deque(f, maxlen=2000)
        except Exception as e:
            return str(e)

    lines = await loop.run_in_executor(None, _read_tail)
    if isinstance(lines, str):
        return json.dumps({"error": f"Failed to read log file: {lines}", "logs": [], "count": 0})

    # Parse and filter (iterate in reverse for most recent first)
    filtered = []
    for line in reversed(lines):
        m = _LOG_LINE_RE.match(line.strip())
        if not m:
            continue
        process, level, logger_name, message = m.groups()
        level_num = _SEVERITY_ORDER.get(level, 0)
        if level_num < min_level:
            continue
        if logger and not logger_name.startswith(logger):
            continue
        filtered.append({
            "process": process,
            "logger": logger_name,
            "severity": level,
            "message": message,
        })
        if len(filtered) >= count:
            break

    result = {
        "logs": filtered,
        "count": len(filtered),
        "filter": {
            "min_severity": severity,
            "logger_prefix": logger or "(all)",
            "max_count": count,
        },
    }
    return json.dumps(result)


# ---------------------------------------------------------------------------
# Vision Target Framework
# ---------------------------------------------------------------------------

# Direction opposites for grid offset sign flips
_DIRECTION_OPPOSITES = {
    "forward": "backward", "backward": "forward",
    "left": "right", "right": "left",
    "up": "down", "down": "up",
}


def _load_beamline_config() -> Dict:
    """Load the full beamline config from default_beamline.yaml."""
    try:
        from ament_index_python.packages import get_package_share_directory
        config_path = os.path.join(
            get_package_share_directory("beambot"), "config", "default_beamline.yaml"
        )
        with open(config_path) as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        logger.error(f"Failed to load beamline config: {e}")
        return {}


def _load_vision_targets() -> Dict:
    """Load vision target configs from default_beamline.yaml."""
    return _load_beamline_config().get("vision_targets", {})


def _load_grippers_config() -> Dict:
    """Load gripper configs (including dock_number) from default_beamline.yaml."""
    return _load_beamline_config().get("grippers", {})


def _load_tip_rack_config() -> Optional[Dict]:
    """Load legacy tip rack config from default_beamline.yaml."""
    return _load_beamline_config().get("tip_rack")


def _build_vision_target_tasks(
    target_name: str,
    element_index: int = 0,
    row: int = -1,
    col: int = -1,
    tag_id: int = -1,
) -> Dict:
    """Build task JSON for a vision target from config.

    For offset mode: moveto scan_pose → vision_moveto with marker-frame offsets.
    For grid mode: moveto scan_pose → vision_moveto → relative cartesian moves.

    Args:
        target_name: Name of the target in vision_targets config.
        element_index: For grid targets, 0-based index in row-major order.
        row: For grid targets, 0-indexed row (overrides element_index).
        col: For grid targets, 0-indexed column (overrides element_index).
        tag_id: Override marker ID from config (-1 = use config value).

    Returns:
        Dict with 'task_json' key on success, 'error' key on failure.
    """
    targets = _load_vision_targets()
    if target_name not in targets:
        available = list(targets.keys()) if targets else []
        return {"error": f"Unknown target '{target_name}'. Available: {available}"}

    cfg = targets[target_name]
    mode = cfg.get("mode", "offset")
    marker_id = tag_id if tag_id >= 0 else cfg["marker_id"]
    scan_pose_name = cfg.get("scan_pose", "")
    start_gripper = cfg.get("start_gripper", "pipettor")

    # Load scan pose from poses.yaml
    poses_dict = {}
    all_poses = _read_poses_file()
    if scan_pose_name and scan_pose_name in all_poses:
        poses_dict[scan_pose_name] = all_poses[scan_pose_name]

    tasks = []

    # Step 1: Move to scan position
    if scan_pose_name:
        tasks.append({"task_type": "moveto", "target": scan_pose_name})

    marker_offset = cfg.get("marker_offset", {})

    if mode == "offset":
        # Single vision_moveto with marker-frame offsets → direct move
        vision_step = {
            "task_type": "vision_moveto",
            "tag_id": marker_id,
            "marker_offset_x": float(marker_offset.get("x", 0.0)),
            "marker_offset_y": float(marker_offset.get("y", 0.0)),
            "marker_offset_z": float(marker_offset.get("z", 0.0)),
        }
        if "z_offset" in cfg:
            vision_step["z_offset"] = float(cfg["z_offset"])
        tasks.append(vision_step)

        task_json = json.dumps({
            "start_gripper": start_gripper,
            "tasks": tasks,
            "poses": poses_dict,
        })
        return {"task_json": task_json, "target": target_name, "mode": "offset"}

    elif mode == "grid":
        grid_cfg = cfg.get("grid", {})
        grid_rows = grid_cfg.get("rows", 1)
        grid_cols = grid_cfg.get("cols", 1)
        # Support separate row/col pitch, fall back to single pitch for backward compat
        default_pitch = grid_cfg.get("pitch", 0.009)
        col_pitch = grid_cfg.get("col_pitch", default_pitch)
        row_pitch = grid_cfg.get("row_pitch", default_pitch)
        # Flange directions for A1 offset from tag
        col_dir_a1 = grid_cfg.get("col_direction", "left")
        row_dir_a1 = grid_cfg.get("row_direction", "up")
        col_dir_away = _DIRECTION_OPPOSITES.get(col_dir_a1, "right")
        row_dir_away = _DIRECTION_OPPOSITES.get(row_dir_a1, "down")
        # Direction indices increase: defaults to opposite of A1 direction
        # (backward compat: tip_rack has A1 at max offset, indices go toward tag)
        col_increasing = grid_cfg.get("col_increasing", col_dir_away)
        row_increasing = grid_cfg.get("row_increasing", row_dir_away)

        # A1 offset from tag: use grid-level overrides if present,
        # else fall back to marker_offset for backward compat
        a1_col_offset = grid_cfg.get("col_offset", abs(marker_offset.get("x", 0.0)))
        a1_row_offset = grid_cfg.get("row_offset", abs(marker_offset.get("y", 0.0)))

        # Resolve row/col from index
        if row >= 0 and col >= 0:
            if row >= grid_rows or col >= grid_cols:
                return {"error": f"row={row}, col={col} out of range "
                                 f"(max: {grid_rows-1}, {grid_cols-1})"}
        else:
            total = grid_rows * grid_cols
            if element_index < 0 or element_index >= total:
                return {"error": f"element_index={element_index} out of range (0-{total-1})"}
            row = element_index // grid_cols
            col = element_index % grid_cols

        # Compute flange-frame offsets for this grid element
        # A1 is at a1_offset in the A1-ward direction.
        # Grid displacement goes in the col/row_increasing direction.
        # If increasing == A1 direction: ADD (elements go further from tag)
        # If increasing != A1 direction: SUBTRACT (elements go back toward tag)
        if col_increasing == col_dir_a1:
            col_offset = a1_col_offset + col * col_pitch
        else:
            col_offset = a1_col_offset - col * col_pitch

        if row_increasing == row_dir_a1:
            row_offset = a1_row_offset + row * row_pitch
        else:
            row_offset = a1_row_offset - row * row_pitch

        col_dir = col_dir_a1 if col_offset >= 0 else col_dir_away
        col_dist = abs(col_offset)
        row_dir = row_dir_a1 if row_offset >= 0 else row_dir_away
        row_dist = abs(row_offset)

        # Vision alignment step (move to marker)
        tasks.append({"task_type": "vision_moveto", "tag_id": marker_id})

        # Build moves from config, replacing sentinels with computed offsets
        for move in cfg.get("moves", []):
            if move == "column_offset":
                if col_dist > 1e-6:  # Skip zero-distance moves
                    tasks.append({
                        "task_type": "moveto", "target": "",
                        "planning_type": "cartesian",
                        "direction": col_dir,
                        "distance": round(col_dist, 6),
                    })
            elif move == "row_offset":
                if row_dist > 1e-6:  # Skip zero-distance moves
                    tasks.append({
                        "task_type": "moveto", "target": "",
                        "planning_type": "cartesian",
                        "direction": row_dir,
                        "distance": round(row_dist, 6),
                    })
            elif isinstance(move, dict) and "direction" in move:
                tasks.append({
                    "task_type": "moveto", "target": "",
                    "planning_type": "cartesian",
                    "direction": move["direction"],
                    "distance": float(move["distance"]),
                })

        task_json = json.dumps({
            "start_gripper": start_gripper,
            "tasks": tasks,
            "poses": poses_dict,
        })
        return {
            "task_json": task_json,
            "target": target_name,
            "mode": "grid",
            "element": {"row": row, "col": col, "index": row * grid_cols + col},
            "offsets_mm": {
                col_dir: round(col_dist * 1000, 1),
                row_dir: round(row_dist * 1000, 1),
            },
        }

    else:
        return {"error": f"Unknown mode '{mode}' for target '{target_name}'"}


@mcp.tool()
async def vision_target(
    target_name: str,
    element_index: int = 0,
    row: int = -1,
    col: int = -1,
    tag_id: int = -1,
) -> str:
    """Build task JSON for a config-driven vision target operation.

    Vision targets are defined in default_beamline.yaml under 'vision_targets'.
    Each target uses ArUco marker detection to locate the target, then either:
      - offset mode: moves directly to a marker-relative position (single point)
      - grid mode: aligns with marker, then does relative moves to grid element

    Args:
        target_name: Name of the vision target from config (e.g. "tip_rack",
            "sample", "vial_rack").
        element_index: For grid targets: 0-based element index in row-major order.
            Ignored if row/col are specified. Ignored for offset targets.
        row: For grid targets: 0-indexed row. Use with col.
        col: For grid targets: 0-indexed column. Use with row.
        tag_id: Override the marker ID from config. Use for targets like "sample"
            where the same offset applies to different markers. -1 = use config value.

    Returns:
        JSON with task_json to send to the orchestrator via send_action_goal.
    """
    result = _build_vision_target_tasks(target_name, element_index, row, col, tag_id=tag_id)
    return json.dumps(result, indent=2)


@mcp.tool()
async def pickup_tip(
    tip_index: int = 0,
    row: int = -1,
    col: int = -1,
) -> str:
    """Build task JSON for picking up a pipettor tip from the tip rack.

    Convenience wrapper around vision_target(target_name="tip_rack").
    Specify the tip by either tip_index (0-95, row-major) or row + col (0-indexed).

    Args:
        tip_index: Tip number 0-95 in row-major order. Ignored if row/col given.
        row: Row index 0-7 (A-H). Use with col.
        col: Column index 0-11 (1-12). Use with row.

    Returns:
        JSON with task_json to send to the orchestrator.
    """
    result = _build_vision_target_tasks("tip_rack", tip_index, row, col)
    return json.dumps(result, indent=2)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    # Configure logging to stderr (stdout is reserved for MCP stdio transport)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        stream=sys.stderr,
    )
    logger.info("Starting EROBS MCP server...")

    # Write crash logs to file since stderr may not be visible
    crash_log = "/tmp/beambot_mcp_crash.log"

    try:
        mcp.run(transport="stdio")
    except Exception:
        import traceback
        tb = traceback.format_exc()
        logger.error(f"MCP server crashed:\n{tb}")
        with open(crash_log, "a") as f:
            f.write(f"\n{'='*60}\n{time.strftime('%Y-%m-%d %H:%M:%S')}\n{tb}\n")
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    main()
