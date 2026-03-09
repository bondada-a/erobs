#!/usr/bin/env python3
"""EROBS MCP Server — Custom tools for Zivid camera, detection, and TF.

Runs alongside ros-mcp-server to provide EROBS-specific tools that handle
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
import json
import logging
import math
import os
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
from rcl_interfaces.msg import Log
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from cv_bridge import CvBridge

logger = logging.getLogger("erobs-mcp")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
# Zivid topics (single-shot, triggered)
ZIVID_IMAGE_TOPIC = "/color/image_color"
ZIVID_CLOUD_TOPIC = "/points/xyzrgba"
ZIVID_CAPTURE_SERVICE = "/capture"
ZIVID_FRAME = "zivid_optical_frame"

# ZED topics (continuous streaming)
ZED_IMAGE_TOPIC = "/zed/zed_node/rgb/image_rect_color"
ZED_CLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered"
ZED_FRAME = "zed_left_camera_optical_frame"

CAMERA_FRAME = ZIVID_FRAME  # Default camera frame (backward compat)

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

# Pose registry
POSES_FILE = os.environ.get(
    "EROBS_POSES_FILE",
    os.path.join(os.path.dirname(__file__), "..", "..", "cms", "poses.yaml"),
)

# Default save locations
DEFAULT_IMAGE_PATH = "/tmp/erobs_capture.jpg"
DEFAULT_ANNOTATED_PATH = "/tmp/erobs_detection.jpg"


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
    aruco_dict = cv2.aruco.Dictionary_get(dict_id)
    aruco_params = cv2.aruco.DetectorParameters_create()
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
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
        super().__init__("erobs_mcp_bridge")
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

        # /rosout log buffer
        self._log_buffer = []
        self._log_buffer_max = 200
        self._log_buffer_lock = threading.Lock()

        self._rosout_sub = self.create_subscription(
            Log, '/rosout', self._on_rosout, 10,
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

        # Robot state (updated by callbacks, None = no data received yet)
        self.joint_names: Optional[List[str]] = None
        self.joint_positions: Optional[List[float]] = None
        self.current_gripper: Optional[str] = None
        self.execution_state: Optional[str] = None

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

    def _on_rosout(self, msg: Log):
        with self._log_buffer_lock:
            self._log_buffer.append(msg)
            if len(self._log_buffer) > self._log_buffer_max:
                self._log_buffer = self._log_buffer[-self._log_buffer_max:]

    def get_filtered_logs(self, min_severity: int = 40, logger_prefix: str = "", count: int = 20):
        """Get recent logs filtered by severity and logger name.

        Severity levels: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
        """
        with self._log_buffer_lock:
            filtered = []
            for msg in reversed(self._log_buffer):
                if msg.level < min_severity:
                    continue
                if logger_prefix and not msg.name.startswith(logger_prefix):
                    continue
                filtered.append({
                    "timestamp": f"{msg.stamp.sec}.{msg.stamp.nanosec:09d}",
                    "logger": msg.name,
                    "severity": {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}.get(msg.level, f"UNKNOWN({msg.level})"),
                    "message": msg.msg,
                })
                if len(filtered) >= count:
                    break
            return filtered

    def trigger_capture(self, timeout: float = 30.0, need_cloud: bool = True) -> bool:
        """Trigger Zivid capture and wait for data. Called from executor thread.

        Returns True if data received within timeout.
        """
        if not self._capture_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Capture service '{CAPTURE_SERVICE}' not available")
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
# MCP Server
# ---------------------------------------------------------------------------

mcp = FastMCP("erobs")

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

    result = {
        "system_running": system_running,
        "gripper": gripper,
        "execution_state": exec_state,
        "joints_deg": joints_deg,
    }
    return json.dumps(result, indent=2)


def _read_poses_file() -> dict:
    """Read the poses YAML file. Returns empty dict if file doesn't exist."""
    path = os.path.realpath(POSES_FILE)
    if not os.path.exists(path):
        return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return data


def _write_poses_file(poses: dict):
    """Write poses dict to the YAML file."""
    path = os.path.realpath(POSES_FILE)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        yaml.dump(poses, f, default_flow_style=None, width=200)


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
        save_path: Where to save the captured image. Default /tmp/erobs_capture.jpg.
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
                             "Check: ros2 topic hz /zed/zed_node/rgb/image_rect_color",
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
    transform_to_base: bool = True,
    save_path: str = DEFAULT_ANNOTATED_PATH,
) -> str:
    """Detect objects in the last captured image.

    IMPORTANT: Call capture_image(mode="3d") first! This operates on the most
    recent capture data.

    Detection methods:
        - "hsv_color": Find objects by color in HSV space. Good for colored balls,
          samples with distinctive hues. Tune hue_low/hue_high for your target color.
          Common ranges: blue=100-130, red=0-10 or 170-180, green=35-85, yellow=20-35.
        - "circle": Hough circle detection. Good for round samples/wafers.
        - "contour": Edge-based contour detection. Finds any shape.
        - "marker": ArUco marker detection (2D image-based, not Zivid native).

    Args:
        method: Detection method — "hsv_color", "circle", "contour", or "marker".
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
        transform_to_base: If True, transform 3D positions from camera frame to base_link.
        save_path: Where to save annotated image. Default /tmp/erobs_detection.jpg.

    Returns:
        JSON with list of detections, each containing pixel coords, 3D position
        (camera and/or base frame), and an annotated image path.
    """
    node = bridge.node

    if node.last_rgb is None:
        return json.dumps({
            "error": "No image data. Call capture_image() first.",
        })

    rgb = node.last_rgb
    cloud = node.last_cloud

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
    else:
        return json.dumps({"error": f"Unknown method '{method}'. Use: hsv_color, circle, contour, marker"})

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
                        node, xyz, node.capture_stamp,
                    )
                    if base_xyz is not None:
                        det["base_xyz"] = list(base_xyz)

    # Annotate and save image
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
        "coordinate_frame": "base_link" if transform_to_base else CAMERA_FRAME,
    }
    return json.dumps(result)


async def _transform_point_to_base(
    node: ROS2BridgeNode,
    camera_xyz: Tuple[float, float, float],
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
                "base_link", CAMERA_FRAME, stamp=capture_stamp,
            )
            if transform is None:
                logger.warning(
                    "TF lookup at capture_stamp failed, falling back to latest transform"
                )

        # Fallback: latest available transform
        if transform is None:
            transform = node.lookup_transform(
                "base_link", CAMERA_FRAME, stamp=None,
            )
            if transform is None:
                logger.error(
                    f"TF lookup failed: {CAMERA_FRAME} → base_link not available "
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


@mcp.tool()
async def get_point_3d(
    pixel_x: int,
    pixel_y: int,
    transform_to_base: bool = True,
    search_radius: int = 10,
    confirm: bool = True,
    save_path: str = "/tmp/erobs_point3d.jpg",
) -> str:
    """Get the 3D position of a pixel from the last captured point cloud.

    IMPORTANT: Call capture_image(mode="3d") first! This uses the most recent
    point cloud data.

    Use this when you can see something in the camera image and want to know
    its real-world 3D position — e.g., "what are the 3D coordinates of the
    object at pixel (500, 300)?" This is useful for planning robot moves to
    arbitrary points visible in the image without needing a specific detector.

    Args:
        pixel_x: X coordinate (column) in the image.
        pixel_y: Y coordinate (row) in the image.
        transform_to_base: If True, return position in base_link frame.
            If False, return in camera (zivid_optical_frame) frame.
        search_radius: If the exact pixel has no depth, search nearby pixels
            within this radius. Default 10.
        confirm: If True, open a GUI window showing the image with the
            suggested point. The user can click to adjust the position,
            then press Enter to confirm or Esc to cancel. Default False.
        save_path: Where to save annotated image showing the queried point.

    Returns:
        JSON with camera_xyz, base_xyz (if transform_to_base), and an
        annotated image showing the queried point.
    """
    node = bridge.node

    if node.last_cloud is None:
        return json.dumps({
            "error": "No point cloud data. Call capture_image(mode='3d') first.",
        })

    if node.last_rgb is None:
        return json.dumps({
            "error": "No image data. Call capture_image(mode='3d') first.",
        })

    # Bounds check
    h, w = node.last_rgb.shape[:2]
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
            bgr = cv2.cvtColor(node.last_rgb, cv2.COLOR_RGB2BGR)
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
    xyz = get_3d_position(node.last_cloud, pixel_x, pixel_y, search_radius)

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
        "camera_frame": CAMERA_FRAME,
        "base_xyz": None,
        "base_frame": "base_link",
    }

    # Transform to base_link
    if transform_to_base:
        base_xyz = await _transform_point_to_base(
            node, xyz, node.capture_stamp,
        )
        if base_xyz is not None:
            result["base_xyz"] = [round(v, 6) for v in base_xyz]

    # Annotate image with the queried point
    annotated = cv2.cvtColor(node.last_rgb.copy(), cv2.COLOR_RGB2BGR)
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


@mcp.tool()
async def get_recent_logs(
    severity: str = "ERROR",
    logger: str = "",
    count: int = 20,
) -> str:
    """Get recent ROS2 log messages from /rosout.

    Use this after a failure to understand what went wrong. Filters by
    minimum severity level and optional logger name prefix.

    Args:
        severity: Minimum severity level — "DEBUG", "INFO", "WARN", "ERROR", "FATAL".
            Default "ERROR" (shows ERROR and FATAL only).
        logger: Filter by logger name prefix (e.g., "beambot", "move_group",
            "moveit"). Empty string = all loggers.
        count: Maximum number of messages to return (most recent first).
            Default 20.

    Returns:
        JSON with list of log entries, each containing timestamp, logger,
        severity, and message.
    """
    severity_map = {"DEBUG": 10, "INFO": 20, "WARN": 30, "ERROR": 40, "FATAL": 50}
    min_level = severity_map.get(severity.upper(), 40)

    node = bridge.node
    logs = node.get_filtered_logs(
        min_severity=min_level,
        logger_prefix=logger,
        count=count,
    )

    result = {
        "logs": logs,
        "count": len(logs),
        "filter": {
            "min_severity": severity,
            "logger_prefix": logger or "(all)",
            "max_count": count,
        },
    }
    return json.dumps(result)


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

    try:
        mcp.run(transport="stdio")
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    main()
