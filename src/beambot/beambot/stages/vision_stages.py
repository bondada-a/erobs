"""VisionStages - Python equivalent of vision_stages.cpp.

Handles vision-guided robot movement:
- ArUco marker detection via Zivid camera
- Circle/object detection via Hough Transform
- TF transforms from camera to base frame
- Collision object management
- Motion to detected poses
"""

import json
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from moveit.task_constructor import stages
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import GetPlanningScene
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from tf_transformations import quaternion_multiply, quaternion_from_euler, quaternion_matrix, euler_from_quaternion

from beambot.camera import get_camera
from beambot.camera.zivid import DetectionResult
from beambot.stages.base_stages import BaseStages, DEFAULT_JOINT_NAMES, DIRECTION_VECTORS


@dataclass
class ObjectInfo:
    """Information about a collision object associated with a marker."""
    name: str
    shape: str
    dimensions: list
    tag_offset: list


@dataclass
class GripperDetection:
    """Auto-detected gripper frame and offset."""
    ik_frame: str
    z_offset: float


class VisionStages(BaseStages):
    """Handles vision-guided movement to ArUco markers and detected objects."""

    # Detection types
    DETECTION_MARKER = "marker"
    DETECTION_CIRCLE = "circle"
    DETECTION_CONTOUR = "contour"

    # Retry configuration defaults
    DEFAULT_RETRY_COUNT = 3  # Number of retries after first attempt
    DEFAULT_RETRY_DELAY = 0.5  # Seconds between retries

    # Default camera settings (used if not specified)
    DEFAULT_CAMERA_TYPE = "zivid"
    DEFAULT_CAMERA_FRAME = "zivid_optical_frame"
    DEFAULT_MARKER_DICTIONARY = "aruco4x4_50"

    # Settle time: wait before capture for robot vibration to dampen
    # Set to 0.3-0.5s for high-accuracy applications if needed
    DEFAULT_SETTLE_TIME = 0.0  # Disabled for now - testing TF timestamp fix

    def __init__(
        self,
        rclpy_node,
        arm_group: str = "",
        ik_frame: str = "",
        camera_type: str = None,
        camera_frame: str = None,
        marker_dictionary: str = None,
        retry_count: int = None,
        retry_delay: float = None,
        settle_time: float = None
    ):
        """Initialize VisionStages.

        Args:
            rclpy_node: ROS node for service calls and TF
            arm_group: MoveIt planning group for arm
            ik_frame: IK frame (empty = auto-detect)
            camera_type: Camera type from beamline config (default: "zivid")
            camera_frame: Camera TF frame (default: "zivid_optical_frame")
            marker_dictionary: ArUco dictionary (default: "aruco4x4_50")
            retry_count: Number of detection retries (default: 3)
            retry_delay: Delay between retries in seconds (default: 0.5)
            settle_time: Seconds to wait before capture for robot to settle (default: 0.3)
        """
        super().__init__(rclpy_node, arm_group, ik_frame=ik_frame)

        # Camera configuration
        self._camera_type = camera_type if camera_type else self.DEFAULT_CAMERA_TYPE
        self._camera_frame = camera_frame if camera_frame else self.DEFAULT_CAMERA_FRAME
        self._marker_dictionary = marker_dictionary if marker_dictionary else self.DEFAULT_MARKER_DICTIONARY

        # Retry configuration
        self._retry_count = retry_count if retry_count is not None else self.DEFAULT_RETRY_COUNT
        self._retry_delay = retry_delay if retry_delay is not None else self.DEFAULT_RETRY_DELAY

        # Settle time before capture (for robot vibration damping)
        self._settle_time = settle_time if settle_time is not None else self.DEFAULT_SETTLE_TIME

        # TF2
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, rclpy_node)
        self._tf_broadcaster = TransformBroadcaster(rclpy_node)

        # Camera module and service client
        self._camera = get_camera(self._camera_type)
        self._capture_client = self._camera.create_client(rclpy_node)

        # Planning scene publisher (same pattern as moveit_lifecycle_manager)
        scene_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self._planning_scene_pub = rclpy_node.create_publisher(
            PlanningScene, "/planning_scene", scene_qos
        )

        # Service client for querying known objects
        self._get_scene_client = rclpy_node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )


        # Parameters
        self._publish_marker_frames = True

        # Object database (loaded from config)
        self._object_database: Dict[int, ObjectInfo] = {}

        # Load vision objects config
        self._load_vision_objects_config()

        # Cache for multi-position scan results
        # Populated by scan_all_tags(), used by run() for fast lookup
        # {tag_id: PoseStamped} - averaged poses from vision_scan
        self._tag_pose_cache: Dict[int, PoseStamped] = {}

        self.logger.info(
            f"VisionStages initialized (camera: {self._camera_type}, "
            f"frame: {self._camera_frame}, ik_frame: "
            f"{'auto-detect' if not ik_frame else ik_frame}, "
            f"settle_time: {self._settle_time:.2f}s)"
        )

    def reset_tf(self):
        """Reset the TF buffer and re-subscribe to /tf_static.

        Call after a tool exchange when robot_state_publisher restarts with
        a new URDF. Clears stale static transforms from the old gripper
        and re-subscribes to pick up the new publisher's frames.
        """
        self.logger.info("Resetting TF buffer (URDF changed)")
        self._tf_buffer.clear()
        # Recreate listener to trigger TRANSIENT_LOCAL re-delivery
        # from the new robot_state_publisher
        self._tf_listener = TransformListener(self._tf_buffer, self.rclpy_node)
        self.logger.info("TF buffer cleared, listener re-created")

    def _load_vision_objects_config(self):
        """Load vision objects configuration from JSON file."""
        try:
            config_path = (
                get_package_share_directory("beambot") +
                "/config/vision_objects.json"
            )
            with open(config_path, 'r') as f:
                config = json.load(f)

            if "vision_objects" not in config:
                return

            for tag_str, obj in config["vision_objects"].items():
                info = ObjectInfo(
                    name=obj["name"],
                    shape=obj["shape"],
                    dimensions=obj["dimensions"],
                    tag_offset=obj["tag_offset"]
                )
                self._object_database[int(tag_str)] = info

            self.logger.info(f"Loaded {len(self._object_database)} vision objects")

        except FileNotFoundError:
            self.logger.warn("vision_objects.json not found, collision objects disabled")
        except Exception as e:
            self.logger.error(f"Failed to load vision objects config: {e}")

    # =========================================================================
    # Tag Pose Cache Methods
    # =========================================================================

    def clear_cache(self):
        """Clear the tag pose cache."""
        count = len(self._tag_pose_cache)
        self._tag_pose_cache.clear()
        self.logger.info(f"Tag pose cache cleared ({count} entries)")

    def get_cached_pose(self, tag_id: int) -> Optional[PoseStamped]:
        """Get cached pose for tag, or None if not cached."""
        return self._tag_pose_cache.get(tag_id)

    def scan_all_tags(
        self,
        scan_positions: List[List[float]],
        scans_per_position: int = 3,
        timeout: float = 10.0,
        settle_time: float = 0.3
    ) -> int:
        """Scan from multiple positions, detect ALL tags, cache averaged poses.

        This method moves the robot to each scan position, performs multiple
        captures at each position to improve detection reliability, detects
        ALL visible ArUco markers, and caches the averaged pose for each tag.

        Args:
            scan_positions: List of joint configurations (6 joints each, radians)
            scans_per_position: Number of captures at each position (default: 3)
            timeout: Detection timeout per capture in seconds
            settle_time: Wait time after move for vibration damping

        Returns:
            Number of unique tags detected and cached
        """
        # Clear previous cache
        self._tag_pose_cache.clear()

        # Collect all detections: {tag_id: [PoseStamped, ...]}
        all_detections: Dict[int, List[PoseStamped]] = {}

        total_scans = len(scan_positions) * scans_per_position
        self.logger.info(
            f"Starting batch scan: {len(scan_positions)} positions × "
            f"{scans_per_position} scans = {total_scans} total captures"
        )

        for pos_idx, joint_pose in enumerate(scan_positions):
            self.logger.info(f"Position {pos_idx+1}/{len(scan_positions)}")

            # Move to scan position
            if not self._move_to_joint_pose(joint_pose):
                self.logger.warn(f"Failed to reach position {pos_idx+1}, skipping")
                continue

            # Wait for robot to settle (vibration damping)
            if settle_time > 0:
                time.sleep(settle_time)

            # Multiple scans at this position
            for scan_idx in range(scans_per_position):
                self.logger.info(f"  Scan {scan_idx+1}/{scans_per_position}")

                # Detect ALL markers (no specific tag_id filter)
                result = self._camera.detect_markers(
                    self._capture_client,
                    self.rclpy_node,
                    marker_ids=None,  # Detect all markers
                    dictionary=self._marker_dictionary,
                    timeout=timeout,
                    settle_time=0  # Already settled after move
                )

                if not result.markers:
                    self.logger.debug("    No markers in this scan")
                    continue

                # Process each detected marker
                for marker_id, marker_pose in result.markers:
                    pose_base = self._transform_to_base_link(
                        marker_pose,
                        capture_stamp=result.capture_stamp
                    )
                    if pose_base:
                        all_detections.setdefault(marker_id, []).append(pose_base)

                self.logger.info(f"    Detected {len(result.markers)} markers")

        # Average all detections per tag and cache
        self.logger.info(f"Processing {len(all_detections)} unique tags...")
        for tag_id, poses in sorted(all_detections.items()):
            if len(poses) >= 2:
                averaged = self._average_poses(poses)
                if averaged:
                    self._tag_pose_cache[tag_id] = averaged
                    pos = averaged.pose.position
                    self.logger.info(
                        f"  Tag {tag_id}: [{pos.x*1000:.2f}, {pos.y*1000:.2f}, "
                        f"{pos.z*1000:.2f}] mm (from {len(poses)} detections)"
                    )
            else:
                self.logger.warn(
                    f"  Tag {tag_id}: only {len(poses)} detection(s), need ≥2 for averaging"
                )

        self.logger.info(
            f"Batch scan complete: {len(self._tag_pose_cache)} tags cached"
        )
        return len(self._tag_pose_cache)

    def run(self, goal) -> 'Optional[str]':
        """Execute VisionMoveTo action.

        Args:
            goal: VisionMoveToAction.Goal with:
                - tag_id: Marker ID (for marker detection)
                - timeout: Detection timeout
                - detection_type: "marker" (default) or "circle"
                - z_offset: Override z_offset (0 = use gripper default)
                - scan_positions_flat: Flattened joint poses for multi-position mode
                - num_scan_positions: Number of scan positions (0 = single-position)
                - detect_only: If true, detect and return position without moving

        Returns:
            None if successful, error string describing failure otherwise
        """
        # Clear any previous detect_only result
        self.last_detected_pose = None
        # Optional: Wait for robot to settle BEFORE any detection
        # This ensures vibrations from the previous motion have damped out
        if self._settle_time > 0:
            self.logger.info(f"Waiting {self._settle_time:.2f}s for robot to settle...")
            settle_iterations = int(self._settle_time / 0.05)  # 50ms intervals
            for _ in range(settle_iterations):
                rclpy.spin_once(self.rclpy_node, timeout_sec=0.05)
            self.logger.info("Settle complete, starting detection")

        # Parse scan positions if provided (multi-position averaging mode)
        scan_positions = None
        num_positions = getattr(goal, 'num_scan_positions', 0)
        if num_positions > 0:
            flat = list(getattr(goal, 'scan_positions_flat', []))
            if len(flat) == num_positions * 6:
                scan_positions = [flat[i*6:(i+1)*6] for i in range(num_positions)]
                self.logger.info(
                    f"Multi-position mode enabled: {num_positions} scan positions"
                )
            else:
                self.logger.warn(
                    f"Invalid scan_positions_flat length: {len(flat)}, "
                    f"expected {num_positions * 6}. Falling back to single-position."
                )

        # Determine detection type (default to marker for backwards compatibility)
        detection_type = getattr(goal, 'detection_type', '') or self.DETECTION_MARKER

        # Get z_offset override (0 or missing means use gripper default)
        z_offset_override = getattr(goal, 'z_offset', 0.0)

        # Check detect_only flag
        detect_only = getattr(goal, 'detect_only', False)

        # Parse offset parameters
        offset_direction = getattr(goal, 'offset_direction', '') or ''
        offset_distance = getattr(goal, 'offset_distance', 0.0)

        # Parse marker-frame offset parameters
        marker_offset_x = getattr(goal, 'marker_offset_x', 0.0)
        marker_offset_y = getattr(goal, 'marker_offset_y', 0.0)
        marker_offset_z = getattr(goal, 'marker_offset_z', 0.0)

        # IK frame override from orchestrator (avoids stale TF auto-detection)
        goal_ik_frame = getattr(goal, 'ik_frame', '') or ''

        # Route to appropriate detection method
        if detection_type == self.DETECTION_CIRCLE:
            self.logger.info("Using circle detection")
            target_pose = self.detect_and_transform_circle(goal.timeout)
            if target_pose is None:
                return "DETECTION_FAILED: Circle/wafer detection failed (no circles found in image)"
        elif detection_type == self.DETECTION_CONTOUR:
            # Get sample_index (default to 1 if not specified or 0)
            sample_index = getattr(goal, 'sample_index', 1)
            if sample_index <= 0:
                sample_index = 1
            self.logger.info(f"Using contour detection (any shape), sample #{sample_index}")
            target_pose = self.detect_and_transform_contour(
                sample_index=sample_index,
                timeout=goal.timeout
            )
            if target_pose is None:
                return f"DETECTION_FAILED: Contour detection failed for sample #{sample_index}"
        else:
            # Marker detection - check cache first (populated by vision_scan task)
            cached_pose = self.get_cached_pose(goal.tag_id)
            if cached_pose is not None:
                pos = cached_pose.pose.position
                self.logger.info(
                    f"Using cached pose for tag {goal.tag_id}: "
                    f"[{pos.x*1000:.2f}, {pos.y*1000:.2f}, {pos.z*1000:.2f}] mm"
                )
                target_pose = cached_pose
            elif scan_positions is not None:
                # Multi-position averaging mode
                target_pose = self.detect_tag_multiposition(
                    tag_id=goal.tag_id,
                    scan_positions=scan_positions,
                    timeout=goal.timeout,
                    settle_time=self._settle_time
                )
                if target_pose is None:
                    return (
                        f"DETECTION_FAILED: Multi-position detection failed for "
                        f"ArUco tag {goal.tag_id} ({num_positions} positions attempted)"
                    )
            else:
                # Single-position mode (existing behavior)
                target_pose = self.detect_and_transform_tag(goal.tag_id, goal.timeout)
                if target_pose is None:
                    return (
                        f"DETECTION_FAILED: ArUco tag {goal.tag_id} not detected "
                        f"(timeout: {goal.timeout}s, retries: {self._retry_count})"
                    )

        # Compute the full approach pose (marker offset, z_offset, orientation)
        approach, active_ik_frame = self._compute_approach_pose(
            target_pose, z_offset_override,
            marker_offset_x=marker_offset_x,
            marker_offset_y=marker_offset_y,
            marker_offset_z=marker_offset_z,
            ik_frame_override=goal_ik_frame,
        )

        # Apply directional offset in flange frame if specified
        if offset_direction and offset_distance > 0:
            approach = self._apply_flange_offset(approach, offset_direction, offset_distance)

        # If detect_only, return the pose without moving
        if detect_only:
            self.last_detected_pose = approach
            pos = approach.pose.position
            self.logger.info(
                f"Detect-only: returning approach pose [{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}] "
                f"in {approach.header.frame_id}"
            )
            return None

        return self._move_to_approach(approach, ik_frame=active_ik_frame)

    def detect_and_transform_tag(
        self,
        tag_id: int,
        timeout: float = 45.0
    ) -> Optional[PoseStamped]:
        """Detect an ArUco marker and transform to base_link frame.

        Includes retry logic for transient detection failures.

        Args:
            tag_id: ArUco marker ID to detect
            timeout: Detection timeout in seconds (per attempt)

        Returns:
            PoseStamped in base_link frame, or None if all retries exhausted
        """
        total_attempts = 1 + self._retry_count  # Initial attempt + retries
        last_error = "Unknown error"

        for attempt in range(total_attempts):
            if attempt > 0:
                self.logger.info(
                    f"Retry {attempt}/{self._retry_count} for tag {tag_id} "
                    f"(waiting {self._retry_delay}s...)"
                )
                time.sleep(self._retry_delay)

            result = self._single_detection_attempt(tag_id, timeout)

            if result is not None:
                # Success - process the detection
                pose_base = self._process_detection_result(tag_id, result)
                if pose_base is not None:
                    if attempt > 0:
                        self.logger.info(
                            f"Tag {tag_id} detected on retry {attempt}"
                        )
                    return pose_base
                else:
                    last_error = f"Tag {tag_id} not found in detection results"
            else:
                last_error = f"Detection attempt failed"

        self.logger.error(
            f"Failed to detect tag {tag_id} after {total_attempts} attempts: {last_error}"
        )
        return None

    def _single_detection_attempt(
        self,
        tag_id: int,
        timeout: float
    ) -> Optional[DetectionResult]:
        """Execute a single detection attempt using camera module.

        Args:
            tag_id: ArUco marker ID to detect
            timeout: Detection timeout in seconds

        Returns:
            DetectionResult with markers and capture timestamp if successful, None on error
        """
        self.logger.info(f"Detecting tag {tag_id}...")

        # Use camera module for detection
        result = self._camera.detect_markers(
            self._capture_client,
            self.rclpy_node,
            marker_ids=[tag_id],
            dictionary=self._marker_dictionary,
            timeout=timeout,
            settle_time=self._settle_time
        )

        if not result.markers:
            self.logger.warn("No markers detected")
            return None

        return result

    def _process_detection_result(
        self,
        tag_id: int,
        detection_result: DetectionResult
    ) -> Optional[PoseStamped]:
        """Process detection result and transform to base_link.

        Uses the capture timestamp from the detection result for TF lookup
        to ensure the transform matches the robot pose at capture time.

        Args:
            tag_id: ArUco marker ID to find
            detection_result: DetectionResult with markers and capture timestamp

        Returns:
            PoseStamped in base_link frame, or None if tag not found
        """
        # Find the requested marker in results
        for marker_id, marker_pose in detection_result.markers:
            if marker_id == tag_id:
                self.logger.info(
                    f"Tag {marker_id} at [{marker_pose.position.x:.3f}, "
                    f"{marker_pose.position.y:.3f}, {marker_pose.position.z:.3f}] "
                    f"in camera frame"
                )

                # Transform to base_link using capture timestamp
                # This ensures we use the TF at the exact moment the image was captured
                pose_base = self._transform_to_base_link(
                    marker_pose,
                    capture_stamp=detection_result.capture_stamp
                )
                if pose_base is None:
                    return None

                self.logger.info(
                    f"Transformed to base_link: [{pose_base.pose.position.x:.3f}, "
                    f"{pose_base.pose.position.y:.3f}, {pose_base.pose.position.z:.3f}]"
                )
                # Debug: pose in mm for easier comparison with TF analysis
                self.logger.debug(
                    f"  BASE_LINK pose (mm): ({pose_base.pose.position.x*1000:.2f}, "
                    f"{pose_base.pose.position.y*1000:.2f}, {pose_base.pose.position.z*1000:.2f})"
                )

                # Optional: broadcast TF frame for RViz
                if self._publish_marker_frames:
                    self._broadcast_marker_tf(tag_id, pose_base)

                # Add collision object if configured
                self._add_collision_object_for_tag(tag_id, pose_base)

                return pose_base

        self.logger.warn(
            f"Tag {tag_id} not in results "
            f"({len(detection_result.markers)} markers detected)"
        )
        return None

    def detect_and_transform_circle(
        self,
        timeout: float = 45.0
    ) -> Optional[PoseStamped]:
        """Detect circular objects and transform to base_link frame.

        Uses Hough circle detection to find circular objects (wafers, etc.)
        and returns the 3D pose transformed to base_link.

        Args:
            timeout: Detection timeout in seconds

        Returns:
            PoseStamped in base_link frame, or None if detection failed
        """
        self.logger.info("Detecting circles...")

        # Use camera module for circle detection
        detected_poses = self._camera.detect_circles(
            self.rclpy_node,
            timeout=timeout
        )

        if not detected_poses:
            self.logger.warn("No circles detected")
            return None

        # Take the first (strongest) detection
        circle_pose = detected_poses[0]

        self.logger.info(
            f"Circle detected at [{circle_pose.position.x:.3f}, "
            f"{circle_pose.position.y:.3f}, {circle_pose.position.z:.3f}] "
            f"in camera frame"
        )

        # Transform to base_link
        pose_base = self._transform_to_base_link(circle_pose)
        if pose_base is None:
            return None

        self.logger.info(
            f"Transformed to base_link: [{pose_base.pose.position.x:.3f}, "
            f"{pose_base.pose.position.y:.3f}, {pose_base.pose.position.z:.3f}]"
        )

        # Broadcast TF frame for RViz visualization
        if self._publish_marker_frames:
            self._broadcast_circle_tf(pose_base)

        return pose_base

    def _broadcast_circle_tf(self, pose: PoseStamped):
        """Broadcast detected circle pose as TF frame for RViz.

        Args:
            pose: Pose in base_link frame
        """
        tf = TransformStamped()
        tf.header.stamp = self.rclpy_node.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "detected_circle"
        tf.transform.translation.x = pose.pose.position.x
        tf.transform.translation.y = pose.pose.position.y
        tf.transform.translation.z = pose.pose.position.z
        tf.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf)

    def detect_and_transform_contour(
        self,
        sample_index: int = 1,
        timeout: float = 45.0
    ) -> Optional[PoseStamped]:
        """Detect objects using contour detection and transform to base_link frame.

        Uses edge detection + contour finding to detect ANY shaped object
        (circles, squares, triangles, irregular shapes). Filters by area.

        Objects are sorted in reading order (left-to-right, top-to-bottom),
        so sample_index=1 is the top-left object.

        Args:
            sample_index: 1-indexed sample number to select (default: 1 = first/top-left)
            timeout: Detection timeout in seconds

        Returns:
            PoseStamped in base_link frame, or None if detection failed
        """
        self.logger.info(f"Detecting contours (any shape), selecting sample #{sample_index}...")

        # Use camera module for contour detection
        detected_poses = self._camera.detect_contours(
            self.rclpy_node,
            timeout=timeout
        )

        if not detected_poses:
            self.logger.warn("No contours detected matching area criteria")
            return None

        self.logger.info(f"Found {len(detected_poses)} sample(s)")

        # Validate sample_index (1-indexed)
        if sample_index < 1 or sample_index > len(detected_poses):
            self.logger.error(
                f"Invalid sample_index={sample_index}, detected {len(detected_poses)} samples "
                f"(valid range: 1-{len(detected_poses)})"
            )
            return None

        # Select the requested sample (convert to 0-indexed)
        contour_pose = detected_poses[sample_index - 1]

        self.logger.info(
            f"Sample #{sample_index} at [{contour_pose.position.x:.3f}, "
            f"{contour_pose.position.y:.3f}, {contour_pose.position.z:.3f}] "
            f"in camera frame"
        )

        # Transform to base_link
        pose_base = self._transform_to_base_link(contour_pose)
        if pose_base is None:
            return None

        self.logger.info(
            f"Transformed to base_link: [{pose_base.pose.position.x:.3f}, "
            f"{pose_base.pose.position.y:.3f}, {pose_base.pose.position.z:.3f}]"
        )

        # Broadcast TF frame for RViz visualization
        if self._publish_marker_frames:
            self._broadcast_detected_object_tf(pose_base, f"detected_sample_{sample_index}")

        return pose_base

    def _move_to_joint_pose(
        self,
        joint_positions: List[float],
    ) -> bool:
        """Move to joint configuration using MTC with Pilz PTP.

        Used for multi-position scanning moves between scan positions.
        Goes through full MoveIt planning pipeline for collision checking.

        Args:
            joint_positions: 6-element list of joint angles in radians

        Returns:
            True if move succeeded, False otherwise
        """
        if len(joint_positions) != 6:
            self.logger.error(
                f"Invalid joint positions: expected 6, got {len(joint_positions)}"
            )
            return False

        self.logger.debug(
            f"Moving to joint pose: [{', '.join(f'{j:.3f}' for j in joint_positions)}]"
        )

        task = self.create_task_template("Scan Position Move")
        planner = self.make_pilz_planner("PTP")

        stage = stages.MoveTo("move to scan position", planner)
        stage.group = self.arm_group
        self._set_ik_frame(stage)
        joint_dict = dict(zip(DEFAULT_JOINT_NAMES, joint_positions))
        stage.setGoal(joint_dict)
        task.add(stage)

        error = self.load_plan_execute(task)
        if error:
            self.logger.error(f"Scan position move failed: {error}")
            return False
        return True

    def _average_poses(self, poses: List[PoseStamped]) -> Optional[PoseStamped]:
        """Average multiple poses (position + quaternion orientation).

        For positions: simple arithmetic mean.
        For quaternions: align to same hemisphere, average, normalize.

        Args:
            poses: List of PoseStamped to average (must have ≥2 poses)

        Returns:
            Averaged PoseStamped, or None if insufficient poses
        """
        if len(poses) < 2:
            self.logger.error(f"Need ≥2 poses to average, got {len(poses)}")
            return None

        # Average positions
        avg_x = sum(p.pose.position.x for p in poses) / len(poses)
        avg_y = sum(p.pose.position.y for p in poses) / len(poses)
        avg_z = sum(p.pose.position.z for p in poses) / len(poses)

        # Average quaternions (with hemisphere alignment)
        # Quaternions q and -q represent the same rotation, so we align all
        # quaternions to the same hemisphere as the first one before averaging
        quats = []
        ref_q = np.array([
            poses[0].pose.orientation.x,
            poses[0].pose.orientation.y,
            poses[0].pose.orientation.z,
            poses[0].pose.orientation.w
        ])

        for p in poses:
            q = np.array([
                p.pose.orientation.x,
                p.pose.orientation.y,
                p.pose.orientation.z,
                p.pose.orientation.w
            ])
            # Flip if in opposite hemisphere (dot product < 0)
            if np.dot(ref_q, q) < 0:
                q = -q
            quats.append(q)

        # Average and normalize
        avg_q = np.mean(quats, axis=0)
        avg_q = avg_q / np.linalg.norm(avg_q)

        # Build result
        result = PoseStamped()
        result.header = poses[0].header
        result.pose.position.x = avg_x
        result.pose.position.y = avg_y
        result.pose.position.z = avg_z
        result.pose.orientation.x = avg_q[0]
        result.pose.orientation.y = avg_q[1]
        result.pose.orientation.z = avg_q[2]
        result.pose.orientation.w = avg_q[3]

        return result

    def detect_tag_multiposition(
        self,
        tag_id: int,
        scan_positions: List[List[float]],
        timeout: float = 45.0,
        settle_time: float = 0.3
    ) -> Optional[PoseStamped]:
        """Detect tag from multiple positions and average results.

        Moves robot to each scan position, captures and detects the marker,
        then averages all successful detections. This reduces systematic
        bias from any single viewing angle.

        Args:
            tag_id: ArUco marker ID to detect
            scan_positions: List of joint configurations (6 joints each, radians)
            timeout: Detection timeout per position in seconds
            settle_time: Seconds to wait after each move for robot vibration damping

        Returns:
            Averaged PoseStamped in base_link frame, or None if <2 detections
        """
        detected_poses = []
        position_results = []  # Track success/fail per position for logging

        self.logger.info(
            f"Multi-position detection: tag {tag_id} from {len(scan_positions)} positions"
        )

        for i, joint_pose in enumerate(scan_positions):
            position_name = f"position {i+1}/{len(scan_positions)}"

            # Move to scan position
            self.logger.info(f"Moving to {position_name}...")
            if not self._move_to_joint_pose(joint_pose):
                self.logger.warn(f"Failed to reach {position_name}, skipping")
                position_results.append((i+1, False, "move_failed"))
                continue

            # Wait for robot to settle (vibration damping)
            if settle_time > 0:
                self.logger.debug(f"Settling for {settle_time:.2f}s...")
                time.sleep(settle_time)

            # Detect tag at this position
            pose = self.detect_and_transform_tag(tag_id, timeout)

            if pose is not None:
                detected_poses.append(pose)
                pos = pose.pose.position
                self.logger.info(
                    f"  {position_name}: detected at "
                    f"[{pos.x*1000:.2f}, {pos.y*1000:.2f}, {pos.z*1000:.2f}] mm"
                )
                position_results.append((i+1, True, None))
            else:
                self.logger.warn(f"  {position_name}: detection failed")
                position_results.append((i+1, False, "detection_failed"))

        # Log summary
        success_count = len(detected_poses)
        self.logger.info(
            f"Multi-position results: {success_count}/{len(scan_positions)} successful"
        )

        if success_count < 2:
            self.logger.error(
                f"Need ≥2 successful detections for averaging, got {success_count}"
            )
            return None

        # Average all successful detections
        averaged_pose = self._average_poses(detected_poses)

        if averaged_pose is not None:
            pos = averaged_pose.pose.position
            self.logger.info(
                f"Averaged pose: [{pos.x*1000:.2f}, {pos.y*1000:.2f}, {pos.z*1000:.2f}] mm"
            )

            # Calculate and log standard deviation for variance analysis
            x_vals = [p.pose.position.x for p in detected_poses]
            y_vals = [p.pose.position.y for p in detected_poses]
            z_vals = [p.pose.position.z for p in detected_poses]
            self.logger.info(
                f"Spread (σ): X={np.std(x_vals)*1000:.3f}mm, "
                f"Y={np.std(y_vals)*1000:.3f}mm, Z={np.std(z_vals)*1000:.3f}mm"
            )

        return averaged_pose

    def _broadcast_detected_object_tf(self, pose: PoseStamped, frame_name: str):
        """Broadcast detected object pose as TF frame for RViz.

        Args:
            pose: Pose in base_link frame
            frame_name: Name for the TF frame (e.g., "detected_contour")
        """
        tf = TransformStamped()
        tf.header.stamp = self.rclpy_node.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = frame_name
        tf.transform.translation.x = pose.pose.position.x
        tf.transform.translation.y = pose.pose.position.y
        tf.transform.translation.z = pose.pose.position.z
        tf.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf)

    def _transform_to_base_link(
        self,
        pose_camera: Pose,
        capture_stamp=None
    ) -> Optional[PoseStamped]:
        """Transform a pose from camera frame to base_link.

        IMPORTANT: Uses capture_stamp for TF lookup to ensure the transform
        matches the robot pose when the image was captured, not when this
        function is called. This fixes the ~3mm bimodal variation issue.

        Args:
            pose_camera: Pose in camera optical frame
            capture_stamp: Timestamp when image was captured (for TF lookup).
                          If None, falls back to latest transform (legacy behavior).

        Returns:
            PoseStamped in base_link frame, or None on failure
        """
        try:
            # Determine which timestamp to use for TF lookup
            if capture_stamp is not None:
                # Convert builtin_interfaces.msg.Time to rclpy.time.Time
                lookup_time = rclpy.time.Time.from_msg(capture_stamp)
                self.logger.debug(
                    f"Using capture timestamp for TF lookup: "
                    f"{capture_stamp.sec}.{capture_stamp.nanosec:09d}"
                )
            else:
                # Fallback to latest available transform (legacy behavior)
                lookup_time = rclpy.time.Time()
                self.logger.debug("Using latest TF (no capture timestamp provided)")

            # Check if transform is available at the requested time
            if not self._tf_buffer.can_transform(
                "base_link",
                self._camera_frame,
                lookup_time,
                timeout=rclpy.duration.Duration(seconds=2.0)
            ):
                self.logger.error(
                    f"TF {self._camera_frame} -> base_link not available "
                    f"at requested time"
                )
                return None

            # Create stamped pose in camera frame
            pose_in = PoseStamped()
            pose_in.header.frame_id = self._camera_frame
            if capture_stamp is not None:
                pose_in.header.stamp = capture_stamp
            else:
                pose_in.header.stamp = self.rclpy_node.get_clock().now().to_msg()
            pose_in.pose = pose_camera

            # Transform using the capture timestamp
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                self._camera_frame,
                lookup_time,
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            pose_out = do_transform_pose_stamped(pose_in, transform)
            pose_out.header.frame_id = "base_link"

            return pose_out

        except TransformException as e:
            self.logger.error(f"TF failed: {e}")
            return None

    def _broadcast_marker_tf(self, marker_id: int, pose: PoseStamped):
        """Broadcast marker pose as TF frame for RViz visualization.

        Args:
            marker_id: Marker ID (used in frame name)
            pose: Pose in base_link frame
        """
        tf = TransformStamped()
        tf.header.stamp = self.rclpy_node.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = f"aruco_{marker_id}"
        tf.transform.translation.x = pose.pose.position.x
        tf.transform.translation.y = pose.pose.position.y
        tf.transform.translation.z = pose.pose.position.z
        tf.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf)

    # Default marker-frame offset (meters). Used when no marker_offset_x/y/z
    # are provided in the goal. Set to 0 — all offsets should come from config.
    SAMPLE_OFFSET_X = 0.0
    SAMPLE_OFFSET_Y = 0.0

    # Default z_offsets per gripper tip frame
    _Z_OFFSETS = {
        "epick_tip": 0.0,
        "robotiq_hande_end": -0.02,
        "pipette_tip_link": 0.0,
        "flange": 0.0,
    }

    @classmethod
    def _z_offset_for_frame(cls, ik_frame: str) -> float:
        """Return default z_offset for a given IK frame name."""
        return cls._Z_OFFSETS.get(ik_frame, 0.0)

    def _compute_approach_pose(
        self,
        target: PoseStamped,
        z_offset_override: float = 0.0,
        marker_offset_x: float = 0.0,
        marker_offset_y: float = 0.0,
        marker_offset_z: float = 0.0,
        ik_frame_override: str = "",
    ) -> Tuple[PoseStamped, str]:
        """Compute the final approach pose from a detected target.

        Applies marker-frame XYZ offset, marker-aligned yaw, z_offset, and
        gripper tip frame detection — everything needed to turn a raw detection
        into an actual robot goal.

        Args:
            target: Raw detected pose in base_link
            z_offset_override: Override z_offset (0 = use gripper default)
            marker_offset_x: Offset in marker local X (meters). 0 = use class default.
            marker_offset_y: Offset in marker local Y (meters). 0 = use class default.
            marker_offset_z: Offset in marker local Z (meters). 0 = none.
            ik_frame_override: Explicit IK frame from orchestrator (e.g. "pipette_tip_link").
                Empty = auto-detect from TF. Bypasses TF-based detection which can
                return stale frames after tool exchange.

        Returns:
            Tuple of (approach PoseStamped, active_ik_frame)
        """
        # Determine IK frame: explicit override > constructor arg > auto-detect
        if ik_frame_override:
            active_ik_frame = ik_frame_override
            active_z_offset = self._z_offset_for_frame(active_ik_frame)
            self.logger.info(
                f"Using orchestrator IK frame: {active_ik_frame} "
                f"(z_offset: {active_z_offset:.3f})"
            )
        elif self.ik_frame and self.ik_frame != "flange":
            active_ik_frame = self.ik_frame
            active_z_offset = self._z_offset_for_frame(active_ik_frame)
        else:
            detection = self._detect_current_gripper()
            active_ik_frame = detection.ik_frame
            active_z_offset = detection.z_offset
            self.logger.info(
                f"Auto-detected: {active_ik_frame} (z_offset: {active_z_offset:.3f})"
            )

        # Use override if provided (non-zero)
        if z_offset_override != 0.0:
            self.logger.info(f"Using z_offset override: {z_offset_override:.3f}")
            active_z_offset = z_offset_override

        # Determine marker-frame offset: use provided values if any are non-zero,
        # otherwise fall back to class defaults (SAMPLE_OFFSET_X/Y)
        if marker_offset_x != 0.0 or marker_offset_y != 0.0 or marker_offset_z != 0.0:
            offset_x = marker_offset_x
            offset_y = marker_offset_y
            offset_z = marker_offset_z
        else:
            offset_x = self.SAMPLE_OFFSET_X
            offset_y = self.SAMPLE_OFFSET_Y
            offset_z = 0.0

        # Apply marker-frame offset → rotate to base_link world frame
        if offset_x != 0.0 or offset_y != 0.0 or offset_z != 0.0:
            q = [
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w
            ]
            rot_matrix = quaternion_matrix(q)[:3, :3]
            local_offset = np.array([offset_x, offset_y, offset_z])
            world_offset = rot_matrix @ local_offset
            self.logger.info(
                f"Marker offset: [{offset_x:.3f}, {offset_y:.3f}, {offset_z:.3f}] → "
                f"world [{world_offset[0]:.3f}, {world_offset[1]:.3f}, {world_offset[2]:.3f}]"
            )
        else:
            world_offset = np.array([0.0, 0.0, 0.0])

        # Compute approach pose: marker offset + z_offset + orientation
        approach = PoseStamped()
        approach.header = target.header
        approach.pose.position.x = target.pose.position.x + world_offset[0]
        approach.pose.position.y = target.pose.position.y + world_offset[1]
        approach.pose.position.z = target.pose.position.z + world_offset[2] + active_z_offset

        # Straight-down orientation with marker-aligned yaw
        # 1. Compute yaw from the original marker orientation + 180° Z rotation
        #    (this produces a yaw consistent with the robot's wrist angle)
        # 2. Combine with fixed roll=180°, pitch=0 for stable flat approach
        #    (avoids bimodal Z errors from using marker roll/pitch directly)
        q_marker = [
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w
        ]
        q_z180 = quaternion_from_euler(0, 0, math.pi)
        q_rotated = quaternion_multiply(q_marker, q_z180)
        _, _, approach_yaw = euler_from_quaternion(q_rotated)
        q_approach = quaternion_from_euler(math.pi, 0, approach_yaw)
        approach.pose.orientation.x = q_approach[0]
        approach.pose.orientation.y = q_approach[1]
        approach.pose.orientation.z = q_approach[2]
        approach.pose.orientation.w = q_approach[3]

        self.logger.info(
            f"Approach pose: [{approach.pose.position.x:.3f}, "
            f"{approach.pose.position.y:.3f}, {approach.pose.position.z:.3f}] "
            f"yaw={math.degrees(approach_yaw):.1f}°, z_offset={active_z_offset:.3f}"
        )

        return approach, active_ik_frame

    def _apply_flange_offset(
        self,
        pose: PoseStamped,
        direction: str,
        distance: float
    ) -> PoseStamped:
        """Apply a directional offset in the flange frame to a base_link pose.

        Looks up the current flange TF to get the flange-to-base rotation,
        then transforms the direction vector from flange frame to base_link
        and applies it as a position offset.

        Args:
            pose: Input pose in base_link frame (modified in place)
            direction: Direction name from DIRECTION_VECTORS (e.g. "right")
            distance: Offset distance in meters

        Returns:
            The pose with offset applied
        """
        if direction not in DIRECTION_VECTORS:
            self.logger.warn(
                f"Unknown offset direction '{direction}', skipping. "
                f"Valid: {list(DIRECTION_VECTORS.keys())}"
            )
            return pose

        # Get the flange orientation in base_link from TF
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link", "flange",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
        except TransformException as e:
            self.logger.error(f"Failed to look up flange TF for offset: {e}")
            return pose

        # Build rotation matrix from flange quaternion
        q = [
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w
        ]
        rot = quaternion_matrix(q)[:3, :3]

        # Transform direction vector from flange frame to base_link
        flange_vec = np.array(DIRECTION_VECTORS[direction])
        base_vec = rot @ flange_vec

        # Apply offset
        pose.pose.position.x += base_vec[0] * distance
        pose.pose.position.y += base_vec[1] * distance
        pose.pose.position.z += base_vec[2] * distance

        self.logger.info(
            f"Applied offset: {distance*1000:.1f}mm {direction} "
            f"(flange {flange_vec} → base [{base_vec[0]:.3f}, {base_vec[1]:.3f}, {base_vec[2]:.3f}])"
        )

        return pose

    def _move_to_approach(
        self, approach: PoseStamped, ik_frame: str = ""
    ) -> 'Optional[str]':
        """Execute MoveIt move to a pre-computed approach pose.

        Args:
            approach: Fully computed approach pose in base_link (with offsets,
                      orientation, and z_offset already applied)
            ik_frame: IK frame to use (e.g. "epick_tip"). If empty,
                      auto-detects from TF.

        Returns:
            None if successful, error string describing failure otherwise
        """
        if not ik_frame:
            detection = self._detect_current_gripper()
            ik_frame = detection.ik_frame

        # Use MTC with Pilz LIN for collision-free straight-line approach
        task = self.create_task_template("Vision Move")
        planner = self.make_pilz_planner("LIN")

        stage = stages.MoveTo("move to tag", planner)
        stage.group = self.arm_group
        ik_frame_pose = PoseStamped()
        ik_frame_pose.header.frame_id = ik_frame
        stage.ik_frame = ik_frame_pose
        stage.setGoal(approach)
        task.add(stage)

        return self.load_plan_execute(task)

    def _move_to_pose(
        self,
        target: PoseStamped,
        z_offset_override: float = 0.0
    ) -> 'Optional[str]':
        """Move robot to the target pose with orientation adjustment.

        Applies sample XY offset, 180° Z rotation, and z_offset for proper approach.
        Used by vision_pick_place_stages.py.

        Args:
            target: Raw detected pose in base_link
            z_offset_override: Override z_offset (0 = use gripper default)

        Returns:
            None if successful, error string describing failure otherwise
        """
        approach, active_ik_frame = self._compute_approach_pose(target, z_offset_override)
        return self._move_to_approach(approach, ik_frame=active_ik_frame)

    def _detect_current_gripper(self) -> GripperDetection:
        """Auto-detect the current gripper by checking TF frames.

        Returns:
            GripperDetection with ik_frame and z_offset
        """
        # Check for ePick gripper
        if self._tf_buffer.can_transform(
            "base", "epick_tip",
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        ):
            return GripperDetection("epick_tip", 0.003)

        # Check for Hand-E gripper
        if self._tf_buffer.can_transform(
            "base", "robotiq_hande_end",
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        ):
            return GripperDetection("robotiq_hande_end", -0.02)

        # Check for Pipettor
        if self._tf_buffer.can_transform(
            "base", "pipette_tip_link",
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        ):
            return GripperDetection("pipette_tip_link", 0.0)

        # Default to flange
        self.logger.info("No gripper detected, using flange")
        return GripperDetection("flange", 0.0)

    def _add_collision_object_for_tag(self, tag_id: int, tag_pose: PoseStamped):
        """Add a collision object for a detected tag.

        Args:
            tag_id: Marker ID
            tag_pose: Pose of the marker
        """
        if tag_id not in self._object_database:
            return

        info = self._object_database[tag_id]

        # Remove existing object with same name
        self._remove_collision_object(info.name)

        # Calculate object pose with offset
        object_pose = self._calculate_object_pose(tag_pose, info.tag_offset)

        # Create collision object
        obj = CollisionObject()
        obj.header.frame_id = object_pose.header.frame_id
        obj.header.stamp = self.rclpy_node.get_clock().now().to_msg()
        obj.id = info.name
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        if info.shape == "box" and len(info.dimensions) == 3:
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = info.dimensions
        elif info.shape == "cylinder" and len(info.dimensions) == 2:
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = info.dimensions
        else:
            self.logger.error(f"Invalid shape: {info.shape}")
            return

        obj.primitives.append(primitive)
        obj.primitive_poses.append(object_pose.pose)

        # Publish to planning scene as diff
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(obj)
        self._planning_scene_pub.publish(scene_msg)
        self.logger.info(f"Added collision object '{info.name}'")

    def _calculate_object_pose(
        self,
        tag_pose: PoseStamped,
        offset: list
    ) -> PoseStamped:
        """Calculate object pose from tag pose with offset.

        Args:
            tag_pose: Pose of the marker
            offset: [x, y, z] offset in marker's local frame

        Returns:
            Object pose in world frame
        """
        # Get rotation matrix from quaternion
        q = [
            tag_pose.pose.orientation.x,
            tag_pose.pose.orientation.y,
            tag_pose.pose.orientation.z,
            tag_pose.pose.orientation.w
        ]
        rot_matrix = quaternion_matrix(q)[:3, :3]

        # Transform offset from local to world frame
        offset_local = np.array(offset)
        offset_world = rot_matrix @ offset_local

        result = PoseStamped()
        result.header = tag_pose.header
        result.pose.position.x = tag_pose.pose.position.x + offset_world[0]
        result.pose.position.y = tag_pose.pose.position.y + offset_world[1]
        result.pose.position.z = tag_pose.pose.position.z + offset_world[2]
        result.pose.orientation = tag_pose.pose.orientation

        return result

    def _remove_collision_object(self, name: str):
        """Remove a collision object if it exists.

        Args:
            name: Object ID to remove
        """
        # Query current planning scene to check if object exists
        if not self._get_scene_client.wait_for_service(timeout_sec=1.0):
            self.logger.warn("GetPlanningScene service not available, skipping removal check")
            # Publish removal anyway - it's safe if object doesn't exist
        else:
            request = GetPlanningScene.Request()
            request.components.components = (
                request.components.WORLD_OBJECT_NAMES
            )
            future = self._get_scene_client.call_async(request)
            rclpy.spin_until_future_complete(self.rclpy_node, future, timeout_sec=2.0)

            if future.done() and future.result() is not None:
                known_names = [
                    obj.id for obj in future.result().scene.world.collision_objects
                ]
                if name not in known_names:
                    return  # Object doesn't exist, nothing to remove

        # Publish removal
        obj = CollisionObject()
        obj.id = name
        obj.operation = CollisionObject.REMOVE

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(obj)
        self._planning_scene_pub.publish(scene_msg)
