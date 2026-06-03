"""VisionStages - Python equivalent of vision_stages.cpp.

Handles vision-guided robot movement:
- ArUco marker detection via Zivid camera
- Circle/object detection via Hough Transform
- TF transforms from camera to base frame
- Collision object management
- Motion to detected poses
"""

import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from moveit.task_constructor import stages
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import GetPlanningScene, GetPositionIK
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from tf_transformations import quaternion_multiply, quaternion_from_euler, quaternion_matrix, euler_from_quaternion

from beambot.camera import DetectionResult, get_camera
from beambot.stages.base_stages import (
    BaseStages, DEFAULT_JOINT_NAMES, DIRECTION_VECTORS, wait_for_future,
)


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
    DETECTION_SAMPLE_ROI = "sample_roi"

    # Retry configuration defaults (not beamline-specific — tuning constants)
    DEFAULT_RETRY_COUNT = 3  # Number of retries after first attempt
    DEFAULT_RETRY_DELAY = 0.5  # Seconds between retries

    # Settle time: wait before capture for robot vibration to dampen
    DEFAULT_SETTLE_TIME = 0.3

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
            camera_type: Camera type from beamline config (REQUIRED)
            camera_frame: Camera TF frame (REQUIRED)
            marker_dictionary: ArUco dictionary (REQUIRED)
            retry_count: Number of detection retries (default: 3)
            retry_delay: Delay between retries in seconds (default: 0.5)
            settle_time: Seconds to wait before capture for robot to settle (default: 0.3)

        Raises:
            ValueError: if camera_type/camera_frame/marker_dictionary aren't supplied.
                Callers must source these from the active beamline YAML's
                `camera:` block — VisionStages refuses CMS-flavored fallbacks
                so a misconfiguration fails loudly at startup.
        """
        super().__init__(rclpy_node, arm_group, ik_frame=ik_frame)

        # Camera configuration — required, no fallback
        missing = [
            name for name, val in (
                ("camera_type", camera_type),
                ("camera_frame", camera_frame),
                ("marker_dictionary", marker_dictionary),
            ) if not val
        ]
        if missing:
            raise ValueError(
                f"VisionStages requires {', '.join(missing)} from the active "
                f"beamline YAML's `camera:` block (set BEAMBOT_BEAMLINE_CONFIG)."
            )
        self._camera_type = camera_type
        self._camera_frame = camera_frame
        self._marker_dictionary = marker_dictionary

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
        self._object_database: dict[int, ObjectInfo] = {}

        # Load vision objects config
        self._load_vision_objects_config()

        # Cache for multi-position scan results
        # Populated by scan_all_tags(), used by run() for fast lookup
        # {tag_id: PoseStamped} - averaged poses from vision_scan
        self._tag_pose_cache: dict[int, PoseStamped] = {}

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
        """Load tag-keyed collision-object metadata from the active beamline YAML.

        Reads `collision_objects:` from $BEAMBOT_BEAMLINE_CONFIG. When a vision
        detection sees one of these tags, the matching shape is added to the
        planning scene so MoveIt avoids it.

        Errors per-entry instead of all-or-nothing — a typo in one entry
        shouldn't silently disable collision avoidance for every object.
        """
        try:
            from beambot.config_loader import load_beamline_config
            config, _ = load_beamline_config()
        except Exception as e:
            self.logger.error(f"Failed to load beamline YAML for collision_objects: {e}")
            return

        objects = config.get("collision_objects") or {}
        for tag_id, obj in objects.items():
            try:
                info = ObjectInfo(
                    name=obj["name"],
                    shape=obj["shape"],
                    dimensions=obj["dimensions"],
                    tag_offset=obj["tag_offset"],
                )
                self._object_database[int(tag_id)] = info
            except (KeyError, TypeError, ValueError) as e:
                self.logger.error(
                    f"collision_objects[{tag_id}]: skipped — {e}. "
                    f"Required keys: name, shape, dimensions, tag_offset."
                )

        self.logger.info(f"Loaded {len(self._object_database)} vision objects")

    # =========================================================================
    # Tag Pose Cache Methods
    # =========================================================================

    def clear_cache(self):
        """Clear the tag pose cache."""
        count = len(self._tag_pose_cache)
        self._tag_pose_cache.clear()
        self.logger.info(f"Tag pose cache cleared ({count} entries)")

    def get_cached_pose(self, tag_id: int) -> PoseStamped | None:
        """Get cached pose for tag, or None if not cached."""
        return self._tag_pose_cache.get(tag_id)

    def scan_all_tags(
        self,
        scan_positions: list[list[float]],
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
        all_detections: dict[int, list[PoseStamped]] = {}

        total_scans = len(scan_positions) * scans_per_position
        self.logger.info(
            f"Starting batch scan: {len(scan_positions)} positions × "
            f"{scans_per_position} scans = {total_scans} total captures"
        )

        for pos_idx, joint_pose in enumerate(scan_positions):
            self.logger.info(f"Position {pos_idx+1}/{len(scan_positions)}")

            # Move to scan position
            if not self._move_to_joint_pose(joint_pose):
                self.logger.warning(f"Failed to reach position {pos_idx+1}, skipping")
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
                self.logger.warning(
                    f"  Tag {tag_id}: only {len(poses)} detection(s), need ≥2 for averaging"
                )

        self.logger.info(
            f"Batch scan complete: {len(self._tag_pose_cache)} tags cached"
        )
        return len(self._tag_pose_cache)

    def run(self, goal) -> 'str | None':
        """Execute VisionMoveTo action.

        Args:
            goal: VisionMoveToAction.Goal with:
                - tag_id: Marker ID (for marker detection)
                - timeout: Detection timeout
                - detection_type: "marker" (default) or "sample_roi"
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
            time.sleep(self._settle_time)
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
                self.logger.warning(
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
        if detection_type == self.DETECTION_SAMPLE_ROI:
            strategy = getattr(goal, 'strategy', '') or 'farthest_edge'
            edge_inset_mm = getattr(goal, 'edge_inset_mm', 0.0) or 6.5
            self.logger.info(
                f"Using sample_roi detection (tag {goal.tag_id}, "
                f"strategy={strategy}, inset={edge_inset_mm}mm)"
            )
            target_pose = self.detect_and_transform_sample_roi(
                tag_id=goal.tag_id,
                strategy=strategy,
                edge_inset_mm=edge_inset_mm,
                timeout=goal.timeout,
            )
            if target_pose is None:
                return (
                    f"DETECTION_FAILED: sample_roi detection failed for tag {goal.tag_id}"
                )
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
        approach, active_ik_frame = self.compute_approach_pose(
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
    ) -> PoseStamped | None:
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
    ) -> DetectionResult | None:
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
            self.logger.warning("No markers detected")
            return None

        return result

    def _process_detection_result(
        self,
        tag_id: int,
        detection_result: DetectionResult
    ) -> PoseStamped | None:
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
                    self._broadcast_detection_tf(f"aruco_{tag_id}", pose_base)

                # Add collision object if configured
                self._add_collision_object_for_tag(tag_id, pose_base)

                return pose_base

        self.logger.warning(
            f"Tag {tag_id} not in results "
            f"({len(detection_result.markers)} markers detected)"
        )
        return None

    def _broadcast_detection_tf(self, frame_name: str, pose: PoseStamped):
        """Broadcast a detected pose as a TF frame for RViz visualization.

        Args:
            frame_name: Name for the TF frame (e.g. "aruco_5", "detected_circle")
            pose: Pose in base_link frame
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

    def detect_and_transform_sample_roi(
        self,
        tag_id: int,
        strategy: str = "farthest_edge",
        edge_inset_mm: float = 6.5,
        timeout: float = 45.0,
    ) -> PoseStamped | None:
        """Detect a sample in an ROI anchored to an ArUco tag.

        Uses the tag's pixel corners to define an ROI, detects the sample
        contour within that ROI, and returns a 3D pickup pose in base_link.
        Includes retry logic for transient failures.

        Args:
            tag_id: ArUco marker ID that anchors the sample ROI
            strategy: Pickup strategy — "center", "farthest_edge", etc.
            edge_inset_mm: Distance inward from edge toward center (mm)
            timeout: Detection timeout in seconds (per attempt)

        Returns:
            PoseStamped in base_link frame, or None if detection failed
        """
        total_attempts = 1 + self._retry_count

        for attempt in range(total_attempts):
            if attempt > 0:
                self.logger.info(
                    f"Retry {attempt}/{self._retry_count} for sample_roi tag {tag_id} "
                    f"(waiting {self._retry_delay}s...)"
                )
                time.sleep(self._retry_delay)

            result = self._camera.detect_sample_roi(
                self.rclpy_node,
                tag_id=tag_id,
                strategy=strategy,
                edge_inset_mm=edge_inset_mm,
                dictionary=self._marker_dictionary,
                timeout=timeout,
            )

            if result is None:
                continue

            pickup_pose, capture_stamp = result

            self.logger.info(
                f"Sample ROI: pickup at [{pickup_pose.position.x:.4f}, "
                f"{pickup_pose.position.y:.4f}, {pickup_pose.position.z:.4f}] "
                f"in camera frame"
            )

            pose_base = self._transform_to_base_link(
                pickup_pose, capture_stamp=capture_stamp
            )
            if pose_base is None:
                continue

            self.logger.info(
                f"Transformed to base_link: [{pose_base.pose.position.x:.4f}, "
                f"{pose_base.pose.position.y:.4f}, {pose_base.pose.position.z:.4f}]"
            )

            if self._publish_marker_frames:
                self._broadcast_detection_tf(f"sample_roi_tag{tag_id}", pose_base)

            if attempt > 0:
                self.logger.info(f"sample_roi detection succeeded on retry {attempt}")
            return pose_base

        self.logger.error(
            f"Failed sample_roi detection for tag {tag_id} after {total_attempts} attempts"
        )
        return None

    def _move_to_joint_pose(
        self,
        joint_positions: list[float],
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

    def _average_poses(self, poses: list[PoseStamped]) -> PoseStamped | None:
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
        scan_positions: list[list[float]],
        timeout: float = 45.0,
        settle_time: float = 0.3
    ) -> PoseStamped | None:
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
                self.logger.warning(f"Failed to reach {position_name}, skipping")
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
                self.logger.warning(f"  {position_name}: detection failed")
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

    def _transform_to_base_link(
        self,
        pose_camera: Pose,
        capture_stamp=None
    ) -> PoseStamped | None:
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

    @staticmethod
    def _z_offset_for_frame(ik_frame: str) -> float:
        """Return default z_offset for a given IK frame name.

        Sourced from grippers.<name>.z_offset in the active beamline YAML
        (matched against tip_frame). The flange case (no gripper) returns 0.
        """
        from beambot.config_loader import z_offset_for_tip_frame
        return z_offset_for_tip_frame(ik_frame, default=0.0)

    def compute_approach_pose(
        self,
        target: PoseStamped,
        z_offset_override: float = 0.0,
        marker_offset_x: float = 0.0,
        marker_offset_y: float = 0.0,
        marker_offset_z: float = 0.0,
        ik_frame_override: str = "",
    ) -> tuple[PoseStamped, str]:
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

        # Apply marker-frame offset → rotate to base_link world frame
        if marker_offset_x != 0.0 or marker_offset_y != 0.0 or marker_offset_z != 0.0:
            q = [
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w
            ]
            rot_matrix = quaternion_matrix(q)[:3, :3]
            local_offset = np.array([marker_offset_x, marker_offset_y, marker_offset_z])
            world_offset = rot_matrix @ local_offset
            self.logger.info(
                f"Marker offset: [{marker_offset_x:.3f}, {marker_offset_y:.3f}, {marker_offset_z:.3f}] → "
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
            f"Approach pose: [{approach.pose.position.x:.6f}, "
            f"{approach.pose.position.y:.6f}, {approach.pose.position.z:.6f}] "
            f"yaw={math.degrees(approach_yaw):.4f}°, z_offset={active_z_offset:.3f}"
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
            self.logger.warning(
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

    # compute_deterministic_ik() is inherited from BaseStages (#55)

    def _move_to_approach(
        self, approach: PoseStamped, ik_frame: str = ""
    ) -> 'str | None':
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

        # Pre-compute IK for deterministic joint goal.
        # KDL's IK has non-deterministic random re-seeding that causes ~1mm
        # bimodal jitter when called through Pilz's trajectory generation.
        # By computing IK once via /compute_ik (which is deterministic for a
        # given seed) and sending the result as a joint goal, we bypass the
        # non-determinism entirely. (#51)
        joint_goal = self.compute_deterministic_ik(approach, ik_frame)
        if joint_goal is None:
            self.logger.warning("Deterministic IK failed, falling back to Cartesian goal")
            # Fallback: use original Cartesian goal (may have ~1mm jitter)
            task = self.create_task_template("Vision Move")
            planner = self.make_pilz_planner("LIN")
            stage = stages.MoveTo("move to tag", planner)
            stage.group = self.arm_group
            ik_frame_pose = PoseStamped()
            ik_frame_pose.header.frame_id = ik_frame
            stage.ik_frame = ik_frame_pose
            stage.setGoal(approach)
            task.add(stage)
        else:
            task = self.create_task_template("Vision Move")
            planner = self.make_pilz_planner("PTP")
            stage = stages.MoveTo("move to tag", planner)
            stage.group = self.arm_group
            self._set_ik_frame(stage)
            stage.setGoal(joint_goal)
            task.add(stage)

        return self.load_plan_execute(task)

    def _detect_current_gripper(self) -> GripperDetection:
        """Auto-detect the current gripper by checking TF frames.

        Returns:
            GripperDetection with ik_frame and z_offset
        """
        # Check for known gripper tip frames in TF (sourced from YAML)
        from beambot.config_loader import configured_tip_frames
        for frame in configured_tip_frames():
            if self._tf_buffer.can_transform(
                "base_link", frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            ):
                return GripperDetection(frame, self._z_offset_for_frame(frame))

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
            self.logger.warning("GetPlanningScene service not available, skipping removal check")
            # Publish removal anyway - it's safe if object doesn't exist
        else:
            request = GetPlanningScene.Request()
            request.components.components = (
                request.components.WORLD_OBJECT_NAMES
            )
            future = self._get_scene_client.call_async(request)
            wait_for_future(future, timeout=2.0)

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
