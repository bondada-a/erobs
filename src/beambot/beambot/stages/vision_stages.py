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
from tf_transformations import quaternion_multiply, quaternion_from_euler, quaternion_matrix

from beambot.camera import get_camera
from beambot.stages.base_stages import BaseStages


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

    def __init__(
        self,
        rclpy_node,
        arm_group: str = "",
        ik_frame: str = "",
        camera_type: str = None,
        camera_frame: str = None,
        marker_dictionary: str = None,
        retry_count: int = None,
        retry_delay: float = None
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
        """
        super().__init__(rclpy_node, arm_group, ik_frame=ik_frame)

        # Camera configuration
        self._camera_type = camera_type if camera_type else self.DEFAULT_CAMERA_TYPE
        self._camera_frame = camera_frame if camera_frame else self.DEFAULT_CAMERA_FRAME
        self._marker_dictionary = marker_dictionary if marker_dictionary else self.DEFAULT_MARKER_DICTIONARY

        # Retry configuration
        self._retry_count = retry_count if retry_count is not None else self.DEFAULT_RETRY_COUNT
        self._retry_delay = retry_delay if retry_delay is not None else self.DEFAULT_RETRY_DELAY

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

        self.logger.info(
            f"VisionStages initialized (camera: {self._camera_type}, "
            f"frame: {self._camera_frame}, ik_frame: "
            f"{'auto-detect' if not ik_frame else ik_frame})"
        )

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

    def run(self, goal) -> bool:
        """Execute VisionMoveTo action.

        Args:
            goal: VisionMoveToAction.Goal with:
                - tag_id: Marker ID (for marker detection)
                - timeout: Detection timeout
                - detection_type: "marker" (default) or "circle"
                - z_offset: Override z_offset (0 = use gripper default)

        Returns:
            True if successful, False otherwise
        """
        # Determine detection type (default to marker for backwards compatibility)
        detection_type = getattr(goal, 'detection_type', '') or self.DETECTION_MARKER

        # Get z_offset override (0 or missing means use gripper default)
        z_offset_override = getattr(goal, 'z_offset', 0.0)

        # Route to appropriate detection method
        if detection_type == self.DETECTION_CIRCLE:
            self.logger.info("Using circle detection")
            target_pose = self.detect_and_transform_circle(goal.timeout)
            if target_pose is None:
                self.logger.error("Failed to detect circle/wafer")
                return False
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
                self.logger.error(f"Failed to detect sample #{sample_index} via contour")
                return False
        else:
            # Default: marker detection
            target_pose = self.detect_and_transform_tag(goal.tag_id, goal.timeout)
            if target_pose is None:
                self.logger.error(f"Failed to detect tag {goal.tag_id}")
                return False

        return self._move_to_pose(target_pose, z_offset_override=z_offset_override)

    def detect_and_transform_tag(
        self,
        tag_id: int,
        timeout: float = 10.0
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
    ) -> Optional[List[Tuple[int, Pose]]]:
        """Execute a single detection attempt using camera module.

        Args:
            tag_id: ArUco marker ID to detect
            timeout: Detection timeout in seconds

        Returns:
            List of (marker_id, pose) tuples if successful, None on error
        """
        self.logger.info(f"Detecting tag {tag_id}...")

        # Use camera module for detection
        detected = self._camera.detect_markers(
            self._capture_client,
            self.rclpy_node,
            marker_ids=[tag_id],
            dictionary=self._marker_dictionary,
            timeout=timeout
        )

        if not detected:
            self.logger.warn("No markers detected")
            return None

        return detected

    def _process_detection_result(
        self,
        tag_id: int,
        detected_markers: List[Tuple[int, Pose]]
    ) -> Optional[PoseStamped]:
        """Process detection result and transform to base_link.

        Args:
            tag_id: ArUco marker ID to find
            detected_markers: List of (marker_id, pose) tuples from camera

        Returns:
            PoseStamped in base_link frame, or None if tag not found
        """
        # Find the requested marker in results
        for marker_id, marker_pose in detected_markers:
            if marker_id == tag_id:
                self.logger.info(
                    f"Tag {marker_id} at [{marker_pose.position.x:.3f}, "
                    f"{marker_pose.position.y:.3f}, {marker_pose.position.z:.3f}] "
                    f"in camera frame"
                )

                # Transform to base_link
                pose_base = self._transform_to_base_link(marker_pose)
                if pose_base is None:
                    return None

                self.logger.info(
                    f"Transformed to base_link: [{pose_base.pose.position.x:.3f}, "
                    f"{pose_base.pose.position.y:.3f}, {pose_base.pose.position.z:.3f}]"
                )

                # Optional: broadcast TF frame for RViz
                if self._publish_marker_frames:
                    self._broadcast_marker_tf(tag_id, pose_base)

                # Add collision object if configured
                self._add_collision_object_for_tag(tag_id, pose_base)

                return pose_base

        self.logger.warn(
            f"Tag {tag_id} not in results "
            f"({len(detected_markers)} markers detected)"
        )
        return None

    def detect_and_transform_circle(
        self,
        timeout: float = 10.0
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
        timeout: float = 10.0
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

    def _transform_to_base_link(self, pose_camera: Pose) -> Optional[PoseStamped]:
        """Transform a pose from camera frame to base_link.

        Args:
            pose_camera: Pose in camera optical frame

        Returns:
            PoseStamped in base_link frame, or None on failure
        """
        try:
            # Check if transform is available (use Time() for latest available, not now())
            if not self._tf_buffer.can_transform(
                "base_link",
                self._camera_frame,
                rclpy.time.Time(),  # Latest available transform
                timeout=rclpy.duration.Duration(seconds=1.0)
            ):
                self.logger.error(
                    f"TF {self._camera_frame} -> base_link not available"
                )
                return None

            # Create stamped pose in camera frame
            pose_in = PoseStamped()
            pose_in.header.frame_id = self._camera_frame
            pose_in.header.stamp = self.rclpy_node.get_clock().now().to_msg()
            pose_in.pose = pose_camera

            # Transform
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                self._camera_frame,
                rclpy.time.Time()
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

    # Sample offset from tag center in tag's local frame (meters)
    # Set to (0, 0) if sample is directly on the tag
    SAMPLE_OFFSET_X = 0.02   # X offset in tag frame (20mm to the right)
    SAMPLE_OFFSET_Y = 0.0    # Y offset in tag frame

    def _move_to_pose(
        self,
        target: PoseStamped,
        z_offset_override: float = 0.0
    ) -> bool:
        """Move robot to the target pose with orientation adjustment.

        Applies sample XY offset, 180° Z rotation, and z_offset for proper approach.

        Args:
            target: Target pose in base_link
            z_offset_override: Override z_offset (0 = use gripper default)

        Returns:
            True if successful, False otherwise
        """
        # Auto-detect gripper if ik_frame not set
        if not self.ik_frame or self.ik_frame == "flange":
            detection = self._detect_current_gripper()
            active_ik_frame = detection.ik_frame
            active_z_offset = detection.z_offset
            self.logger.info(
                f"Auto-detected: {active_ik_frame} (z_offset: {active_z_offset:.3f})"
            )
        else:
            active_ik_frame = self.ik_frame
            # Default z_offset based on gripper type
            if "epick" in active_ik_frame:
                active_z_offset = 0.1
            else:
                active_z_offset = -0.02

        # Use override if provided (non-zero)
        if z_offset_override != 0.0:
            self.logger.info(f"Using z_offset override: {z_offset_override:.3f}")
            active_z_offset = z_offset_override

        # Apply sample XY offset in tag's local frame
        if self.SAMPLE_OFFSET_X != 0.0 or self.SAMPLE_OFFSET_Y != 0.0:
            q = [
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w
            ]
            rot_matrix = quaternion_matrix(q)[:3, :3]
            local_offset = np.array([self.SAMPLE_OFFSET_X, self.SAMPLE_OFFSET_Y, 0.0])
            world_offset = rot_matrix @ local_offset
            self.logger.info(
                f"Sample offset: [{self.SAMPLE_OFFSET_X:.3f}, {self.SAMPLE_OFFSET_Y:.3f}] → "
                f"world [{world_offset[0]:.3f}, {world_offset[1]:.3f}]"
            )
        else:
            world_offset = np.array([0.0, 0.0, 0.0])

        # Compute approach pose: XY offset + 180° Z rotation + z_offset
        approach = PoseStamped()
        approach.header = target.header
        approach.pose.position.x = target.pose.position.x + world_offset[0]
        approach.pose.position.y = target.pose.position.y + world_offset[1]
        approach.pose.position.z = target.pose.position.z + active_z_offset

        # Apply 180° rotation around Z axis
        q_orig = [
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w
        ]
        q_rot = quaternion_from_euler(0, 0, math.pi)
        q_final = quaternion_multiply(q_orig, q_rot)

        approach.pose.orientation.x = q_final[0]
        approach.pose.orientation.y = q_final[1]
        approach.pose.orientation.z = q_final[2]
        approach.pose.orientation.w = q_final[3]

        self.logger.info(
            f"Moving to [{approach.pose.position.x:.3f}, "
            f"{approach.pose.position.y:.3f}, {approach.pose.position.z:.3f}] "
            f"with 180° Z-rot, z_offset={active_z_offset:.3f}"
        )

        # Build MTC task
        task = self.create_task_template("Vision Move")
        cartesian = self.make_cartesian_planner()

        stage = stages.MoveTo("move to tag", cartesian)
        stage.group = self.arm_group
        # Set ik_frame for this specific movement
        ik_frame_pose = PoseStamped()
        ik_frame_pose.header.frame_id = active_ik_frame
        stage.ik_frame = ik_frame_pose
        stage.setGoal(approach)
        task.add(stage)

        return self.load_plan_execute(task)

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
            return GripperDetection("epick_tip", 0.027) #pen suction cup
            # return GripperDetection("epick_tip", 0.022)

        # Check for Hand-E gripper
        if self._tf_buffer.can_transform(
            "base", "robotiq_hande_end",
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        ):
            return GripperDetection("robotiq_hande_end", -0.02)

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
