"""VisionStages - Python equivalent of vision_stages.cpp.

Handles vision-guided robot movement:
- ArUco marker detection via Zivid camera
- TF transforms from camera to base frame
- Collision object management
- Motion to detected marker poses
"""

import json
import math
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from moveit.task_constructor import stages
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import GetPlanningScene
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from tf_transformations import quaternion_multiply, quaternion_from_euler, quaternion_matrix

from zivid_interfaces.srv import CaptureAndDetectMarkers

from mtc_py_lib.stages.base_stages import BaseStages


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
    """Handles vision-guided movement to ArUco markers."""

    # Default parameters
    DEFAULT_MARKER_DICTIONARY = "aruco4x4_50"
    DEFAULT_CAMERA_FRAME = "zivid_optical_frame"

    def __init__(self, rclpy_node, arm_group: str = "", ik_frame: str = ""):
        """Initialize VisionStages.

        Args:
            rclpy_node: ROS node for service calls and TF
            arm_group: MoveIt planning group for arm
            ik_frame: IK frame (empty = auto-detect)
        """
        super().__init__(rclpy_node, arm_group, ik_frame=ik_frame)

        # TF2
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, rclpy_node)
        self._tf_broadcaster = TransformBroadcaster(rclpy_node)

        # Zivid service client
        self._capture_client = rclpy_node.create_client(
            CaptureAndDetectMarkers,
            "/capture_and_detect_markers"
        )

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
        self._marker_dictionary = self.DEFAULT_MARKER_DICTIONARY
        self._publish_marker_frames = True

        # Object database (loaded from config)
        self._object_database: Dict[int, ObjectInfo] = {}

        # Load vision objects config
        self._load_vision_objects_config()

        self.logger.info(
            f"VisionStages initialized (ik_frame: "
            f"{'auto-detect' if not ik_frame else ik_frame})"
        )

    def _load_vision_objects_config(self):
        """Load vision objects configuration from JSON file."""
        try:
            config_path = (
                get_package_share_directory("mtc_pipeline") +
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
            goal: VisionMoveToAction.Goal with tag_id and timeout

        Returns:
            True if successful, False otherwise
        """
        tag_pose = self.detect_and_transform_tag(goal.tag_id, goal.timeout)
        if tag_pose is None:
            self.logger.error(f"Failed to detect tag {goal.tag_id}")
            return False

        return self._move_to_pose(tag_pose)

    def detect_and_transform_tag(
        self,
        tag_id: int,
        timeout: float = 10.0
    ) -> Optional[PoseStamped]:
        """Detect an ArUco marker and transform to base_link frame.

        Args:
            tag_id: ArUco marker ID to detect
            timeout: Detection timeout in seconds

        Returns:
            PoseStamped in base_link frame, or None if detection failed
        """
        self.logger.info(f"Detecting tag {tag_id}...")

        # Wait for service
        if not self._capture_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Zivid service not available")
            return None

        # Send detection request
        request = CaptureAndDetectMarkers.Request()
        request.marker_ids = [tag_id]
        request.marker_dictionary = self._marker_dictionary

        future = self._capture_client.call_async(request)

        # Wait for result
        rclpy.spin_until_future_complete(self.rclpy_node, future, timeout_sec=timeout)

        if not future.done():
            self.logger.error("Zivid service timeout")
            return None

        result = future.result()
        if not result.success:
            self.logger.error(f"Detection failed: {result.message}")
            return None

        # Find the requested marker in results
        for marker in result.detection_result.detected_markers:
            if marker.id == tag_id:
                self.logger.info(
                    f"Tag {marker.id} at [{marker.pose.position.x:.3f}, "
                    f"{marker.pose.position.y:.3f}, {marker.pose.position.z:.3f}] "
                    f"in camera frame"
                )

                # Transform to base_link
                pose_base = self._transform_to_base_link(marker.pose)
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
            f"({len(result.detection_result.detected_markers)} markers detected)"
        )
        return None

    def _transform_to_base_link(self, pose_camera: Pose) -> Optional[PoseStamped]:
        """Transform a pose from camera frame to base_link.

        Args:
            pose_camera: Pose in camera optical frame

        Returns:
            PoseStamped in base_link frame, or None on failure
        """
        try:
            # Check if transform is available
            if not self._tf_buffer.can_transform(
                "base_link",
                self.DEFAULT_CAMERA_FRAME,
                self.rclpy_node.get_clock().now(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            ):
                self.logger.error(
                    f"TF {self.DEFAULT_CAMERA_FRAME} -> base_link not available"
                )
                return None

            # Create stamped pose in camera frame
            pose_in = PoseStamped()
            pose_in.header.frame_id = self.DEFAULT_CAMERA_FRAME
            pose_in.header.stamp = self.rclpy_node.get_clock().now().to_msg()
            pose_in.pose = pose_camera

            # Transform
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                self.DEFAULT_CAMERA_FRAME,
                rclpy.time.Time()
            )
            pose_out = do_transform_pose(pose_in, transform)
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

    def _move_to_pose(self, target: PoseStamped) -> bool:
        """Move robot to the target pose with orientation adjustment.

        Applies 180° Z rotation and z_offset for proper approach.

        Args:
            target: Target pose in base_link

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

        # Compute approach pose: 180° Z rotation + z_offset
        approach = PoseStamped()
        approach.header = target.header
        approach.pose.position.x = target.pose.position.x
        approach.pose.position.y = target.pose.position.y
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
            return GripperDetection("epick_tip", 0.027)

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
