#!/usr/bin/env python3
"""
Simple standalone script to test wafer detection with Zivid camera.

This script:
1. Calls the Zivid capture service
2. Receives image + point cloud
3. Detects circular wafer using Hough Transform
4. Displays the detection with visualization
5. Prints the 3D pose
6. Plans robot motion to detected wafer using MTC (Cartesian path)

Usage:
    # Make sure beambot_bringup is running first (provides MoveIt + TF):
    ros2 launch beambot beambot_bringup.launch.py

    # Then run this script:
    python3 test_wafer_detection.py

    # Or if installed:
    ros2 run beambot test_wafer_detection.py
"""

import cv2
import numpy as np
import struct
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Vector3Stamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from tf_transformations import quaternion_from_euler, quaternion_multiply

# TF2 for coordinate transforms
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped

# Zivid ArUco detection service
from zivid_interfaces.srv import CaptureAndDetectMarkers

# MTC imports for motion planning
from moveit.task_constructor import core, stages
from moveit_msgs.msg import MoveItErrorCodes

# Import shared MTC node from beambot
from beambot.stages.base_stages import (
    _mtc_node,
    DEFAULT_ARM_GROUP,
    DEFAULT_IK_FRAME,
    VELOCITY_SCALING,
    ACCELERATION_SCALING,
)


class WaferDetectionTest(Node):
    def __init__(self):
        super().__init__('wafer_detection_test')

        self.bridge = CvBridge()

        # Storage for received data
        self.latest_image: Optional[Image] = None
        self.latest_cloud: Optional[PointCloud2] = None

        # Detection parameters (tune these for your setup!)
        # NOTE: All radius values are in PIXELS, not mm!
        # For a 10mm radius wafer:
        #   - At ~300mm distance: ~50-70 pixels
        #   - At ~500mm distance: ~30-50 pixels
        #   - At ~800mm distance: ~20-30 pixels
        self.min_radius = 15      # Min wafer radius in pixels (small wafer, far away)
        self.max_radius = 100     # Max wafer radius in pixels (small wafer, close up)
        self.blur_kernel = 5      # Gaussian blur kernel size (smaller for small objects)
        self.param1 = 50          # Canny edge threshold
        self.param2 = 25          # Accumulator threshold (lower = more sensitive)
        self.min_dist = 50        # Min distance between detected circles

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'color/image_color', self.on_image, 10
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, 'points/xyzrgba', self.on_cloud, 10
        )

        # Capture service client
        self.capture_client = self.create_client(Trigger, 'capture')

        # ArUco detection service client (for comparison)
        self.aruco_client = self.create_client(
            CaptureAndDetectMarkers, '/capture_and_detect_markers'
        )

        # Publishers for RViz visualization
        self.pose_pub = self.create_publisher(PoseStamped, 'detected_wafer/pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'detected_wafer/markers', 10)

        # Camera frame (must match your TF tree)
        self.camera_frame = 'zivid_optical_frame'

        # Wafer physical parameters (for visualization)
        self.wafer_radius_m = 0.010  # 10mm radius - adjust to your wafer!
        self.wafer_thickness_m = 0.001  # 1mm thickness

        # TF2 for transforms
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Store last detected pose for move command
        self.last_detected_pose_camera: Optional[PoseStamped] = None
        self.last_detected_pose_base: Optional[PoseStamped] = None

        # Z offset for approach (adjust based on your gripper)
        # ePick: ~0.05-0.1m above (vacuum from above)
        # Hand-E: ~0.02m (side grasp)
        self.approach_z_offset = 0.05  # 5cm above wafer for safety

        self.get_logger().info('=' * 60)
        self.get_logger().info('Wafer Detection Test')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Detection parameters:')
        self.get_logger().info(f'  min_radius: {self.min_radius} px')
        self.get_logger().info(f'  max_radius: {self.max_radius} px')
        self.get_logger().info(f'  param2 (sensitivity): {self.param2}')
        self.get_logger().info('=' * 60)

    def on_image(self, msg: Image):
        """Store latest image."""
        self.latest_image = msg
        self.get_logger().info(f'Received image: {msg.width}x{msg.height}')

    def on_cloud(self, msg: PointCloud2):
        """Store latest point cloud."""
        self.latest_cloud = msg
        self.get_logger().info(f'Received point cloud: {msg.width}x{msg.height} points')

    def wait_for_service(self) -> bool:
        """Wait for capture service to be available."""
        self.get_logger().info('Waiting for capture service...')
        if not self.capture_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Capture service not available!')
            self.get_logger().error('Make sure zivid_camera node is running:')
            self.get_logger().error('  ros2 launch zivid_camera zivid_camera.launch.py')
            return False
        self.get_logger().info('Capture service available!')
        return True

    def capture(self) -> bool:
        """Trigger a capture and wait for data."""
        self.get_logger().info('Triggering capture...')

        # Clear previous data
        self.latest_image = None
        self.latest_cloud = None

        # Call capture service
        future = self.capture_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            self.get_logger().error('Capture service call timed out!')
            return False

        result = future.result()
        if not result.success:
            self.get_logger().error(f'Capture failed: {result.message}')
            return False

        self.get_logger().info('Capture triggered, waiting for data...')

        # Wait for image and cloud to arrive
        timeout = 5.0  # seconds
        rate = self.create_rate(10)  # 10 Hz
        elapsed = 0.0

        while elapsed < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_image is not None and self.latest_cloud is not None:
                self.get_logger().info('Data received!')
                return True
            elapsed += 0.1

        self.get_logger().error('Timeout waiting for image/cloud data!')
        return False

    def detect_circle(self, rgb_image: np.ndarray) -> Optional[Tuple[int, int, int]]:
        """
        Detect circular wafer in RGB image using Hough Transform.

        Returns:
            (center_x, center_y, radius) in pixels, or None if not found
        """
        # Convert to grayscale
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

        # Blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 2)

        # Detect circles using Hough Transform
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,                      # Accumulator resolution ratio
            minDist=self.min_dist,     # Min distance between circle centers
            param1=self.param1,        # Canny edge detection threshold
            param2=self.param2,        # Accumulator threshold
            minRadius=self.min_radius,
            maxRadius=self.max_radius
        )

        if circles is None:
            return None

        # Take the strongest detection (first one)
        circles = np.uint16(np.around(circles))
        cx, cy, radius = circles[0, 0]

        return int(cx), int(cy), int(radius)

    def get_3d_position(
        self,
        cloud: PointCloud2,
        cx: int,
        cy: int,
        search_radius: int = 10
    ) -> Optional[Tuple[float, float, float]]:
        """
        Get 3D position from organized point cloud at pixel (cx, cy).

        The point cloud is organized (same dimensions as image), so we can
        directly index into it using pixel coordinates.
        """
        width = cloud.width
        height = cloud.height
        point_step = cloud.point_step  # Bytes per point (typically 16 for XYZRGBA)

        def get_xyz_at(u: int, v: int) -> Optional[Tuple[float, float, float]]:
            """Extract XYZ at pixel (u, v)."""
            if u < 0 or u >= width or v < 0 or v >= height:
                return None

            # Calculate byte offset into the data array
            # row_step = width * point_step for organized clouds
            offset = v * cloud.row_step + u * point_step

            # Extract X, Y, Z (first 12 bytes as 3 floats, little-endian)
            try:
                x, y, z = struct.unpack_from('<fff', cloud.data, offset)
            except struct.error:
                return None

            # Check for NaN/invalid depth
            if np.isnan(x) or np.isnan(y) or np.isnan(z):
                return None

            # Check for zero (sometimes indicates invalid)
            if x == 0.0 and y == 0.0 and z == 0.0:
                return None

            return (x, y, z)

        # Try center first
        xyz = get_xyz_at(cx, cy)
        if xyz is not None:
            return xyz

        # Search in expanding squares around center
        for r in range(1, search_radius + 1):
            for du in range(-r, r + 1):
                for dv in range(-r, r + 1):
                    # Only check perimeter of square (optimization)
                    if abs(du) == r or abs(dv) == r:
                        xyz = get_xyz_at(cx + du, cy + dv)
                        if xyz is not None:
                            self.get_logger().info(
                                f'Found valid depth at offset ({du}, {dv}) from center'
                            )
                            return xyz

        return None

    def publish_rviz_markers(
        self,
        pose_3d: Tuple[float, float, float],
        radius_pixels: int,
        timestamp
    ):
        """
        Publish visualization markers for RViz.

        Publishes:
        - PoseStamped on 'detected_wafer/pose'
        - MarkerArray on 'detected_wafer/markers' containing:
          - Cyan cylinder representing the wafer
          - Red arrow showing approach direction (Z-down)
        """
        x, y, z = pose_3d

        # --- Publish PoseStamped ---
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.camera_frame
        pose_msg.header.stamp = timestamp

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # Orientation: flat surface facing camera (identity quaternion in optical frame)
        # In optical frame, Z points forward (into scene), so wafer surface is perpendicular to Z
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.pose_pub.publish(pose_msg)

        # --- Publish MarkerArray ---
        marker_array = MarkerArray()

        # Marker 1: Cylinder representing the wafer (cyan)
        wafer_marker = Marker()
        wafer_marker.header.frame_id = self.camera_frame
        wafer_marker.header.stamp = timestamp
        wafer_marker.ns = "wafer"
        wafer_marker.id = 0
        wafer_marker.type = Marker.CYLINDER
        wafer_marker.action = Marker.ADD

        wafer_marker.pose.position.x = x
        wafer_marker.pose.position.y = y
        wafer_marker.pose.position.z = z

        # Cylinder axis is Z by default in RViz
        # With identity quaternion, cylinder axis aligns with optical Z (into surface)
        # This matches the coordinate frame axes (which are also identity orientation)
        wafer_marker.pose.orientation.x = 0.0
        wafer_marker.pose.orientation.y = 0.0
        wafer_marker.pose.orientation.z = 0.0
        wafer_marker.pose.orientation.w = 1.0

        # Size: diameter x diameter x thickness (along Z axis)
        wafer_marker.scale.x = self.wafer_radius_m * 2  # diameter
        wafer_marker.scale.y = self.wafer_radius_m * 2  # diameter
        wafer_marker.scale.z = self.wafer_thickness_m   # thickness

        # Cyan color, semi-transparent
        wafer_marker.color.r = 0.0
        wafer_marker.color.g = 0.8
        wafer_marker.color.b = 0.8
        wafer_marker.color.a = 0.7

        wafer_marker.lifetime.sec = 0  # Persistent until next detection

        marker_array.markers.append(wafer_marker)

        # Marker 2, 3, 4: Coordinate frame axes (like ArUco detection shows)
        # ArUco convention: Z points INTO the marker face (into the surface)
        # For a flat wafer on table: Z points down into table
        # In optical frame: +Z is into scene, so Z-axis of wafer = +Z of camera

        axis_length = 0.03  # 3cm axes
        axis_width = 0.003  # 3mm thick

        # Z-axis (Blue) - points INTO surface (same as camera +Z)
        z_arrow = Marker()
        z_arrow.header.frame_id = self.camera_frame
        z_arrow.header.stamp = timestamp
        z_arrow.ns = "frame"
        z_arrow.id = 1
        z_arrow.type = Marker.ARROW
        z_arrow.action = Marker.ADD
        z_arrow.pose.position.x = x
        z_arrow.pose.position.y = y
        z_arrow.pose.position.z = z
        # Arrow default is +X, rotate to +Z: -90° around Y
        z_arrow.pose.orientation.x = 0.0
        z_arrow.pose.orientation.y = -0.707
        z_arrow.pose.orientation.z = 0.0
        z_arrow.pose.orientation.w = 0.707
        z_arrow.scale.x = axis_length
        z_arrow.scale.y = axis_width
        z_arrow.scale.z = axis_width * 2
        z_arrow.color.r = 0.0
        z_arrow.color.g = 0.0
        z_arrow.color.b = 1.0
        z_arrow.color.a = 1.0
        z_arrow.lifetime.sec = 0
        marker_array.markers.append(z_arrow)

        # X-axis (Red) - in plane of wafer
        x_arrow = Marker()
        x_arrow.header.frame_id = self.camera_frame
        x_arrow.header.stamp = timestamp
        x_arrow.ns = "frame"
        x_arrow.id = 2
        x_arrow.type = Marker.ARROW
        x_arrow.action = Marker.ADD
        x_arrow.pose.position.x = x
        x_arrow.pose.position.y = y
        x_arrow.pose.position.z = z
        # Default +X, no rotation needed
        x_arrow.pose.orientation.w = 1.0
        x_arrow.scale.x = axis_length
        x_arrow.scale.y = axis_width
        x_arrow.scale.z = axis_width * 2
        x_arrow.color.r = 1.0
        x_arrow.color.g = 0.0
        x_arrow.color.b = 0.0
        x_arrow.color.a = 1.0
        x_arrow.lifetime.sec = 0
        marker_array.markers.append(x_arrow)

        # Y-axis (Green) - in plane of wafer
        y_arrow = Marker()
        y_arrow.header.frame_id = self.camera_frame
        y_arrow.header.stamp = timestamp
        y_arrow.ns = "frame"
        y_arrow.id = 3
        y_arrow.type = Marker.ARROW
        y_arrow.action = Marker.ADD
        y_arrow.pose.position.x = x
        y_arrow.pose.position.y = y
        y_arrow.pose.position.z = z
        # Arrow default is +X, rotate to +Y: +90° around Z
        y_arrow.pose.orientation.x = 0.0
        y_arrow.pose.orientation.y = 0.0
        y_arrow.pose.orientation.z = 0.707
        y_arrow.pose.orientation.w = 0.707
        y_arrow.scale.x = axis_length
        y_arrow.scale.y = axis_width
        y_arrow.scale.z = axis_width * 2
        y_arrow.color.r = 0.0
        y_arrow.color.g = 1.0
        y_arrow.color.b = 0.0
        y_arrow.color.a = 1.0
        y_arrow.lifetime.sec = 0
        marker_array.markers.append(y_arrow)

        # Marker 5: Text label
        text_marker = Marker()
        text_marker.header.frame_id = self.camera_frame
        text_marker.header.stamp = timestamp
        text_marker.ns = "label"
        text_marker.id = 5
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.pose.position.x = x
        text_marker.pose.position.y = y - 0.03  # Offset below
        text_marker.pose.position.z = z

        text_marker.text = f"Wafer\nZ: {z*1000:.0f}mm"

        text_marker.scale.z = 0.015  # Text height

        # White text
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0

        text_marker.lifetime.sec = 0

        marker_array.markers.append(text_marker)

        # Publish
        self.marker_pub.publish(marker_array)
        self.get_logger().info(
            f'Published RViz markers on /detected_wafer/pose and /detected_wafer/markers'
        )

    def transform_to_base_link(self, pose_camera: PoseStamped) -> Optional[PoseStamped]:
        """Transform pose from camera frame to base_link.

        Args:
            pose_camera: Pose in camera optical frame

        Returns:
            PoseStamped in base_link frame, or None on failure
        """
        try:
            # Check if transform is available
            if not self._tf_buffer.can_transform(
                "base_link",
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            ):
                self.get_logger().error(
                    f"TF {self.camera_frame} -> base_link not available"
                )
                return None

            # Lookup and apply transform
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                self.camera_frame,
                rclpy.time.Time()
            )
            pose_base = do_transform_pose_stamped(pose_camera, transform)
            pose_base.header.frame_id = "base_link"

            return pose_base

        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    def move_to_wafer(self) -> bool:
        """Display the detected wafer pose for verification.

        NOTE: Direct Cartesian movement is not yet implemented.
        This prints the pose so you can verify detection is correct
        and compare with RViz visualization.

        Returns:
            True (always, since this is just display)
        """
        from tf_transformations import euler_from_quaternion

        if self.last_detected_pose_base is None:
            self.get_logger().error("No detected wafer pose! Capture first (SPACE).")
            return False

        p = self.last_detected_pose_base.pose.position
        q = self.last_detected_pose_base.pose.orientation

        # Apply z_offset for approach height
        approach_z = p.z + self.approach_z_offset

        # Apply 180° rotation around Z axis (same as vision_stages)
        q_orig = [q.x, q.y, q.z, q.w]
        q_rot = quaternion_from_euler(0, 0, math.pi)
        q_final = quaternion_multiply(q_orig, q_rot)

        # Convert to euler for readability
        euler_orig = euler_from_quaternion(q_orig)
        euler_final = euler_from_quaternion(q_final)

        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info("WAFER POSE SUMMARY (for robot movement)")
        self.get_logger().info("=" * 70)
        self.get_logger().info("")
        self.get_logger().info("Detected Position (base_link):")
        self.get_logger().info(f"  X: {p.x*1000:8.1f} mm")
        self.get_logger().info(f"  Y: {p.y*1000:8.1f} mm")
        self.get_logger().info(f"  Z: {p.z*1000:8.1f} mm")
        self.get_logger().info("")
        self.get_logger().info("Approach Position (with z_offset):")
        self.get_logger().info(f"  X: {p.x*1000:8.1f} mm")
        self.get_logger().info(f"  Y: {p.y*1000:8.1f} mm")
        self.get_logger().info(f"  Z: {approach_z*1000:8.1f} mm  (+{self.approach_z_offset*1000:.0f}mm offset)")
        self.get_logger().info("")
        self.get_logger().info("Approach Orientation (with 180° Z rotation):")
        self.get_logger().info(f"  Roll:  {math.degrees(euler_final[0]):7.1f}°")
        self.get_logger().info(f"  Pitch: {math.degrees(euler_final[1]):7.1f}°")
        self.get_logger().info(f"  Yaw:   {math.degrees(euler_final[2]):7.1f}°")
        self.get_logger().info("")
        self.get_logger().info("Quaternion (for task JSON):")
        self.get_logger().info(f"  [{q_final[0]:.6f}, {q_final[1]:.6f}, {q_final[2]:.6f}, {q_final[3]:.6f}]")
        self.get_logger().info("")
        self.get_logger().info("-" * 70)
        self.get_logger().info("To test movement, create a task JSON with this pose and run via GUI/client")
        self.get_logger().info("-" * 70)
        self.get_logger().info("")

        return True

    def plan_to_wafer(self, execute: bool = False) -> bool:
        """
        Plan (and optionally execute) robot motion to detected wafer using MTC.

        Uses Cartesian path planning for the approach motion.

        Args:
            execute: If True, execute the planned motion. If False, plan only.

        Returns:
            True if planning (and execution if requested) succeeded
        """
        from tf_transformations import euler_from_quaternion

        if self.last_detected_pose_base is None:
            self.get_logger().error("No detected wafer pose! Capture first (SPACE).")
            return False

        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info("MTC CARTESIAN PLANNING TO WAFER")
        self.get_logger().info("=" * 70)

        # Get pose from detection
        p = self.last_detected_pose_base.pose.position
        q = self.last_detected_pose_base.pose.orientation

        # Apply 180° rotation around Z axis (same as vision_stages)
        # This orients the gripper to approach from above
        q_orig = [q.x, q.y, q.z, q.w]
        q_rot = quaternion_from_euler(0, 0, math.pi)
        q_final = quaternion_multiply(q_orig, q_rot)

        # Create approach pose (offset above the wafer)
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = "base_link"
        approach_pose.header.stamp = self.get_clock().now().to_msg()
        approach_pose.pose.position.x = p.x
        approach_pose.pose.position.y = p.y
        approach_pose.pose.position.z = p.z + self.approach_z_offset
        approach_pose.pose.orientation.x = q_final[0]
        approach_pose.pose.orientation.y = q_final[1]
        approach_pose.pose.orientation.z = q_final[2]
        approach_pose.pose.orientation.w = q_final[3]

        euler = euler_from_quaternion(q_final)
        self.get_logger().info(f"Target approach pose (base_link):")
        self.get_logger().info(f"  Position: ({p.x*1000:.1f}, {p.y*1000:.1f}, {(p.z + self.approach_z_offset)*1000:.1f}) mm")
        self.get_logger().info(f"  Orientation: ({math.degrees(euler[0]):.1f}, {math.degrees(euler[1]):.1f}, {math.degrees(euler[2]):.1f})°")
        self.get_logger().info("")

        try:
            # Create MTC Task
            self.get_logger().info("Creating MTC task...")
            task = core.Task()
            task.name = "wafer_approach"
            task.loadRobotModel(_mtc_node)

            # Stage 1: Current state
            self.get_logger().info("  Adding CurrentState stage...")
            task.add(stages.CurrentState("current_state"))

            # Create Cartesian planner
            self.get_logger().info("  Creating Cartesian planner...")
            cartesian_planner = core.CartesianPath()
            cartesian_planner.max_velocity_scaling_factor = VELOCITY_SCALING
            cartesian_planner.max_acceleration_scaling_factor = ACCELERATION_SCALING
            cartesian_planner.step_size = 0.001  # 1mm step size
            cartesian_planner.min_fraction = 0.8  # Require 80% of path to be valid

            # Stage 2: MoveTo approach pose using Cartesian planner
            self.get_logger().info("  Adding MoveTo stage (Cartesian)...")
            move_stage = stages.MoveTo("move_to_wafer", cartesian_planner)
            move_stage.group = DEFAULT_ARM_GROUP

            # Set IK frame
            ik_frame_pose = PoseStamped()
            ik_frame_pose.header.frame_id = DEFAULT_IK_FRAME
            move_stage.ik_frame = ik_frame_pose

            # Set goal as Cartesian pose
            move_stage.setGoal(approach_pose)

            task.add(move_stage)

            # Initialize task
            self.get_logger().info("Initializing task...")
            try:
                task.init()
            except Exception as e:
                self.get_logger().error(f"Task init failed: {e}")
                return False

            # Plan
            self.get_logger().info("Planning...")
            if not task.plan(max_solutions=1):
                self.get_logger().error("Planning failed!")
                self.get_logger().error("Possible causes:")
                self.get_logger().error("  - Kinematics solver not loaded (is beambot_bringup running?)")
                self.get_logger().error("  - Target pose unreachable")
                self.get_logger().error("  - Collision detected")
                return False

            if not task.solutions:
                self.get_logger().error("No solutions found!")
                return False

            self.get_logger().info(f"SUCCESS! Found {len(task.solutions)} solution(s)")
            self.get_logger().info("")

            # Print solution info
            solution = task.solutions[0]
            self.get_logger().info("Solution details:")
            self.get_logger().info(f"  Cost: {solution.cost:.4f}")
            self.get_logger().info("")

            if execute:
                self.get_logger().info("Executing motion...")
                result = task.execute(solution)

                if result.val != MoveItErrorCodes.SUCCESS:
                    self.get_logger().error(f"Execution failed! Error code: {result.val}")
                    return False

                self.get_logger().info("Execution successful!")
            else:
                self.get_logger().info("Plan-only mode - not executing.")
                self.get_logger().info("Press 'E' to execute the motion.")

            self.get_logger().info("=" * 70)
            return True

        except Exception as e:
            self.get_logger().error(f"MTC planning failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False

    def detect_and_compare_with_tag(self, tag_id: int = 0):
        """Detect both wafer and ArUco tag, then compare their poses.

        This helps validate the wafer detection by comparing with
        a known-good ArUco detection.

        Args:
            tag_id: ArUco tag ID to detect (default: 0)
        """
        from tf_transformations import euler_from_quaternion

        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"COMPARING WAFER vs ARUCO TAG {tag_id}")
        self.get_logger().info("=" * 70)

        # Check if we have a wafer detection
        if self.last_detected_pose_base is None:
            self.get_logger().error("No wafer detected! Press SPACE to capture first.")
            return

        # Detect ArUco tag
        self.get_logger().info(f"Detecting ArUco tag {tag_id}...")

        if not self.aruco_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("ArUco detection service not available!")
            return

        request = CaptureAndDetectMarkers.Request()
        request.marker_ids = [tag_id]
        request.marker_dictionary = "aruco4x4_50"

        future = self.aruco_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            self.get_logger().error("ArUco detection timed out!")
            return

        result = future.result()
        if not result.success:
            self.get_logger().error(f"ArUco detection failed: {result.message}")
            return

        # Find the tag in results
        tag_pose_camera = None
        for marker in result.detection_result.detected_markers:
            if marker.id == tag_id:
                tag_pose_camera = marker.pose
                break

        if tag_pose_camera is None:
            self.get_logger().error(f"Tag {tag_id} not found in detection results!")
            return

        self.get_logger().info(f"Tag {tag_id} detected!")

        # Transform tag pose to base_link
        tag_pose_stamped = PoseStamped()
        tag_pose_stamped.header.frame_id = self.camera_frame
        tag_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        tag_pose_stamped.pose = tag_pose_camera

        tag_pose_base = self.transform_to_base_link(tag_pose_stamped)
        if tag_pose_base is None:
            self.get_logger().error("Failed to transform tag pose to base_link!")
            return

        # Get poses for comparison
        wafer_p = self.last_detected_pose_base.pose.position
        wafer_q = self.last_detected_pose_base.pose.orientation
        tag_p = tag_pose_base.pose.position
        tag_q = tag_pose_base.pose.orientation

        # Convert quaternions to euler
        wafer_euler = euler_from_quaternion([wafer_q.x, wafer_q.y, wafer_q.z, wafer_q.w])
        tag_euler = euler_from_quaternion([tag_q.x, tag_q.y, tag_q.z, tag_q.w])

        # Calculate differences
        dx = (wafer_p.x - tag_p.x) * 1000  # mm
        dy = (wafer_p.y - tag_p.y) * 1000
        dz = (wafer_p.z - tag_p.z) * 1000
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        d_roll = math.degrees(wafer_euler[0] - tag_euler[0])
        d_pitch = math.degrees(wafer_euler[1] - tag_euler[1])
        d_yaw = math.degrees(wafer_euler[2] - tag_euler[2])

        # Print comparison
        self.get_logger().info("")
        self.get_logger().info("                      WAFER          TAG 0         DIFF")
        self.get_logger().info("                    ─────────      ─────────     ─────────")
        self.get_logger().info(f"Position X (mm):    {wafer_p.x*1000:9.1f}      {tag_p.x*1000:9.1f}     {dx:+8.1f}")
        self.get_logger().info(f"Position Y (mm):    {wafer_p.y*1000:9.1f}      {tag_p.y*1000:9.1f}     {dy:+8.1f}")
        self.get_logger().info(f"Position Z (mm):    {wafer_p.z*1000:9.1f}      {tag_p.z*1000:9.1f}     {dz:+8.1f}")
        self.get_logger().info(f"                                              ─────────")
        self.get_logger().info(f"Distance (mm):                                  {dist:8.1f}")
        self.get_logger().info("")
        self.get_logger().info(f"Roll  (deg):        {math.degrees(wafer_euler[0]):9.1f}      {math.degrees(tag_euler[0]):9.1f}     {d_roll:+8.1f}")
        self.get_logger().info(f"Pitch (deg):        {math.degrees(wafer_euler[1]):9.1f}      {math.degrees(tag_euler[1]):9.1f}     {d_pitch:+8.1f}")
        self.get_logger().info(f"Yaw   (deg):        {math.degrees(wafer_euler[2]):9.1f}      {math.degrees(tag_euler[2]):9.1f}     {d_yaw:+8.1f}")
        self.get_logger().info("")
        self.get_logger().info("Quaternion (WAFER):")
        self.get_logger().info(f"  [{wafer_q.x:.6f}, {wafer_q.y:.6f}, {wafer_q.z:.6f}, {wafer_q.w:.6f}]")
        self.get_logger().info("Quaternion (TAG):")
        self.get_logger().info(f"  [{tag_q.x:.6f}, {tag_q.y:.6f}, {tag_q.z:.6f}, {tag_q.w:.6f}]")
        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info("")

    def visualize_detection(
        self,
        rgb_image: np.ndarray,
        detection: Optional[Tuple[int, int, int]],
        pose_3d: Optional[Tuple[float, float, float]]
    ) -> np.ndarray:
        """
        Draw detection on image and add text overlay.

        Returns:
            Annotated image (BGR format for OpenCV display)
        """
        # Convert RGB to BGR for OpenCV
        display = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

        if detection is None:
            # No detection - show error message
            cv2.putText(
                display, 'No wafer detected!', (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
            )
            cv2.putText(
                display, 'Try adjusting param2 (sensitivity)', (50, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
            )
            return display

        cx, cy, radius = detection

        # Draw detected circle (green)
        cv2.circle(display, (cx, cy), radius, (0, 255, 0), 3)

        # Draw center point (red)
        cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)

        # Draw crosshairs
        cv2.line(display, (cx - 20, cy), (cx + 20, cy), (0, 0, 255), 2)
        cv2.line(display, (cx, cy - 20), (cx, cy + 20), (0, 0, 255), 2)

        # Add text overlay
        text_y = 30
        cv2.putText(
            display, f'Detection: ({cx}, {cy}) r={radius}px', (10, text_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )

        if pose_3d is not None:
            x, y, z = pose_3d
            text_y += 30
            cv2.putText(
                display, f'3D Pose (camera frame):', (10, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2
            )
            text_y += 30
            cv2.putText(
                display, f'  X: {x*1000:.1f} mm', (10, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2
            )
            text_y += 30
            cv2.putText(
                display, f'  Y: {y*1000:.1f} mm', (10, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2
            )
            text_y += 30
            cv2.putText(
                display, f'  Z: {z*1000:.1f} mm (depth)', (10, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2
            )
        else:
            text_y += 30
            cv2.putText(
                display, 'No valid depth at center!', (10, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
            )

        # Instructions at bottom
        h = display.shape[0]
        cv2.putText(
            display, 'SPACE: capture | T: compare tag | P: plan | E: execute | M: summary | Q: quit', (10, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1
        )

        return display

    def run_detection(self):
        """Run detection on latest captured data."""
        if self.latest_image is None or self.latest_cloud is None:
            self.get_logger().error('No data available!')
            return

        # Convert ROS Image to OpenCV
        rgb_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='rgb8')

        self.get_logger().info('Running circle detection...')

        # Detect circle
        detection = self.detect_circle(rgb_image)

        pose_3d = None
        if detection is not None:
            cx, cy, radius = detection
            self.get_logger().info(f'Circle detected at ({cx}, {cy}) with radius {radius}px')

            # Get 3D position
            pose_3d = self.get_3d_position(self.latest_cloud, cx, cy)

            if pose_3d is not None:
                x, y, z = pose_3d
                # Estimate real-world radius from pixels and depth
                # Approximate: real_size ≈ pixel_size × depth / focal_length
                # Using rough focal length estimate of ~1800 pixels for Zivid 2+
                estimated_radius_mm = (radius * z * 1000) / 1800  # rough estimate

                self.get_logger().info('=' * 60)
                self.get_logger().info('DETECTION RESULT:')
                self.get_logger().info(f'  Pixel radius: {radius} px')
                self.get_logger().info(f'  Estimated real radius: ~{estimated_radius_mm:.1f} mm')
                self.get_logger().info('')
                self.get_logger().info('3D POSE (in camera optical frame):')
                self.get_logger().info(f'  X: {x:.4f} m ({x*1000:.1f} mm)')
                self.get_logger().info(f'  Y: {y:.4f} m ({y*1000:.1f} mm)')
                self.get_logger().info(f'  Z: {z:.4f} m ({z*1000:.1f} mm) [depth]')
                self.get_logger().info('=' * 60)

                # Publish RViz markers
                self.publish_rviz_markers(
                    pose_3d,
                    radius,
                    self.latest_image.header.stamp
                )

                # Store pose in camera frame
                self.last_detected_pose_camera = PoseStamped()
                self.last_detected_pose_camera.header.frame_id = self.camera_frame
                self.last_detected_pose_camera.header.stamp = self.latest_image.header.stamp
                self.last_detected_pose_camera.pose.position.x = x
                self.last_detected_pose_camera.pose.position.y = y
                self.last_detected_pose_camera.pose.position.z = z
                self.last_detected_pose_camera.pose.orientation.w = 1.0  # Identity (Z into surface)

                # Transform to base_link for robot movement
                self.last_detected_pose_base = self.transform_to_base_link(
                    self.last_detected_pose_camera
                )

                if self.last_detected_pose_base is not None:
                    pb = self.last_detected_pose_base.pose.position
                    self.get_logger().info('')
                    self.get_logger().info('3D POSE (in base_link):')
                    self.get_logger().info(f'  X: {pb.x:.4f} m ({pb.x*1000:.1f} mm)')
                    self.get_logger().info(f'  Y: {pb.y:.4f} m ({pb.y*1000:.1f} mm)')
                    self.get_logger().info(f'  Z: {pb.z:.4f} m ({pb.z*1000:.1f} mm)')
                    self.get_logger().info('')
                    self.get_logger().info('Press M to move robot to wafer!')
                else:
                    self.get_logger().warn('Could not transform to base_link (TF not available)')
            else:
                self.get_logger().warn('Could not get valid depth at detection center!')
        else:
            self.get_logger().warn('No circle detected!')
            self.get_logger().info('Tips for small wafers (~10mm radius):')
            self.get_logger().info('  - Lower param2 for more sensitivity (try 15-20)')
            self.get_logger().info('  - For 10mm wafer at 500mm: expect ~30-50 pixel radius')
            self.get_logger().info('  - Current settings: min_r=%d, max_r=%d, param2=%d' %
                                   (self.min_radius, self.max_radius, self.param2))
            self.get_logger().info('  - Ensure good contrast (dark wafer on white paper)')
            self.get_logger().info('  - Try reducing blur_kernel to 3 for sharper edges')

        # Visualize
        display = self.visualize_detection(rgb_image, detection, pose_3d)

        # Show image
        cv2.imshow('Wafer Detection', display)

        return detection, pose_3d

    def run(self):
        """Main loop - capture and detect on keypress."""
        if not self.wait_for_service():
            return

        self.get_logger().info('')
        self.get_logger().info('Controls:')
        self.get_logger().info('  SPACE - Capture image and detect wafer')
        self.get_logger().info('  T     - Compare wafer with ArUco tag 0')
        self.get_logger().info('  P     - Plan motion to wafer (MTC Cartesian)')
        self.get_logger().info('  E     - Execute planned motion')
        self.get_logger().info('  M     - Show pose summary')
        self.get_logger().info('  Q     - Quit')
        self.get_logger().info('')

        # Create a resizable window (Zivid images are 1944x1200, too big for most screens)
        cv2.namedWindow('Wafer Detection', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Wafer Detection', 1280, 800)  # Reasonable default size

        # Initial capture
        if self.capture():
            self.run_detection()
        else:
            # Show blank window with instructions
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                blank, 'Capture failed - press SPACE to retry', (50, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
            )
            cv2.imshow('Wafer Detection', blank)

        # Main loop
        while True:
            key = cv2.waitKey(100) & 0xFF  # 100ms timeout, check for ROS callbacks

            # Spin ROS callbacks
            rclpy.spin_once(self, timeout_sec=0.01)

            if key == ord('q') or key == ord('Q'):
                self.get_logger().info('Quitting...')
                break
            elif key == ord(' '):  # Space bar
                self.get_logger().info('Capturing...')
                if self.capture():
                    self.run_detection()
            elif key == ord('t') or key == ord('T'):
                self.get_logger().info('Comparing with ArUco tag 0...')
                self.detect_and_compare_with_tag(tag_id=0)
            elif key == ord('m') or key == ord('M'):
                self.get_logger().info('Pose summary...')
                self.move_to_wafer()
            elif key == ord('p') or key == ord('P'):
                self.get_logger().info('Planning motion to wafer...')
                self.plan_to_wafer(execute=False)
            elif key == ord('e') or key == ord('E'):
                self.get_logger().info('Planning and executing motion to wafer...')
                self.plan_to_wafer(execute=True)

        cv2.destroyAllWindows()


def main():
    rclpy.init()

    node = WaferDetectionTest()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
