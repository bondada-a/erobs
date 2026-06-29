#!/usr/bin/env python3
"""
Script to move robot to ArUco marker positions sequentially.
Visits each marker one by one with configurable dwell time.

Usage:
  cd /home/aditya/work/github_ws/experimental
  source install/setup.bash
  python3 send_pose_goal.py
"""

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from tf_transformations import quaternion_multiply, quaternion_from_euler
import math

# ============ CONFIGURATION ============
GOAL_FRAME = "zivid_optical_frame"

# Which link to place at the goal (gripper tip)
IK_FRAME = "epick_tip"  # or "robotiq_hande_end" or "flange"

# Apply 180° Z rotation like vision_moveto does
APPLY_VISION_ROTATION = True

# Z offset to hover above target (ePick = 0.027, Hand-E = -0.02)
Z_OFFSET = 0.026

# Apply offset in camera frame (along optical axis) vs world frame (vertical)
# - True: offset along camera Z (perpendicular to assumed surface)
# - False: offset along world Z (vertical)
OFFSET_IN_CAMERA_FRAME = True

# Motion duration per move (seconds)
MOTION_DURATION = 2.0

# Dwell time at each marker (seconds)
DWELL_TIME = 0.5

# Which markers to visit (None = all, or specify list like [0, 1, 2])
MARKERS_TO_VISIT = None  # None means visit all

# Home/scan position to return to between markers (joint angles in DEGREES)
SAMPLE_SCAN_POSE = [17.77, -113.23, -63.9, -92.31, -267.99, -160.22]

# ============ MARKER POSITIONS (camera frame) ============
# Detected 28 ArUco markers (updated 2026-01-15)
# Missing: ID 5, 19
MARKERS = {
    0:  (-0.024, -0.087, 0.406),
    1:  ( 0.016, -0.088, 0.406),
    2:  ( 0.056, -0.089, 0.407),
    3:  ( 0.095, -0.090, 0.407),
    4:  ( 0.134, -0.091, 0.408),
    # 5 not detected
    6:  ( 0.017, -0.051, 0.407),
    7:  ( 0.057, -0.052, 0.407),
    8:  ( 0.096, -0.053, 0.408),
    9:  ( 0.136, -0.054, 0.408),
    10: (-0.022, -0.014, 0.407),
    11: ( 0.018, -0.015, 0.408),
    12: ( 0.057, -0.016, 0.408),
    13: ( 0.097, -0.016, 0.408),
    14: ( 0.136, -0.017, 0.409),
    15: (-0.021,  0.024, 0.408),
    16: ( 0.019,  0.023, 0.408),
    17: ( 0.058,  0.022, 0.409),
    18: ( 0.098,  0.020, 0.409),
    # 19 not detected
    20: (-0.020,  0.060, 0.408),
    21: ( 0.020,  0.060, 0.409),
    22: ( 0.059,  0.059, 0.409),
    23: ( 0.099,  0.058, 0.410),
    24: ( 0.138,  0.056, 0.410),
    25: (-0.019,  0.097, 0.409),
    26: ( 0.021,  0.096, 0.409),
    27: ( 0.060,  0.095, 0.410),
    28: ( 0.099,  0.094, 0.410),
    29: ( 0.139,  0.093, 0.411),
}
# ========================================

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]


class MarkerVisitor(Node):
    def __init__(self):
        super().__init__('marker_visitor')

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # IK service
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Trajectory action
        self.traj_client = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        # Current joint state
        self.current_joints = None
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

    def _joint_cb(self, msg):
        self.current_joints = msg

    def wait_for_services(self):
        self.get_logger().info('Waiting for services...')

        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/compute_ik not available!')
            return False

        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Trajectory controller not available!')
            return False

        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joints is not None:
                break
        if self.current_joints is None:
            self.get_logger().error('No joint states received!')
            return False

        # Wait for TF
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.tf_buffer.can_transform('base_link', GOAL_FRAME, rclpy.time.Time()):
                break

        self.get_logger().info('All services ready!')
        return True

    def compute_ik(self, pose: PoseStamped):
        """Compute IK for pose, returns joint positions or None."""
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_arm"
        request.ik_request.robot_state.joint_state = self.current_joints
        request.ik_request.pose_stamped = pose
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.timeout = Duration(seconds=5.0).to_msg()
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_name = IK_FRAME

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            return None

        result = future.result()
        if result.error_code.val != 1:
            return None

        joint_positions = []
        for name in ARM_JOINTS:
            if name in result.solution.joint_state.name:
                idx = result.solution.joint_state.name.index(name)
                joint_positions.append(result.solution.joint_state.position[idx])
            else:
                return None

        return joint_positions

    def execute_trajectory(self, joint_positions, duration=None):
        """Send joint trajectory to controller."""
        if duration is None:
            duration = MOTION_DURATION

        trajectory = JointTrajectory()
        trajectory.joint_names = ARM_JOINTS

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(seconds=duration).to_msg()
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        return result.result.error_code == 0

    def move_to_sample_scan(self):
        """Move to sample_scan joint position."""
        print("\n  → Returning to sample_scan position...")

        # Convert degrees to radians
        joint_positions = [math.radians(deg) for deg in SAMPLE_SCAN_POSE]

        success = self.execute_trajectory(joint_positions)

        if success:
            print("  ✓ At sample_scan position")
        else:
            print("  ✗ Failed to reach sample_scan position")

        return success

    def prepare_goal_pose(self, x, y, z, verbose=False):
        """Prepare goal pose with transforms and offsets."""
        # Create pose in camera frame
        pose_camera = PoseStamped()
        pose_camera.header.frame_id = GOAL_FRAME
        pose_camera.header.stamp = self.get_clock().now().to_msg()
        pose_camera.pose.position.x = x
        pose_camera.pose.position.y = y

        # Apply offset in camera frame if enabled
        # Camera Z points INTO scene, so subtract to move TOWARD camera (hover above surface)
        if OFFSET_IN_CAMERA_FRAME and Z_OFFSET != 0.0:
            pose_camera.pose.position.z = z - Z_OFFSET
            if verbose:
                print(f"  Offset applied in camera frame: Z {z:.4f} - {Z_OFFSET} = {z - Z_OFFSET:.4f}")
        else:
            pose_camera.pose.position.z = z

        # Identity quaternion
        pose_camera.pose.orientation.w = 1.0

        # Transform to base_link
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', GOAL_FRAME,
                rclpy.time.Time(), timeout=Duration(seconds=2.0)
            )

            if verbose:
                # Show how camera Z (depth) maps to base_link
                t = transform.transform
                print(f"  TF {GOAL_FRAME} → base_link:")
                print(f"    Translation: ({t.translation.x:.4f}, {t.translation.y:.4f}, {t.translation.z:.4f})")
                print(f"    Rotation (quat): ({t.rotation.x:.4f}, {t.rotation.y:.4f}, {t.rotation.z:.4f}, {t.rotation.w:.4f})")

            goal_pose = do_transform_pose_stamped(pose_camera, transform)
            goal_pose.header.frame_id = 'base_link'
        except Exception as e:
            self.get_logger().error(f'TF transform failed: {e}')
            return None

        # Apply 180° Z rotation
        if APPLY_VISION_ROTATION:
            q_orig = [
                goal_pose.pose.orientation.x,
                goal_pose.pose.orientation.y,
                goal_pose.pose.orientation.z,
                goal_pose.pose.orientation.w
            ]
            q_rot = quaternion_from_euler(0, 0, math.pi)
            q_final = quaternion_multiply(q_orig, q_rot)
            goal_pose.pose.orientation.x = q_final[0]
            goal_pose.pose.orientation.y = q_final[1]
            goal_pose.pose.orientation.z = q_final[2]
            goal_pose.pose.orientation.w = q_final[3]

        # Apply Z offset in world frame (vertical) if not already applied in camera frame
        if not OFFSET_IN_CAMERA_FRAME and Z_OFFSET != 0.0:
            goal_pose.pose.position.z += Z_OFFSET

        return goal_pose

    def move_to_marker(self, marker_id, x, y, z):
        """Move to a single marker position."""
        print(f"\n{'='*50}")
        print(f"Moving to Marker {marker_id}")
        print(f"  Camera frame:  X={x:.4f}, Y={y:.4f}, Z={z:.4f} (Z=depth)")

        # Show TF details for first marker only
        show_tf = (marker_id == 0)
        goal_pose = self.prepare_goal_pose(x, y, z, verbose=show_tf)
        if goal_pose is None:
            print("  ERROR: Failed to prepare goal pose")
            return False

        # Show base_link position
        if OFFSET_IN_CAMERA_FRAME:
            print("  Base link (offset applied in camera frame):")
            print(f"    X={goal_pose.pose.position.x:.4f}, Y={goal_pose.pose.position.y:.4f}, Z={goal_pose.pose.position.z:.4f}")
        else:
            base_z_before_offset = goal_pose.pose.position.z - Z_OFFSET
            print("  Base link (before Z_OFFSET):")
            print(f"    X={goal_pose.pose.position.x:.4f}, Y={goal_pose.pose.position.y:.4f}, Z={base_z_before_offset:.4f}")
            print(f"  Base link (after +{Z_OFFSET}m Z offset):")
            print(f"    X={goal_pose.pose.position.x:.4f}, Y={goal_pose.pose.position.y:.4f}, Z={goal_pose.pose.position.z:.4f}")

        # Compute IK
        joint_solution = self.compute_ik(goal_pose)
        if joint_solution is None:
            print(f"  ERROR: IK failed for marker {marker_id}")
            return False

        # Execute
        print("  Executing motion...")
        success = self.execute_trajectory(joint_solution)

        if success:
            print(f"  ✓ Reached marker {marker_id}")
        else:
            print(f"  ✗ Motion failed for marker {marker_id}")

        return success

    def analyze_depth_mapping(self):
        """Analyze how camera depth maps to base_link coordinates (no motion)."""
        print(f"\n{'#'*60}")
        print("# DEPTH MAPPING ANALYSIS")
        print("# Comparing left-side (shallow Z) vs right-side (deep Z) markers")
        print(f"{'#'*60}")

        # Select representative markers: left column vs right column
        left_markers = [0, 5, 10, 15, 25]   # X ≈ -0.02, Z ≈ 0.406-0.409
        right_markers = [4, 9, 14, 19, 24]  # X ≈ 0.13-0.14, Z ≈ 0.408-0.410

        print(f"\n{'='*60}")
        print("LEFT SIDE MARKERS (closer to camera, smaller Z):")
        print(f"{'='*60}")
        for mid in left_markers:
            if mid not in MARKERS:
                continue
            x, y, z = MARKERS[mid]
            pose = self.prepare_goal_pose(x, y, z, verbose=False)
            if pose:
                bz = pose.pose.position.z - Z_OFFSET  # before offset
                print(f"  ID {mid:2d}: Camera(X={x:+.3f}, Y={y:+.3f}, Z={z:.3f}) "
                      f"→ Base(X={pose.pose.position.x:.3f}, Y={pose.pose.position.y:.3f}, Z={bz:.3f})")

        print(f"\n{'='*60}")
        print("RIGHT SIDE MARKERS (further from camera, larger Z):")
        print(f"{'='*60}")
        for mid in right_markers:
            if mid not in MARKERS:
                continue
            x, y, z = MARKERS[mid]
            pose = self.prepare_goal_pose(x, y, z, verbose=False)
            if pose:
                bz = pose.pose.position.z - Z_OFFSET  # before offset
                print(f"  ID {mid:2d}: Camera(X={x:+.3f}, Y={y:+.3f}, Z={z:.3f}) "
                      f"→ Base(X={pose.pose.position.x:.3f}, Y={pose.pose.position.y:.3f}, Z={bz:.3f})")

        # Show expected vs actual behavior
        print(f"\n{'='*60}")
        print("ANALYSIS:")
        print(f"{'='*60}")
        print("  Camera Z increases by ~4mm (0.406 → 0.410) from left to right.")
        print("  Expected: Base X or Y should increase (robot moves forward)")
        print("  Check: Does Base Z increase more than X/Y? → Camera pointing down")
        print(f"{'='*60}\n")

    def run(self):
        """Visit all markers sequentially, returning to sample_scan between each."""
        # Determine which markers to visit
        if MARKERS_TO_VISIT is None:
            marker_ids = sorted(MARKERS.keys())
        else:
            marker_ids = [m for m in MARKERS_TO_VISIT if m in MARKERS]

        print(f"\n{'#'*50}")
        print("# MARKER VISITOR")
        print(f"# Visiting {len(marker_ids)} markers: {marker_ids}")
        print("# Pattern: sample_scan → marker → sample_scan → ...")
        print(f"# IK Frame: {IK_FRAME}")
        print(f"# Z Offset: {Z_OFFSET}m ({'camera frame' if OFFSET_IN_CAMERA_FRAME else 'world frame'})")
        print(f"# Motion duration: {MOTION_DURATION}s")
        print(f"# Dwell time: {DWELL_TIME}s")
        print(f"{'#'*50}")

        successful = 0
        failed = 0
        failed_ids = []

        # Start at sample_scan position
        print("\n[INIT] Moving to initial sample_scan position...")
        if not self.move_to_sample_scan():
            print("ERROR: Could not reach initial sample_scan position!")
            return False

        for i, marker_id in enumerate(marker_ids):
            x, y, z = MARKERS[marker_id]
            print(f"\n[{i+1}/{len(marker_ids)}]", end="")

            # Move to marker
            if self.move_to_marker(marker_id, x, y, z):
                successful += 1
                # Dwell at marker
                if DWELL_TIME > 0:
                    time.sleep(DWELL_TIME)
            else:
                failed += 1
                failed_ids.append(marker_id)

            # Return to sample_scan position
            if not self.move_to_sample_scan():
                print(f"  WARNING: Could not return to sample_scan after marker {marker_id}")

        # Summary
        print(f"\n{'='*50}")
        print("SUMMARY")
        print(f"  Successful: {successful}/{len(marker_ids)}")
        print(f"  Failed: {failed}/{len(marker_ids)}")
        if failed_ids:
            print(f"  Failed markers: {failed_ids}")
        print(f"{'='*50}")

        return failed == 0


def main():
    rclpy.init()
    node = MarkerVisitor()

    # Check for analysis-only mode
    analyze_only = '--analyze' in sys.argv or '-a' in sys.argv

    try:
        if not node.wait_for_services():
            sys.exit(1)

        if analyze_only:
            print("\n[ANALYSIS MODE - No robot motion]")
            node.analyze_depth_mapping()
            sys.exit(0)
        else:
            success = node.run()
            sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
