# tf_utilities.py
# Copyright 2024 Brookhaven National Laboratory
# BSD 3-Clause License. See LICENSE.txt for details.

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros import Buffer, TransformListener, TransformException
import tf_transformations


class TFUtilities:
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger().get_child('tf_util')

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        # Parameter: suffix for pre-pickup approach frame
        # Declare and read from this node's parameters
        param = self.node.declare_parameter('pre_pickup_location.name', '')
        self.pre_pickup_approach_point_frame_suffix = param.value

        # Fixed frame names
        self.world_frame = 'map'
        self.grasping_point_on_gripper_frame = 'grasping_point_link'
        self.wrist_2_frame = 'wrist_2_link'

    def degrees_to_radians(self, degrees: float) -> float:
        """Convert degrees to radians."""
        return degrees * math.pi / 180.0

    def get_sample_pose(self, sample_id: int) -> TransformStamped:
        """Get the sample pose by ID."""
        sample_frame = str(sample_id)
        try:
            sample_pose = self.tf_buffer.lookup_transform(
                self.world_frame,
                sample_frame,
                Time(),
                Duration(seconds=10.0)
            )
            return sample_pose
        except TransformException as ex:
            self.logger.error(f'TF lookup for sample_pose failed: {ex}')
            raise

    def get_sample_pre_pickup_pose(self, sample_id: int) -> TransformStamped:
        """Get the pre-pickup pose of the sample by ID."""
        frame = f"{sample_id}_{self.pre_pickup_approach_point_frame_suffix}"
        try:
            pre_pickup_pose = self.tf_buffer.lookup_transform(
                self.world_frame,
                frame,
                Time(),
                Duration(seconds=10.0)
            )
            return pre_pickup_pose
        except TransformException as ex:
            self.logger.error(f'TF lookup for pre_pickup_pose failed: {ex}')
            raise

    def get_wrist_elbow_alignment(
        self,
        mgi,  # MoveGroupInterface or MoveGroupCommander
        sample_pose: TransformStamped
    ) -> Tuple[float, float]:
        """
        Returns (wrist_adjustment_rad, sample_yaw_deg) to align the arm orthogonal to the sample.
        wrist_adjustment = current_joint[4] + wrist2_yaw - sample_yaw
        sample_yaw returned in degrees.
        """
        try:
            t_wrist2 = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.wrist_2_frame,
                Time(),
                Duration(seconds=10.0)
            )
        except TransformException as ex:
            self.logger.error(f'TF lookup for wrist2 failed: {ex}')
            raise

        # Convert quaternions to RPY
        q_w = t_wrist2.transform.rotation
        q_s = sample_pose.transform.rotation
        wrist2_quat = [q_w.x, q_w.y, q_w.z, q_w.w]
        sample_quat = [q_s.x, q_s.y, q_s.z, q_s.w]
        wrist2_roll, wrist2_pitch, wrist2_yaw = tf_transformations.euler_from_quaternion(wrist2_quat)
        sample_roll, sample_pitch, sample_yaw = tf_transformations.euler_from_quaternion(sample_quat)

        # Get current joint values
        try:
            joint_vals = mgi.get_current_joint_values()
        except AttributeError:
            # Fallback for moveit_py: getCurrentState()
            state = mgi.get_current_state()
            joint_vals = list(state.joint_group_positions)

        # Compute adjustments
        wrist_adjust_rad = joint_vals[4] + wrist2_yaw - sample_yaw
        sample_yaw_deg = sample_yaw * 180.0 / math.pi
        return wrist_adjust_rad, sample_yaw_deg

    def get_pickup_action_z_adj(
        self,
        mgi,
        sample_pose: TransformStamped
    ) -> List[Pose]:
        """Returns Cartesian waypoint to shift in Z direction for pickup."""
        waypoints: List[Pose] = []
        try:
            t_grasp = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.grasping_point_on_gripper_frame,
                Time(),
                Duration(seconds=10.0)
            )
            dz = (sample_pose.transform.translation.z -
                  t_grasp.transform.translation.z)
            target: Pose = mgi.get_current_pose().pose
            target.position.z += dz
            target.position.z -= 0.02  # extra 2cm down
            waypoints.append(target)
        except Exception as ex:
            self.logger.error(f'pickup_action_z_adj failed: {ex}')
        return waypoints

    def get_pickup_action_pre_pickup(
        self,
        mgi,
        pre_pickup_pose: TransformStamped
    ) -> List[Pose]:
        """Returns Cartesian waypoint for pre-pickup offset."""
        waypoints: List[Pose] = []
        try:
            t_grasp = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.grasping_point_on_gripper_frame,
                Time(),
                Duration(seconds=10.0)
            )
            dx = (pre_pickup_pose.transform.translation.x -
                  t_grasp.transform.translation.x)
            dy = (pre_pickup_pose.transform.translation.y -
                  t_grasp.transform.translation.y)
            target: Pose = mgi.get_current_pose().pose
            target.position.x += dx
            target.position.y += dy
            waypoints.append(target)
        except Exception as ex:
            self.logger.error(f'pickup_action_pre_pickup failed: {ex}')
        return waypoints

    def get_pickup_action_pickup(
        self,
        mgi,
        pre_pickup_pose: TransformStamped,
        sample_pose: TransformStamped
    ) -> List[Pose]:
        """Returns Cartesian waypoint to move from pre-pickup to actual sample."""
        waypoints: List[Pose] = []
        try:
            dx = (sample_pose.transform.translation.x -
                  pre_pickup_pose.transform.translation.x)
            dy = (sample_pose.transform.translation.y -
                  pre_pickup_pose.transform.translation.y)
            target: Pose = mgi.get_current_pose().pose
            target.position.x += dx
            target.position.y += dy
            waypoints.append(target)
        except Exception as ex:
            self.logger.error(f'pickup_action_pickup failed: {ex}')
        return waypoints
