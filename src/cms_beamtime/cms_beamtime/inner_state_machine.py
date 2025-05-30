# inner_state_machine.py
# Python port of inner_state_machine.hpp/cpp for ROS 2 Humble without moveit_py
# Uses ROS 2 service & action clients to call underlying MoveIt2 C++ nodes

from enum import Enum, auto
from typing import List, Optional
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.srv import GetMotionPlan, GetCartesianPath
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import Pose
from pdf_beamtime_interfaces.srv import GripperControlMsg


class ExternalState(Enum):
    HOME = auto()
    PICKUP_APPROACH = auto()
    PICKUP = auto()
    GRASP_SUCCESS = auto()
    GRASP_FAILURE = auto()
    PICKUP_RETREAT = auto()
    PLACE_APPROACH = auto()
    PLACE = auto()
    RELEASE_SUCCESS = auto()
    RELEASE_FAILURE = auto()
    PLACE_RETREAT = auto()


class InternalState(Enum):
    RESTING = auto()
    MOVING = auto()
    PAUSED = auto()
    ABORT = auto()
    HALT = auto()
    STOP = auto()
    CLEANUP = auto()


class InnerStateMachine:
    def __init__(self, node: Node, gripper_node: Node):
        # ROS nodes
        self.node = node
        self.gripper_node = gripper_node

        # FSM states
        self.internal_state: InternalState = InternalState.RESTING
        self.joint_goal: List[float] = []

        # Robot configuration (fixed for UR5e)
        self.planning_group = 'ur_arm'
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.world_frame = 'world'

        # Gripper client
        self.gripper_client = gripper_node.create_client(
            GripperControlMsg, 'gripper_service')

        # MoveIt clients
        self.plan_client = node.create_client(
            GetMotionPlan, '/plan_kinematic_path')
        self.cart_client = node.create_client(
            GetCartesianPath, '/compute_cartesian_path')

        # Execution action client for trajectories
        self._traj_client = ActionClient(
            node,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
            callback_group=ReentrantCallbackGroup()
        )

        # Internal workflow indices
        self.external_sequence: List[ExternalState] = list(ExternalState)
        self.external_index: int = 0

    def move_robot(self, joint_goal: List[float]) -> bool:
        """Plan & execute a joint-space trajectory via MoveIt2 services/actions."""
        if self.internal_state != InternalState.RESTING:
            return False
        self.joint_goal = joint_goal

        # Build planning request
        req = GetMotionPlan.Request()
        mp_req = req.motion_plan_request
        mp_req.group_name = self.planning_group
        mp_req.allowed_planning_time = 5.0

        # Goal constraints
        goal_cons = Constraints()
        for name, pos in zip(self.joint_names, joint_goal):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_cons.joint_constraints.append(jc)
        mp_req.goal_constraints.append(goal_cons)

        # Plan
        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Planning service unavailable')
            return False
        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        resp = future.result()
        if resp.motion_plan_response.error_code.val != resp.motion_plan_response.error_code.SUCCESS:
            return False

        # Execute
        # Extract JointTrajectory from RobotTrajectory
        robot_traj = resp.motion_plan_response.trajectory  # moveit_msgs/RobotTrajectory
        joint_traj = robot_traj.joint_trajectory          # trajectory_msgs/JointTrajectory

        from control_msgs.action import FollowJointTrajectory
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = joint_traj

        # Send goal
        send_goal = self._traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal)
        goal_handle = send_goal.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Trajectory goal rejected')
            return False

        # Wait for result
        get_res = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, get_res)
        result = get_res.result().result
        return result.error_code == 0

    
    def move_robot_cartesian(self, waypoints: List[Pose]) -> bool:
        """Plan & execute a Cartesian path via MoveIt2 Cartesian service."""
        if self.internal_state != InternalState.RESTING:
            return False

        req = GetCartesianPath.Request()
        req.group_name = self.planning_group
        req.header.frame_id = self.world_frame
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.reuse_last_path = False

        if not self.cart_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Cartesian path service unavailable')
            return False
        future = self.cart_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        resp = future.result()
        if resp.fraction < 0.99999:
            return False

        from control_msgs.action import FollowJointTrajectory
        goal_msg = FollowJointTrajectory.Goal()
        # Extract JointTrajectory: resp.solution is RobotTrajectory
        goal_msg.trajectory = resp.solution.joint_trajectory

        send_goal = self._traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal)
        goal_handle = send_goal.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Cartesian trajectory rejected')
            return False

        get_res = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, get_res)
        result = get_res.result().result
        return result.error_code == 0


    def open_gripper(self) -> bool:
        return self._call_gripper('OPEN')

    def close_gripper(self) -> bool:
        return self._call_gripper('CLOSE')

    def _call_gripper(self, command: str) -> bool:
        if self.internal_state != InternalState.RESTING:
            return False
        req = GripperControlMsg.Request()
        req.command = command
        req.grip = 100
        if not self.gripper_client.wait_for_service(timeout_sec=5.0):
            return False
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self.gripper_node, future)
        if future.result().results:
            time.sleep(3.0)
            return True
        return False

    def pause(self) -> None:
        """Cancel current trajectory and go to PAUSED state."""
        if self.internal_state in (InternalState.RESTING, InternalState.MOVING):
            self._traj_client.cancel_all_goals_async()
            self.internal_state = InternalState.PAUSED

    def abort(self) -> None:
        """Emergency stop: cancel all and go to ABORT state."""
        self._traj_client.cancel_all_goals_async()
        self.internal_state = InternalState.ABORT

    def halt(self) -> None:
        """Immediate halt and go to HALT state."""
        self._traj_client.cancel_all_goals_async()
        self.internal_state = InternalState.HALT

    def rewind(self) -> None:
        """From PAUSED, return to RESTING."""
        if self.internal_state == InternalState.PAUSED:
            self.internal_state = InternalState.RESTING

    def set_internal_state(self, new_state: InternalState) -> None:
        self.node.get_logger().info(
            f'Internal state changed from {self.internal_state.name} to {new_state.name}'
        )
        self.internal_state = new_state

    def get_internal_state(self) -> InternalState:
        return self.internal_state

    def step_external(self) -> Optional[ExternalState]:
        self.external_index = (self.external_index + 1) % len(self.external_sequence)
        return self.external_sequence[self.external_index]

    def current_external_state(self) -> ExternalState:
        return self.external_sequence[self.external_index]
