#!/usr/bin/env python3
"""FollowJointTrajectory server backed by Isaac Sim joint-state topics.

MoveIt sends trajectories to the conventional UR controller action.  This
node samples those trajectories at a fixed rate and publishes position
commands as ``sensor_msgs/JointState`` for an Isaac articulation graph.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Sequence

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand, ParallelGripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


DEFAULT_ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def validate_trajectory(joint_names: Sequence[str], points, controlled_joints):
    """Return an error string, or ``None`` when a trajectory is executable."""
    if not points:
        return "trajectory contains no points"
    if len(joint_names) != len(set(joint_names)):
        return "trajectory contains duplicate joint names"
    unknown = set(joint_names) - set(controlled_joints)
    if unknown:
        return f"unknown joints: {sorted(unknown)}"
    if set(joint_names) != set(controlled_joints):
        missing = set(controlled_joints) - set(joint_names)
        return f"trajectory does not command all controlled joints; missing: {sorted(missing)}"
    previous = -1.0
    for point in points:
        if len(point.positions) != len(joint_names):
            return "trajectory point position count does not match joint names"
        stamp = duration_seconds(point.time_from_start)
        if stamp < 0.0 or stamp <= previous:
            return "trajectory point times must be non-negative and strictly increasing"
        if any(not math.isfinite(value) for value in point.positions):
            return "trajectory contains a non-finite position"
        previous = stamp
    return None


def interpolate_positions(points, elapsed: float):
    """Linearly sample trajectory positions at seconds from its start."""
    if elapsed <= duration_seconds(points[0].time_from_start):
        return list(points[0].positions)
    for left, right in zip(points, points[1:]):
        left_t = duration_seconds(left.time_from_start)
        right_t = duration_seconds(right.time_from_start)
        if elapsed <= right_t:
            ratio = (elapsed - left_t) / (right_t - left_t)
            return [a + ratio * (b - a) for a, b in zip(left.positions, right.positions)]
    return list(points[-1].positions)


class IsaacTrajectoryAdapter(Node):
    def __init__(self):
        super().__init__("isaac_trajectory_adapter")
        self.declare_parameter("action_name", "/scaled_joint_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("command_topic", "/isaac_joint_commands")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joints", DEFAULT_ARM_JOINTS)
        self.declare_parameter("command_rate", 100.0)
        self.declare_parameter("goal_tolerance", 0.02)
        self.declare_parameter("goal_time_tolerance", 2.0)
        self.declare_parameter("joint_state_timeout", 1.0)
        self.declare_parameter("gripper", "none")

        self._joints = list(self.get_parameter("joints").value)
        self._rate = float(self.get_parameter("command_rate").value)
        self._goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self._goal_time_tolerance = float(self.get_parameter("goal_time_tolerance").value)
        self._state_timeout = float(self.get_parameter("joint_state_timeout").value)
        if self._rate <= 0.0:
            raise ValueError("command_rate must be positive")

        group = ReentrantCallbackGroup()
        self._command_pub = self.create_publisher(
            JointState, str(self.get_parameter("command_topic").value), 10
        )
        self._state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._state_callback,
            20,
            callback_group=group,
        )
        self._state_lock = threading.Lock()
        self._positions = {}
        self._state_received_at = None
        self._goal_lock = threading.Lock()
        self._goal_active = False
        self._gripper_active = False
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )
        self._gripper_server = self._create_gripper_server(group)
        self.get_logger().info(
            "Bridging FollowJointTrajectory to %s for joints: %s"
            % (self.get_parameter("command_topic").value, ", ".join(self._joints))
        )

    def _create_gripper_server(self, group):
        gripper = str(self.get_parameter("gripper").value)
        if gripper == "hande":
            action_type = ParallelGripperCommand
            name = "/gripper_action_controller/gripper_cmd"
            callback = self._execute_parallel_gripper
        elif gripper in {"2fg7", "epick"}:
            action_type = GripperCommand
            name = (
                "/epick_gripper_action_controller/gripper_cmd"
                if gripper == "epick"
                else "/gripper_action_controller/gripper_cmd"
            )
            callback = self._execute_scalar_gripper
        else:
            return None
        return ActionServer(
            self,
            action_type,
            name,
            execute_callback=callback,
            goal_callback=self._accept_gripper_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )

    def _accept_gripper_goal(self, request):
        command = request.command
        if hasattr(command, "name") and (
            not command.name
            or len(command.name) != len(command.position)
            or any(not math.isfinite(value) for value in command.position)
        ):
            return GoalResponse.REJECT
        if hasattr(command, "position") and isinstance(command.position, float) and not math.isfinite(command.position):
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._gripper_active:
                return GoalResponse.REJECT
            self._gripper_active = True
        return GoalResponse.ACCEPT

    def _drive_gripper(self, goal_handle, names, positions, feedback_factory, result_factory):
        deadline = time.monotonic() + self._goal_time_tolerance
        try:
            while rclpy.ok():
                self._publish_command(names, positions)
                actual = self._actual_positions(names)
                feedback = feedback_factory(actual)
                goal_handle.publish_feedback(feedback)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return result_factory(actual, False)
                if actual is not None and max(
                    abs(want - got) for want, got in zip(positions, actual)
                ) <= self._goal_tolerance:
                    goal_handle.succeed()
                    return result_factory(actual, True)
                if time.monotonic() >= deadline:
                    goal_handle.abort()
                    return result_factory(actual, False)
                time.sleep(1.0 / self._rate)
        finally:
            with self._goal_lock:
                self._gripper_active = False

    def _execute_parallel_gripper(self, goal_handle):
        command = goal_handle.request.command
        names = list(command.name)
        positions = list(command.position)

        def message(actual, reached, result=False):
            value = ParallelGripperCommand.Result() if result else ParallelGripperCommand.Feedback()
            value.state.name = names
            value.state.position = list(actual or positions)
            value.reached_goal = reached
            return value

        return self._drive_gripper(
            goal_handle, names, positions,
            lambda actual: message(actual, False),
            lambda actual, reached: message(actual, reached, result=True),
        )

    def _execute_scalar_gripper(self, goal_handle):
        gripper = str(self.get_parameter("gripper").value)
        name = "gripper" if gripper == "epick" else "twofg7_left_finger_joint"
        position = float(goal_handle.request.command.position)

        def message(actual, reached, result=False):
            value = GripperCommand.Result() if result else GripperCommand.Feedback()
            value.position = actual[0] if actual else position
            value.reached_goal = reached
            return value

        return self._drive_gripper(
            goal_handle, [name], [position],
            lambda actual: message(actual, False),
            lambda actual, reached: message(actual, reached, result=True),
        )

    def _state_callback(self, msg: JointState):
        with self._state_lock:
            self._positions.update(dict(zip(msg.name, msg.position)))
            self._state_received_at = time.monotonic()

    def _accept_goal(self, request):
        error = validate_trajectory(request.trajectory.joint_names, request.trajectory.points, self._joints)
        if error:
            self.get_logger().error(f"Rejecting trajectory: {error}")
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warning("Rejecting trajectory while another goal is active")
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def _actual_positions(self, names):
        with self._state_lock:
            if self._state_received_at is None or time.monotonic() - self._state_received_at > self._state_timeout:
                return None
            if any(name not in self._positions for name in names):
                return None
            return [self._positions[name] for name in names]

    def _publish_command(self, names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = list(positions)
        self._command_pub.publish(msg)

    def _publish_feedback(self, goal_handle, names, desired, elapsed, actual):
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(names)
        feedback.desired = JointTrajectoryPoint(positions=list(desired))
        feedback.desired.time_from_start.sec = int(elapsed)
        feedback.desired.time_from_start.nanosec = int((elapsed % 1.0) * 1e9)
        if actual is not None:
            feedback.actual = JointTrajectoryPoint(positions=list(actual))
            feedback.error = JointTrajectoryPoint(
                positions=[want - got for want, got in zip(desired, actual)]
            )
        goal_handle.publish_feedback(feedback)

    def _result(self, code, text):
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = text
        return result

    def _execute(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        points = trajectory.points
        final_time = duration_seconds(points[-1].time_from_start)
        period = 1.0 / self._rate
        started = time.monotonic()
        try:
            while rclpy.ok():
                elapsed = time.monotonic() - started
                desired = interpolate_positions(points, min(elapsed, final_time))
                self._publish_command(names, desired)
                actual = self._actual_positions(names)
                self._publish_feedback(goal_handle, names, desired, elapsed, actual)

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "trajectory canceled")

                if elapsed >= final_time:
                    if actual is not None and max(abs(a - b) for a, b in zip(actual, desired)) <= self._goal_tolerance:
                        goal_handle.succeed()
                        return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "")
                    if elapsed >= final_time + self._goal_time_tolerance:
                        goal_handle.abort()
                        text = "joint feedback missing or outside goal tolerance"
                        return self._result(FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, text)
                time.sleep(period)

            goal_handle.abort()
            return self._result(FollowJointTrajectory.Result.INVALID_GOAL, "ROS shutdown")
        finally:
            with self._goal_lock:
                self._goal_active = False

    def destroy_node(self):
        self._server.destroy()
        if self._gripper_server is not None:
            self._gripper_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IsaacTrajectoryAdapter()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
