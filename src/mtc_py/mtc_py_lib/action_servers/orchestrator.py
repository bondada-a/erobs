#!/usr/bin/env python3
"""MTC Orchestrator - coordinates multi-step robot tasks.

Python equivalent of mtc_orchestrator_action_server.cpp.
Receives task scripts (JSON) and dispatches steps to specialized action servers.
Manages MoveIt lifecycle based on gripper configuration.

Supports beamline-agnostic deployment via beamline configuration files.
"""

import json
import threading
from typing import Dict, Any
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from mtc_py.action import (
    MTCExecution,
    MoveToAction,
    EndEffectorAction,
    PickPlaceAction,
    ToolExchangeAction,
    VisionMoveToAction,
    VisionPickPlaceAction,
    PipettorAction,
)

from mtc_py_lib.core.beamline_config import load_beamline_config
from mtc_py_lib.core.moveit_lifecycle_manager import MoveItLifecycleManager


class MTCOrchestratorServer(Node):
    """Action server that coordinates multi-step robot tasks.

    Receives task scripts in JSON format and dispatches individual
    steps to specialized MTC action servers (MoveTo, EndEffector, etc.)

    Supports beamline-agnostic deployment via beamline configuration files.
    """

    # Default timeouts for each action type (seconds)
    # Can be overridden via ROS parameters: timeout.moveto, timeout.end_effector, etc.
    DEFAULT_TIMEOUTS = {
        "moveto": 120.0,
        "end_effector": 30.0,
        "pick_and_place": 180.0,
        "tool_exchange": 180.0,
        "vision_moveto": 60.0,
        "vision_pick_place": 180.0,
        "pipettor": 60.0,
    }

    def __init__(self):
        super().__init__("mtc_orchestrator_py")

        self._executing = False
        self._lock = threading.Lock()
        self._current_gripper = "none"

        # Load beamline configuration (required)
        self.declare_parameter("beamline_config", "config/default_beamline.yaml")
        self.declare_parameter("robot_ip", "")  # Can override beamline default

        beamline_config_file = self.get_parameter("beamline_config").value
        self._beamline_config = load_beamline_config(beamline_config_file)
        self.get_logger().info(f"Loaded beamline: {self._beamline_config.name}")

        # Robot IP: parameter overrides beamline config
        robot_ip_param = self.get_parameter("robot_ip").value
        self._default_robot_ip = robot_ip_param or self._beamline_config.robot.ip

        # Declare timeout parameters (configurable at launch)
        self._timeouts = {}
        for action_type, default_timeout in self.DEFAULT_TIMEOUTS.items():
            param_name = f"timeout.{action_type}"
            self.declare_parameter(param_name, default_timeout)
            self._timeouts[action_type] = self.get_parameter(param_name).value

        self.get_logger().info(f"Timeouts configured: {self._timeouts}")

        # Callback group for concurrent operations
        self._callback_group = ReentrantCallbackGroup()

        # MoveIt lifecycle manager - launches MoveIt based on gripper config
        self._moveit_manager = MoveItLifecycleManager(self, self._beamline_config)

        # Create action server
        self._action_server = ActionServer(
            self,
            MTCExecution,
            "mtc_execution_py",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        # Create action clients for specialized servers
        self._moveto_client = ActionClient(
            self, MoveToAction, "mtc_moveto_py",
            callback_group=self._callback_group
        )
        self._endeffector_client = ActionClient(
            self, EndEffectorAction, "mtc_endeffector_py",
            callback_group=self._callback_group
        )
        self._pickplace_client = ActionClient(
            self, PickPlaceAction, "mtc_pickplace_py",
            callback_group=self._callback_group
        )
        self._toolexchange_client = ActionClient(
            self, ToolExchangeAction, "mtc_toolexchange_py",
            callback_group=self._callback_group
        )
        self._vision_client = ActionClient(
            self, VisionMoveToAction, "mtc_vision_moveto_py",
            callback_group=self._callback_group
        )
        self._vision_pickplace_client = ActionClient(
            self, VisionPickPlaceAction, "mtc_vision_pickplace_py",
            callback_group=self._callback_group
        )
        self._pipettor_client = ActionClient(
            self, PipettorAction, "mtc_pipettor_py",
            callback_group=self._callback_group
        )

        self.get_logger().info("MTC Orchestrator (Python) started on 'mtc_execution_py'")

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Handle incoming goal requests."""
        with self._lock:
            if self._executing:
                self.get_logger().warn("Goal rejected: another task is executing")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Handle cancel requests."""
        self.get_logger().info("Cancel request received - will stop after current task")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the orchestration goal."""
        with self._lock:
            if self._executing:
                result = MTCExecution.Result()
                result.error_message = "Server busy"
                return result
            self._executing = True

        try:
            return self._execute(goal_handle)
        finally:
            with self._lock:
                self._executing = False

    def _execute(self, goal_handle: ServerGoalHandle) -> MTCExecution.Result:
        """Main execution logic."""
        self.get_logger().info("Executing orchestration goal")

        result = MTCExecution.Result()
        feedback = MTCExecution.Feedback()

        # Parse and validate goal
        parsed = self._parse_goal(goal_handle.request, result)
        if parsed is None:
            goal_handle.abort()
            return result

        robot_ip, start_gripper, tasks, poses_json = parsed
        task_count = len(tasks)

        # Initialize gripper state
        self._current_gripper = start_gripper

        # Step 1: Launch MoveIt for the gripper configuration
        self._update_feedback(
            feedback, goal_handle, 0, task_count, "Initializing MoveIt"
        )

        if not self._moveit_manager.launch_for_gripper(start_gripper, robot_ip):
            result.error_message = "Failed to initialize MoveIt stack"
            goal_handle.abort()
            return result

        # Execute each task
        for i, task in enumerate(tasks):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                result.error_message = "Task was canceled"
                result.completed_steps = i
                goal_handle.canceled()
                return result

            task_type = task.get("task_type", "")
            if not task_type:
                result.error_message = f"Step {i} missing 'task_type' field"
                result.completed_steps = i
                goal_handle.abort()
                return result

            # Update feedback
            self._update_feedback(
                feedback, goal_handle, i + 1, task_count, task_type
            )

            # Execute step
            if not self._execute_step(task_type, task, poses_json):
                result.error_message = f"{task_type} step failed"
                result.completed_steps = i
                goal_handle.abort()
                return result

            result.completed_steps = i + 1

        # Success
        result.success = True
        result.total_steps = task_count
        self._update_feedback(
            feedback, goal_handle, task_count, task_count, ""
        )
        goal_handle.succeed()

        self.get_logger().info("Orchestration goal completed successfully")
        return result

    def _parse_goal(self, goal: MTCExecution.Goal, result: MTCExecution.Result):
        """Parse and validate the goal JSON.

        Returns:
            (robot_ip, start_gripper, tasks, poses_json) if valid,
            None if invalid (result populated with error)
        """
        if not goal.full_json:
            result.error_message = "Goal missing required full_json"
            return None

        try:
            script = json.loads(goal.full_json)
        except json.JSONDecodeError as e:
            result.error_message = f"Invalid JSON: {e}"
            return None

        if "start_gripper" not in script:
            result.error_message = "Task script missing 'start_gripper'"
            return None

        if "tasks" not in script:
            result.error_message = "Task script missing 'tasks'"
            return None

        robot_ip = goal.robot_ip or self._default_robot_ip
        if not robot_ip:
            result.error_message = "No robot_ip provided and no default configured"
            return None

        return (
            robot_ip,
            script["start_gripper"],
            script["tasks"],
            json.dumps(script.get("poses", {})),
        )

    def _execute_step(
        self, task_type: str, step: Dict[str, Any], poses_json: str
    ) -> bool:
        """Execute a single step by dispatching to the appropriate action server."""
        self.get_logger().info(f"Executing step: {task_type}")

        if task_type == "moveto":
            return self._call_moveto(step, poses_json)
        elif task_type == "end_effector":
            return self._call_endeffector(step, poses_json)
        elif task_type == "pick_and_place":
            return self._call_pickplace(step, poses_json)
        elif task_type == "tool_exchange":
            return self._handle_tool_exchange(step, poses_json)
        elif task_type == "vision_moveto":
            return self._call_vision_moveto(step, poses_json)
        elif task_type == "vision_pick_place":
            return self._call_vision_pickplace(step, poses_json)
        elif task_type == "pipettor":
            return self._call_pipettor(step, poses_json)
        else:
            self.get_logger().error(f"Unknown task type: {task_type}")
            return False

    def _send_and_wait(
        self, client: ActionClient, goal, name: str, timeout: float
    ) -> bool:
        """Send a goal to an action client and wait for result."""

        # Wait for server
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"{name} action server unavailable")
            return False

        # Send goal and wait for acceptance
        send_future = client.send_goal_async(goal)

        # Wait for goal acceptance (10s timeout)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            self.get_logger().error(f"{name} goal acceptance timed out")
            return False

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"{name} goal was rejected")
            return False

        # Wait for result with timeout
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            self.get_logger().error(f"{name} timed out after {timeout}s")
            goal_handle.cancel_goal_async()
            return False

        result = result_future.result()
        return result.result.success

    def _call_moveto(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the MoveTo action server."""
        goal = MoveToAction.Goal()
        goal.target = step.get("target", "")
        goal.planning_type = step.get("planning_type", "joint")
        goal.direction = step.get("direction", "")
        goal.distance = float(step.get("distance", 0.0))
        goal.poses_json = poses_json

        return self._send_and_wait(
            self._moveto_client, goal, "moveto", self._timeouts["moveto"]
        )

    def _call_endeffector(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the EndEffector action server."""
        goal = EndEffectorAction.Goal()
        goal.end_effector_type = step.get("end_effector_type", "")
        goal.end_effector_action = step.get("end_effector_action", "")
        goal.poses_json = poses_json

        return self._send_and_wait(
            self._endeffector_client, goal, "end_effector", self._timeouts["end_effector"]
        )

    def _call_pickplace(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the PickPlace action server."""
        goal = PickPlaceAction.Goal()
        goal.gripper = step.get("gripper", "")
        goal.pick_approach = step.get("pick_approach", "")
        goal.pick_target = step.get("pick_target", "")
        goal.place_approach = step.get("place_approach", "")
        goal.place_target = step.get("place_target", "")
        goal.poses_json = poses_json

        return self._send_and_wait(
            self._pickplace_client, goal, "pick_place", self._timeouts["pick_and_place"]
        )

    def _call_toolexchange(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the ToolExchange action server."""
        goal = ToolExchangeAction.Goal()
        goal.operation = step.get("operation", "")
        goal.gripper = step.get("gripper", "")
        goal.current_attached_gripper = self._current_gripper
        goal.dock_number = int(step.get("dock_number", 0))
        goal.approach_pose = step.get("approach_pose", "")
        goal.poses_json = poses_json

        return self._send_and_wait(
            self._toolexchange_client, goal, "tool_exchange", self._timeouts["tool_exchange"]
        )

    def _handle_tool_exchange(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Handle tool exchange with gripper state tracking and MoveIt restart.

        Like the C++ version, this restarts MoveIt with the new gripper config
        after a tool exchange operation completes.
        """
        # Execute the physical exchange motion
        if not self._call_toolexchange(step, poses_json):
            return False

        # Update gripper state based on operation
        operation = step.get("operation", "")
        new_gripper = self._current_gripper

        if operation == "dock":
            new_gripper = "none"
        elif operation == "load":
            new_gripper = step.get("gripper", self._current_gripper)

        # If gripper changed, restart MoveIt with new configuration
        if new_gripper != self._current_gripper:
            self.get_logger().info(
                f"Gripper changed: {self._current_gripper} → {new_gripper}, restarting MoveIt"
            )
            self._current_gripper = new_gripper

            robot_ip = self._moveit_manager.robot_ip
            if robot_ip and not self._moveit_manager.launch_for_gripper(new_gripper, robot_ip):
                self.get_logger().error("Failed to restart MoveIt with new gripper config")
                return False

        return True

    def _call_vision_moveto(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the VisionMoveTo action server."""
        goal = VisionMoveToAction.Goal()
        goal.tag_id = int(step.get("tag_id", 0))
        goal.timeout = float(step.get("timeout", 10.0))
        goal.poses_json = poses_json

        return self._send_and_wait(
            self._vision_client, goal, "vision_moveto", self._timeouts["vision_moveto"]
        )

    def _call_vision_pickplace(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the VisionPickPlace action server."""
        goal = VisionPickPlaceAction.Goal()
        goal.pick_tag_id = int(step.get("pick_tag_id", 0))
        goal.place_tag_id = int(step.get("place_tag_id", -1))
        goal.gripper = step.get("gripper", "")
        goal.grasp_offset_json = step.get("grasp_offset_json", "")
        goal.place_poses_json = step.get("place_poses_json", "")
        goal.approach_offset = float(step.get("approach_offset", 0.1))
        goal.retreat_offset = float(step.get("retreat_offset", 0.15))

        return self._send_and_wait(
            self._vision_pickplace_client, goal, "vision_pick_place",
            self._timeouts["vision_pick_place"]
        )

    def _call_pipettor(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the Pipettor action server."""
        from std_msgs.msg import ColorRGBA

        goal = PipettorAction.Goal()
        goal.operation = step.get("operation", "")
        goal.volume_pct = float(step.get("volume_pct", 0.0))
        goal.poses_json = poses_json

        # Parse LED color if provided
        if "led_color" in step:
            led = step["led_color"]
            goal.led_color = ColorRGBA()
            goal.led_color.r = float(led.get("r", 0.0))
            goal.led_color.g = float(led.get("g", 0.0))
            goal.led_color.b = float(led.get("b", 0.0))
            goal.led_color.a = 1.0

        return self._send_and_wait(
            self._pipettor_client, goal, "pipettor", self._timeouts["pipettor"]
        )

    def _update_feedback(
        self,
        feedback: MTCExecution.Feedback,
        goal_handle: ServerGoalHandle,
        current_step: int,
        total_steps: int,
        task_type: str,
    ):
        """Publish progress feedback."""
        feedback.current_step = current_step
        feedback.current_action = task_type
        feedback.progress_percentage = (current_step / total_steps) * 100.0 if total_steps > 0 else 0.0
        feedback.status_message = "Task completed" if not task_type else f"Executing: {task_type}"
        feedback.current_gripper = self._current_gripper

        goal_handle.publish_feedback(feedback)


def main(args=None):
    """Run the MTC Orchestrator server."""
    rclpy.init(args=args)
    node = MTCOrchestratorServer()

    # Use multithreaded executor for concurrent action client calls
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MTC Orchestrator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
