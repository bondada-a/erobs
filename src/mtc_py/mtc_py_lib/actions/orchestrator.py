"""MTC Orchestrator - coordinates multi-step robot tasks.

Python equivalent of mtc_orchestrator_action_server.cpp.
Receives task scripts (JSON) and dispatches steps to specialized action servers.
Manages MoveIt lifecycle based on gripper configuration.
"""

import json
import threading
from typing import Dict, Any, Optional
from dataclasses import dataclass

from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from mtc_py.action import (
    MTCExecution,
    MoveToAction,
    EndEffectorAction,
    PickPlaceAction,
    ToolExchangeAction,
)

from mtc_py_lib.core.moveit_lifecycle_manager import MoveItLifecycleManager


@dataclass
class ParsedGoal:
    """Validated and parsed goal data."""
    robot_ip: str
    start_gripper: str
    tasks: list
    poses_json: str

    @property
    def task_count(self) -> int:
        return len(self.tasks)


class MTCOrchestratorServer(Node):
    """Action server that coordinates multi-step robot tasks.

    Receives task scripts in JSON format and dispatches individual
    steps to specialized MTC action servers (MoveTo, EndEffector, etc.)
    """

    # Timeouts for each action type (seconds)
    TIMEOUTS = {
        "moveto": 120.0,
        "end_effector": 30.0,
        "pick_and_place": 180.0,
        "tool_exchange": 180.0,
    }

    def __init__(self):
        super().__init__("mtc_orchestrator_py")

        self._executing = False
        self._lock = threading.Lock()
        self._current_gripper = "none"

        # Callback group for concurrent operations
        self._callback_group = ReentrantCallbackGroup()

        # MoveIt lifecycle manager - launches MoveIt based on gripper config
        self._moveit_manager = MoveItLifecycleManager(self)

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
                return self._create_error_result("Server busy")
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

        # Initialize feedback
        feedback.current_gripper = parsed.start_gripper
        self._current_gripper = parsed.start_gripper

        # Step 1: Launch MoveIt for the gripper configuration
        self._update_feedback(
            feedback, goal_handle, 0, parsed.task_count, "Initializing MoveIt"
        )

        if not parsed.robot_ip:
            result.success = False
            result.error_message = "Goal missing required robot_ip"
            goal_handle.abort()
            return result

        if not self._moveit_manager.launch_for_gripper(
            parsed.start_gripper, parsed.robot_ip
        ):
            result.success = False
            result.error_message = "Failed to initialize MoveIt stack"
            goal_handle.abort()
            return result

        # Execute each task
        for i, task in enumerate(parsed.tasks):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                result.success = False
                result.error_message = "Task was canceled"
                result.completed_steps = i
                goal_handle.canceled()
                return result

            task_type = task.get("task_type", "")
            if not task_type:
                result.success = False
                result.error_message = f"Step {i} missing 'task_type' field"
                result.completed_steps = i
                goal_handle.abort()
                return result

            # Update feedback
            self._update_feedback(
                feedback, goal_handle, i + 1, parsed.task_count, task_type
            )

            # Execute step
            if not self._execute_step(task_type, task, parsed.poses_json):
                result.success = False
                result.error_message = f"{task_type} step failed"
                result.completed_steps = i
                goal_handle.abort()
                return result

            result.completed_steps = i + 1

        # Success
        result.success = True
        result.total_steps = parsed.task_count
        result.completed_steps = parsed.task_count
        self._update_feedback(
            feedback, goal_handle, parsed.task_count, parsed.task_count, ""
        )
        goal_handle.succeed()

        self.get_logger().info("Orchestration goal completed successfully")
        return result

    def _parse_goal(
        self, goal: MTCExecution.Goal, result: MTCExecution.Result
    ) -> Optional[ParsedGoal]:
        """Parse and validate the goal JSON.

        Returns:
            ParsedGoal if valid, None if invalid (result populated with error)
        """
        if not goal.full_json:
            result.success = False
            result.error_message = "Goal missing required full_json"
            return None

        try:
            script = json.loads(goal.full_json)
        except json.JSONDecodeError as e:
            result.success = False
            result.error_message = f"Invalid JSON: {e}"
            return None

        # Validate required fields
        if "start_gripper" not in script or not isinstance(script["start_gripper"], str):
            result.success = False
            result.error_message = "Task script missing required 'start_gripper'"
            return None

        if "tasks" not in script or not isinstance(script["tasks"], list):
            result.success = False
            result.error_message = "Task script missing required 'tasks' array"
            return None

        return ParsedGoal(
            robot_ip=goal.robot_ip or "",
            start_gripper=script["start_gripper"],
            tasks=script["tasks"],
            poses_json=json.dumps(script.get("poses", {})),
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
        else:
            self.get_logger().error(f"Unknown task type: {task_type}")
            return False

    def _send_and_wait(
        self, client: ActionClient, goal, name: str, timeout: float
    ) -> bool:
        """Send a goal to an action client and wait for result."""
        import time

        # Wait for server
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"{name} action server unavailable")
            return False

        # Send goal and wait for acceptance
        send_future = client.send_goal_async(goal)

        # Wait for goal acceptance with timeout
        start = time.time()
        while not send_future.done():
            if time.time() - start > 10.0:  # 10s timeout for goal acceptance
                self.get_logger().error(f"{name} goal acceptance timed out")
                return False
            time.sleep(0.05)

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"{name} goal was rejected")
            return False

        # Wait for result with timeout
        result_future = goal_handle.get_result_async()

        start = time.time()
        while not result_future.done():
            if time.time() - start > timeout:
                self.get_logger().error(f"{name} timed out after {timeout}s")
                goal_handle.cancel_goal_async()
                return False
            time.sleep(0.1)

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
            self._moveto_client, goal, "moveto", self.TIMEOUTS["moveto"]
        )

    def _call_endeffector(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the EndEffector action server."""
        goal = EndEffectorAction.Goal()
        goal.end_effector_type = step.get("end_effector_type", "")
        goal.end_effector_action = step.get("end_effector_action", "")
        goal.poses_json = poses_json

        return self._send_and_wait(
            self._endeffector_client, goal, "end_effector", self.TIMEOUTS["end_effector"]
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
            self._pickplace_client, goal, "pick_place", self.TIMEOUTS["pick_and_place"]
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
            self._toolexchange_client, goal, "tool_exchange", self.TIMEOUTS["tool_exchange"]
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

            # Get robot_ip from the manager's current state
            # We need to relaunch MoveIt with the new gripper config
            robot_ip = self._moveit_manager._robot_ip if hasattr(self._moveit_manager, '_robot_ip') else ""
            if robot_ip and not self._moveit_manager.launch_for_gripper(new_gripper, robot_ip):
                self.get_logger().error("Failed to restart MoveIt with new gripper config")
                return False
        else:
            self._current_gripper = new_gripper

        return True

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

    def _create_error_result(self, error_message: str) -> MTCExecution.Result:
        """Create an error result."""
        result = MTCExecution.Result()
        result.success = False
        result.error_message = error_message
        return result
