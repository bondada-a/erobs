#!/usr/bin/env python3
"""MTC Orchestrator - coordinates multi-step robot tasks.

Receives task scripts (JSON) and dispatches steps to specialized action servers.
Manages MoveIt lifecycle based on gripper configuration.

Supports beamline-agnostic deployment via beamline configuration files.

Batching optimization: Consecutive simple tasks (moveto, end_effector) are
grouped into a single MTC Task with multiple stages, reducing planning
overhead (~1.5s per task saved).
"""

import json
import math
import threading
import time
from typing import Dict, Any, List

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy

from beambot_interfaces.action import (
    MTCExecution,
    MoveToAction,
    EndEffectorAction,
    PickSampleAction,
    PlaceSampleAction,
    ToolExchangeAction,
    VisionMoveToAction,
    VisionScanAction,
    PipettorAction,
)
from controller_manager_msgs.srv import ListControllers, SwitchController
from std_srvs.srv import Trigger
from std_msgs.msg import String

from beambot.core.moveit_lifecycle_manager import MoveItLifecycleManager
from beambot.core.vacuum_monitor import VacuumMonitor
from beambot.stages.move_to_stages import MoveToStages
from beambot.stages.end_effector_stages import EndEffectorStages
from beambot.batch_planner import group_into_batches


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
        "tool_exchange": 180.0,
        "vision_moveto": 60.0,
        "vision_scan": 180.0,  # Batch scan: 3 positions × 3 scans
        "pick_sample": 180.0,
        "place_sample": 180.0,
        "pipettor": 60.0,
    }

    def __init__(self):
        super().__init__("beambot_orchestrator")

        self._executing = False
        self._lock = threading.Lock()
        self._current_gripper = "unknown"
        self._last_error = ""  # Error from last failed action/batch
        self._last_result = None  # Full result from last _send_and_wait
        self._last_detected_position = None  # [x, y, z] from detect_only vision
        self._last_detected_orientation = None  # [x, y, z, w] from detect_only vision

        # Pause/Resume state
        self._pause_requested = False
        self._is_paused = False
        self._pause_event = threading.Event()
        self._pause_event.set()  # Start in "go" state (not blocked)

        # Load beamline configuration (single source of truth)
        self.declare_parameter("beamline_config", "config/default_beamline.yaml")
        self.declare_parameter("use_mock_hardware", False)
        self.declare_parameter("enable_batching", True)
        self.declare_parameter("cup_profile", "")  # Override cup profile (empty = use beamline config default)

        config_file = self.get_parameter("beamline_config").value
        self._use_mock_hardware = self.get_parameter("use_mock_hardware").value
        self._enable_batching = self.get_parameter("enable_batching").value

        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        self._grippers = config["grippers"]  # Dict of gripper_name -> {moveit_package, tool_voltage, gripper_group, states}
        self._robot_ip = config["robot"]["ip"]  # Single source: config file
        self._arm_group = config.get("robot", {}).get("arm_group", "ur_manipulator")  # For batched execution
        self.get_logger().info(f"Loaded beamline: {config['beamline']} (robot: {self._robot_ip})")
        if self._use_mock_hardware:
            self.get_logger().info("Using FAKE HARDWARE (simulation mode)")
        if not self._enable_batching:
            self.get_logger().info("Batching DISABLED - each task executes via action server")

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
        self._moveit_manager = MoveItLifecycleManager(
            self, self._grippers, self._robot_ip, self._callback_group,
            use_mock_hardware=self._use_mock_hardware
        )

        # Create action server
        self._action_server = ActionServer(
            self,
            MTCExecution,
            "beambot_execution",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        # Create action clients for specialized servers
        self._moveto_client = ActionClient(
            self, MoveToAction, "beambot_moveto",
            callback_group=self._callback_group
        )
        self._endeffector_client = ActionClient(
            self, EndEffectorAction, "beambot_endeffector",
            callback_group=self._callback_group
        )
        self._toolexchange_client = ActionClient(
            self, ToolExchangeAction, "beambot_toolexchange",
            callback_group=self._callback_group
        )
        self._vision_client = ActionClient(
            self, VisionMoveToAction, "beambot_vision_moveto",
            callback_group=self._callback_group
        )
        self._vision_scan_client = ActionClient(
            self, VisionScanAction, "beambot_vision_scan",
            callback_group=self._callback_group
        )
        self._pick_sample_client = ActionClient(
            self, PickSampleAction, "beambot_pick_sample",
            callback_group=self._callback_group
        )
        self._place_sample_client = ActionClient(
            self, PlaceSampleAction, "beambot_place_sample",
            callback_group=self._callback_group
        )
        self._pipettor_client = ActionClient(
            self, PipettorAction, "beambot_pipettor",
            callback_group=self._callback_group
        )

        # Vision server TF reset service (called after tool exchange)
        self._vision_reset_tf_client = self.create_client(
            Trigger, "beambot_vision_reset_tf",
            callback_group=self._callback_group
        )

        # Controller manager service clients for recovery
        self._list_controllers_client = self.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
            callback_group=self._callback_group
        )
        self._switch_controller_client = self.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
            callback_group=self._callback_group
        )

        # Controllers that must be active for robot motion
        self._base_controllers = ["scaled_joint_trajectory_controller"]
        self._required_controllers = list(self._base_controllers)

        # Pause/Resume services
        self._pause_service = self.create_service(
            Trigger, 'beambot/pause', self._pause_callback,
            callback_group=self._callback_group
        )
        self._resume_service = self.create_service(
            Trigger, 'beambot/resume', self._resume_callback,
            callback_group=self._callback_group
        )

        # Execution state publisher
        self._state_publisher = self.create_publisher(
            String, 'beambot/execution_state', 10
        )

        # Current gripper publisher (latched so late subscribers get last value)
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._gripper_publisher = self.create_publisher(
            String, 'beambot/current_gripper', latched_qos
        )
        self._publish_gripper(self._current_gripper)

        # Vacuum monitoring (ePick grasp verification)
        self._vacuum = VacuumMonitor(self, self._grippers, self._callback_group)

        self.get_logger().info("MTC Orchestrator (Python) started on 'beambot_execution'")
        self.get_logger().info("Pause/Resume services available: beambot/pause, beambot/resume")

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Handle incoming goal requests."""
        with self._lock:
            if self._executing:
                self.get_logger().warning("Goal rejected: another task is executing")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Handle cancel requests."""
        self.get_logger().info("Cancel request received - will stop after current task")
        return CancelResponse.ACCEPT

    def _pause_callback(self, request, response):
        """Handle pause service request.

        Sets a flag to pause execution after the current task completes.
        """
        with self._lock:
            if not self._executing:
                response.success = False
                response.message = "No task is currently executing"
                return response

            if self._is_paused:
                response.success = False
                response.message = "Already paused"
                return response

            if self._pause_requested:
                response.success = False
                response.message = "Pause already requested, waiting for current task to complete"
                return response

            self._pause_requested = True
            self._publish_state("COMPLETING_TASK")
            response.success = True
            response.message = "Pause requested - will pause after current task completes"

        self.get_logger().info("Pause requested - will pause after current task")
        return response

    def _resume_callback(self, request, response):
        """Handle resume service request.

        Signals the paused execution to continue.
        """
        with self._lock:
            if not self._is_paused:
                response.success = False
                response.message = "Not currently paused"
                return response

            self._is_paused = False
            self._pause_event.set()  # Unblock the waiting thread
            response.success = True
            response.message = "Resuming execution"

        self.get_logger().info("Resume requested - continuing execution")
        return response

    def _publish_state(self, state: str):
        """Publish current execution state to the state topic.

        States: IDLE, RUNNING, PAUSED, COMPLETING_TASK
        """
        msg = String()
        msg.data = state
        self._state_publisher.publish(msg)

    def _publish_gripper(self, gripper: str):
        """Publish current gripper to latched topic."""
        msg = String()
        msg.data = gripper
        self._gripper_publisher.publish(msg)

    def _update_required_controllers(self):
        """Update required controllers list based on current gripper."""
        gripper_controller = self._grippers.get(
            self._current_gripper, {}
        ).get("controller_name", "")
        self._required_controllers = list(self._base_controllers)
        if gripper_controller:
            self._required_controllers.append(gripper_controller)

    def _handle_pause(
        self,
        feedback: MTCExecution.Feedback,
        goal_handle: ServerGoalHandle,
        current_step: int,
        total_steps: int
    ):
        """Block execution until resumed or cancelled.

        Called when _pause_requested is True. Updates state, publishes feedback,
        and waits for resume signal or cancel request.
        """
        with self._lock:
            self._pause_requested = False
            self._is_paused = True
            self._pause_event.clear()  # Block the event

        self._publish_state("PAUSED")
        self.get_logger().info(f"Paused after step {current_step}/{total_steps}")

        # Update feedback to show paused state
        feedback.status_message = f"PAUSED - completed {current_step}/{total_steps}, waiting for resume"
        goal_handle.publish_feedback(feedback)

        # Wait loop - check for cancel periodically, keep publishing feedback
        while not self._pause_event.wait(timeout=0.5):
            # Check for cancel during pause
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel received while paused")
                with self._lock:
                    self._is_paused = False
                    self._pause_event.set()
                return

            # Keep publishing feedback so client knows we're alive
            goal_handle.publish_feedback(feedback)

        # Resumed
        self.get_logger().info("Execution resumed")
        self._publish_state("RUNNING")

    def _ensure_controllers_active(self) -> bool:
        """Check if required controllers are active and restart them if needed.

        This handles the case where the UR driver's controller_stopper stops
        controllers after a connection drop but doesn't restart them.

        Returns:
            True if all required controllers are active, False on failure
        """
        # Check if service is available
        if not self._list_controllers_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("Controller manager service not available")
            return True  # Proceed anyway, let MoveIt handle the error

        # List controllers
        list_request = ListControllers.Request()
        future = self._list_controllers_client.call_async(list_request)

        # Wait for response (polling to avoid executor conflicts)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 5.0:
                self.get_logger().warning("Timeout listing controllers")
                return True  # Proceed anyway
            time.sleep(0.01)

        list_response = future.result()
        if list_response is None:
            self.get_logger().warning("Failed to list controllers")
            return True  # Proceed anyway

        # Check which required controllers are inactive
        inactive_controllers = []
        for controller in list_response.controller:
            if controller.name in self._required_controllers:
                if controller.state != "active":
                    inactive_controllers.append(controller.name)
                    self.get_logger().warning(
                        f"Controller '{controller.name}' is {controller.state}, needs restart"
                    )

        if not inactive_controllers:
            return True  # All controllers active

        # Try to activate inactive controllers
        self.get_logger().info(f"Activating controllers: {inactive_controllers}")

        switch_request = SwitchController.Request()
        switch_request.activate_controllers = inactive_controllers
        switch_request.deactivate_controllers = []
        switch_request.strictness = SwitchController.Request.BEST_EFFORT
        switch_request.activate_asap = True

        future = self._switch_controller_client.call_async(switch_request)

        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 5.0:
                self.get_logger().error("Timeout activating controllers")
                return False
            time.sleep(0.01)

        switch_response = future.result()
        if switch_response is None or not switch_response.ok:
            self.get_logger().error("Failed to activate controllers")
            return False

        self.get_logger().info("Controllers reactivated successfully")
        return True

    # _group_into_batches extracted to beambot.batch_planner.group_into_batches

    def _execute_batch(
        self,
        batch_tasks: List[Dict[str, Any]],
        poses_json: str
    ) -> bool:
        """Execute a batch of tasks as a single MTC Task.

        Creates one MTC Task, adds stages from each task, then plans and
        executes once. This reduces planning overhead (~1.5s per task saved).

        Args:
            batch_tasks: List of batchable task dictionaries
            poses_json: JSON string with pose definitions

        Returns:
            True if all tasks succeeded, False on any failure
        """
        if not batch_tasks:
            return True

        # Log batch info
        task_types = [t.get("task_type", "?") for t in batch_tasks]
        self.get_logger().info(
            f"Executing batch of {len(batch_tasks)} tasks: {task_types}"
        )

        # Create stage instances (they share MTC node via module-level singleton)
        moveto_stage = MoveToStages(self, self._arm_group)
        endeffector_stage = EndEffectorStages(self, self._arm_group)

        # Create single MTC Task for the batch
        task = moveto_stage.create_task_template(f"Batch ({len(batch_tasks)} tasks)")

        # Add stages from each task
        for i, batch_task in enumerate(batch_tasks):
            task_type = batch_task.get("task_type", "")
            error = None

            if task_type == "moveto":
                goal = self._create_moveto_goal(batch_task, poses_json)
                error = moveto_stage.add_to_task(task, goal)

            elif task_type == "end_effector":
                goal = self._create_endeffector_goal(batch_task)
                error = endeffector_stage.add_to_task(task, goal)

            else:
                self._last_error = f"Unknown batchable type: {task_type}"
                self.get_logger().error(self._last_error)
                return False

            if error is not None:
                self._last_error = f"Batch task {i} ({task_type}): {error}"
                self.get_logger().error(self._last_error)
                return False

        # Plan and execute the entire batch at once
        error = moveto_stage.load_plan_execute(task)
        if error is not None:
            self._last_error = error
            return False
        return True

    def _create_moveto_goal(
        self, step: Dict[str, Any], poses_json: str
    ) -> MoveToAction.Goal:
        """Create a MoveToAction.Goal from task dict."""
        goal = MoveToAction.Goal()
        goal.target = step.get("target", "")
        goal.planning_type = step.get("planning_type", "")
        goal.direction = step.get("direction", "")
        goal.distance = float(step.get("distance", 0.0))
        goal.cartesian_target = [float(v) for v in step.get("cartesian_target", [])]
        goal.frame_id = step.get("frame_id", "base_link")
        goal.poses_json = poses_json
        goal.constraints_json = json.dumps(step["constraints"]) if "constraints" in step else ""
        return goal

    def _create_endeffector_goal(self, step: Dict[str, Any]) -> EndEffectorAction.Goal:
        """Create an EndEffectorAction.Goal from task dict."""
        # Use current gripper if not specified in task
        gripper_type = step.get("end_effector_type", self._current_gripper)
        gripper_config = self._grippers.get(gripper_type, {})

        goal = EndEffectorAction.Goal()
        goal.gripper_group = gripper_config.get("gripper_group", "")
        goal.end_effector_action = step.get("end_effector_action", "")
        return goal

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

        # Reset state for new goal
        self._vacuum.reset()
        self._last_detected_position = None
        self._last_detected_orientation = None

        result = MTCExecution.Result()
        feedback = MTCExecution.Feedback()

        # Parse and validate goal
        parsed = self._parse_goal(goal_handle.request, result)
        if parsed is None:
            goal_handle.abort()
            self._publish_state("IDLE")
            return result

        start_gripper, tasks, poses_json = parsed
        task_count = len(tasks)

        # Initialize gripper state
        self._current_gripper = start_gripper
        self._publish_gripper(start_gripper)
        self._update_required_controllers()

        # Apply cup_profile override if parameter was changed via MCP
        cup_override = self.get_parameter("cup_profile").value
        if cup_override and start_gripper in self._grippers:
            self._grippers[start_gripper]["cup_profile"] = cup_override

        # Step 1: Launch MoveIt for the gripper configuration
        self._update_feedback(
            feedback, goal_handle, 0, task_count, "Initializing MoveIt"
        )

        if not self._moveit_manager.launch_moveit_with_gripper(start_gripper):
            result.error_message = "Failed to initialize MoveIt stack"
            goal_handle.abort()
            self._publish_state("IDLE")
            return result

        # Publish running state before starting task execution
        self._publish_state("RUNNING")

        # Group tasks into batches for optimized execution.
        # Disable batching when ePick is attached — the vacuum watchdog needs
        # step boundaries between every move to detect dropped objects.
        batching_enabled = self._enable_batching and start_gripper != "epick"
        if self._enable_batching and not batching_enabled:
            self.get_logger().info(
                "Batching disabled for ePick — vacuum watchdog needs per-step boundaries"
            )
        batches = group_into_batches(tasks, enabled=batching_enabled)
        self.get_logger().info(
            f"Grouped {task_count} tasks into {len(batches)} batches"
        )

        # Track overall task index for feedback
        completed_tasks = 0

        # Execute each batch
        for batch_type, batch_tasks in batches:
            batch_size = len(batch_tasks)

            # Check for cancellation at batch boundary
            if goal_handle.is_cancel_requested:
                self.get_logger().warning(
                    f"Task cancelled after step {completed_tasks}/{task_count}"
                )
                result.error_message = "Task was canceled"
                result.completed_steps = completed_tasks
                goal_handle.canceled()
                self._publish_state("IDLE")
                return result

            # Check for pause request at batch boundary
            if self._pause_requested:
                self._handle_pause(feedback, goal_handle, completed_tasks, task_count)

                # Check if cancelled DURING pause
                if goal_handle.is_cancel_requested:
                    self.get_logger().warning(
                        f"Task cancelled while paused at step {completed_tasks}/{task_count}"
                    )
                    result.error_message = "Task was cancelled while paused"
                    result.completed_steps = completed_tasks
                    goal_handle.canceled()
                    self._publish_state("IDLE")
                    return result

            # Check if vacuum was lost since last step
            vacuum_error = self._vacuum.check_lost()
            if vacuum_error:
                result.error_message = f"Step {completed_tasks + 1} aborted: {vacuum_error}"
                result.completed_steps = completed_tasks
                goal_handle.abort()
                self._publish_state("IDLE")
                return result

            # Check if MoveIt subprocess is still alive before dispatching
            if not self._moveit_manager.is_moveit_alive():
                exit_info = self._moveit_manager.get_moveit_exit_info()
                result.error_message = f"MoveIt crashed before step {completed_tasks + 1}: {exit_info}"
                self.get_logger().error(result.error_message)
                result.completed_steps = completed_tasks
                goal_handle.abort()
                self._publish_state("IDLE")
                return result

            if batch_type == "batched":
                # Execute batch of batchable tasks as single MTC Task
                batch_desc = ", ".join(t.get("task_type", "?") for t in batch_tasks)
                self._update_feedback(
                    feedback, goal_handle, completed_tasks + 1, task_count,
                    f"batch[{batch_size}]: {batch_desc}"
                )

                # Ensure controllers are active before batch execution
                if not self._ensure_controllers_active():
                    self.get_logger().error("Failed to ensure controllers are active")
                    result.error_message = "Controller activation failed"
                    result.completed_steps = completed_tasks
                    goal_handle.abort()
                    self._publish_state("IDLE")
                    return result

                if not self._execute_batch(batch_tasks, poses_json):
                    result.error_message = f"Batch failed at step {completed_tasks + 1}: {self._last_error}"
                    result.completed_steps = completed_tasks
                    goal_handle.abort()
                    self._publish_state("IDLE")
                    return result

                self._vacuum.update_after_tasks(batch_tasks, self._current_gripper)
                completed_tasks += batch_size

            else:
                # Execute single task via action server (non-batchable)
                task = batch_tasks[0]
                task_type = task.get("task_type", "")

                if not task_type:
                    result.error_message = f"Step {completed_tasks} missing 'task_type' field"
                    result.completed_steps = completed_tasks
                    goal_handle.abort()
                    self._publish_state("IDLE")
                    return result

                self._update_feedback(
                    feedback, goal_handle, completed_tasks + 1, task_count, task_type
                )

                if not self._execute_step(task_type, task, poses_json):
                    result.error_message = f"{task_type} failed: {self._last_error}"
                    result.completed_steps = completed_tasks
                    goal_handle.abort()
                    self._publish_state("IDLE")
                    return result

                self._vacuum.update_after_tasks([task], self._current_gripper)
                completed_tasks += 1

            result.completed_steps = completed_tasks

        # Final vacuum check — catch drops during the last step
        vacuum_error = self._vacuum.check_lost()
        if vacuum_error:
            result.error_message = f"Final step aborted: {vacuum_error}"
            result.completed_steps = completed_tasks
            goal_handle.abort()
            self._publish_state("IDLE")
            return result

        # Success
        result.success = True
        result.total_steps = task_count

        # Propagate detected pose from detect_only vision_moveto
        if self._last_detected_position is not None:
            result.detected_position = self._last_detected_position
        if self._last_detected_orientation is not None:
            result.detected_orientation = self._last_detected_orientation

        self._update_feedback(
            feedback, goal_handle, task_count, task_count, ""
        )
        goal_handle.succeed()
        self._publish_state("IDLE")

        self.get_logger().info("Orchestration goal completed successfully")
        return result

    def _parse_goal(self, goal: MTCExecution.Goal, result: MTCExecution.Result):
        """Parse and validate the goal JSON.

        Returns:
            (start_gripper, tasks, poses_json) if valid,
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

        # Validate gripper exists in config
        start_gripper = script["start_gripper"]
        if start_gripper not in self._grippers:
            result.error_message = f"Unknown gripper: {start_gripper} (available: {', '.join(self._grippers.keys())})"
            return None

        return (
            start_gripper,
            script["tasks"],
            json.dumps(script.get("poses", {})),
        )

    # ------------------------------------------------------------------
    def _execute_step(
        self, task_type: str, step: Dict[str, Any], poses_json: str
    ) -> bool:
        """Execute a single step by dispatching to the appropriate action server."""
        # Ensure controllers are active before executing (handles socket drop recovery)
        if not self._ensure_controllers_active():
            self._last_error = "Controller activation failed before step execution"
            self.get_logger().error(self._last_error)
            return False

        self.get_logger().info(f"Executing step: {task_type}")

        if task_type == "moveto":
            return self._call_moveto(step, poses_json)
        elif task_type == "end_effector":
            return self._call_endeffector(step, poses_json)
        elif task_type == "tool_exchange":
            return self._handle_tool_exchange(step, poses_json)
        elif task_type == "vision_moveto":
            return self._call_vision_moveto(step, poses_json)
        elif task_type == "vision_scan":
            return self._call_vision_scan(step, poses_json)
        elif task_type == "pick_sample":
            return self._call_pick_sample(step, poses_json)
        elif task_type == "place_sample":
            return self._call_place_sample(step, poses_json)
        elif task_type == "pipettor":
            return self._call_pipettor(step, poses_json)
        else:
            self._last_error = f"Unknown task type: '{task_type}'"
            self.get_logger().error(self._last_error)
            return False

    def _send_and_wait(
        self, client: ActionClient, goal, name: str, timeout: float
    ) -> bool:
        """Send a goal to an action client and wait for result.

        Uses polling instead of spin_until_future_complete to avoid
        executor conflicts (this callback is already being spun by
        the MultiThreadedExecutor).

        On failure, stores error_message in self._last_error for propagation
        to the orchestrator result.
        """
        self._last_error = ""

        # Wait for server
        if not client.wait_for_server(timeout_sec=5.0):
            self._last_error = f"TIMEOUT: {name} action server unavailable (waited 5s)"
            self.get_logger().error(self._last_error)
            return False

        # Send goal and wait for acceptance
        send_future = client.send_goal_async(goal)

        # Wait for goal acceptance (10s timeout) - poll instead of spin
        start_time = time.time()
        while not send_future.done():
            if time.time() - start_time > 10.0:
                self._last_error = f"TIMEOUT: {name} goal acceptance timed out (10s)"
                self.get_logger().error(self._last_error)
                return False
            time.sleep(0.01)

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self._last_error = f"{name} goal was rejected by action server"
            self.get_logger().error(self._last_error)
            return False

        # Wait for result with timeout - poll instead of spin
        result_future = goal_handle.get_result_async()

        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > timeout:
                self._last_error = f"TIMEOUT: {name} timed out after {timeout}s"
                self.get_logger().error(self._last_error)
                goal_handle.cancel_goal_async()
                return False
            time.sleep(0.01)

        result = result_future.result()
        self._last_result = result.result
        if not result.result.success:
            # Capture the real error_message from the action server
            self._last_error = getattr(result.result, 'error_message', '') or f"{name} failed (no details)"
            self.get_logger().error(f"{name} failed: {self._last_error}")
        return result.result.success

    def _call_moveto(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the MoveTo action server."""
        goal = self._create_moveto_goal(step, poses_json)
        return self._send_and_wait(
            self._moveto_client, goal, "moveto", self._timeouts["moveto"]
        )

    def _call_endeffector(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the EndEffector action server."""
        goal = self._create_endeffector_goal(step)
        return self._send_and_wait(
            self._endeffector_client, goal, "end_effector", self._timeouts["end_effector"]
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

    # Gripper name → IK tip frame mapping
    _GRIPPER_IK_FRAMES = {
        "epick": "epick_tip",
        "hande": "robotiq_hande_end",
        "pipettor": "pipette_tip_link",
        "2fg7": "2fg7_tip",
        "none": "flange",
    }

    def _gripper_ik_frame(self) -> str:
        """Return the IK frame for the currently attached gripper."""
        return self._GRIPPER_IK_FRAMES.get(self._current_gripper, "flange")

    def _set_tool_voltage_via_io(self, voltage: int) -> bool:
        """Set tool voltage via UR driver's set_io service.

        Unlike the raw socket approach, this doesn't stop the external_control
        program — safe to call while the robot is ready for trajectories.
        """
        from ur_msgs.srv import SetIO
        client = self.create_client(
            SetIO, "/io_and_status_controller/set_io",
            callback_group=self._callback_group
        )
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("set_io service not available")
            self.destroy_client(client)
            return False

        request = SetIO.Request()
        request.fun = 4   # FUN_SET_TOOL_VOLTAGE
        request.pin = 0
        request.state = float(voltage)

        future = client.call_async(request)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            time.sleep(0.05)

        self.destroy_client(client)
        if future.done() and future.result().success:
            self.get_logger().info(f"Tool voltage set to {voltage}V via set_io")
            return True
        self.get_logger().error(f"Failed to set tool voltage to {voltage}V")
        return False

    def _reset_vision_tf(self):
        """Call the vision server's TF reset service after tool exchange.

        Clears stale static transforms from the old URDF so
        _detect_current_gripper picks up the correct IK frame.
        """
        if not self._vision_reset_tf_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning(
                "Vision TF reset service not available — "
                "vision server may detect wrong gripper frame"
            )
            return
        future = self._vision_reset_tf_client.call_async(Trigger.Request())
        # Poll for result (can't use spin in callback context)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            time.sleep(0.05)
        if future.done() and future.result().success:
            self.get_logger().info("Vision server TF buffer reset")
        else:
            self.get_logger().warning("Vision TF reset did not complete")

    def _handle_tool_exchange(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Handle tool exchange with gripper state tracking and MoveIt restart.

        Like the C++ version, this restarts MoveIt with the new gripper config
        after a tool exchange operation completes.
        """
        operation = step.get("operation", "")

        # For dock: turn off tool voltage BEFORE the motion so the
        # Quick Changer releases (de-energizes the lock).
        # Uses the UR driver's set_io service (doesn't stop external_control,
        # unlike the raw socket approach via secondary interface).
        if operation == "dock" and not self._use_mock_hardware:
            self.get_logger().info("Setting tool voltage to 0V for dock (QC release)")
            self._set_tool_voltage_via_io(0)
            time.sleep(0.5)  # Wait for QC to release

        # Execute the physical exchange motion
        if not self._call_toolexchange(step, poses_json):
            # _last_error already set by _send_and_wait
            return False

        # Update gripper state based on operation
        operation = step.get("operation", "")
        new_gripper = self._current_gripper

        if operation == "dock":
            new_gripper = "none"
        elif operation == "load":
            new_gripper = step.get("gripper", self._current_gripper)
            # Validate gripper exists
            if new_gripper not in self._grippers:
                self._last_error = (
                    f"Unknown gripper '{new_gripper}' in tool_exchange "
                    f"(available: {', '.join(self._grippers.keys())})"
                )
                self.get_logger().error(self._last_error)
                return False

        # If gripper changed, restart MoveIt with new configuration
        if new_gripper != self._current_gripper:
            self.get_logger().info(
                f"Gripper changed: {self._current_gripper} → {new_gripper}, restarting MoveIt"
            )
            self._current_gripper = new_gripper
            self._publish_gripper(new_gripper)
            self._update_required_controllers()

            if not self._moveit_manager.launch_moveit_with_gripper(new_gripper):
                self._last_error = (
                    f"Failed to restart MoveIt after tool exchange "
                    f"({self._current_gripper} → {new_gripper})"
                )
                self.get_logger().error(self._last_error)
                return False

            # Reset vision server TF buffer so it picks up the new URDF frames
            self._reset_vision_tf()

            # Restore tool voltage for the new gripper (dock sets it to 0V)
            if operation == "load" and not self._use_mock_hardware:
                tool_voltage = self._grippers.get(new_gripper, {}).get("tool_voltage", 0)
                if tool_voltage > 0:
                    self.get_logger().info(
                        f"Restoring tool voltage to {tool_voltage}V for {new_gripper}"
                    )
                    self._set_tool_voltage_via_io(tool_voltage)

        return True

    def _call_vision_moveto(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the VisionMoveTo action server.

        Supports:
        - tag_id: ArUco marker ID (for marker detection)
        - detection_type: "marker" (default), "circle", or "contour"
        - sample_index: Sample number for contour detection (1-indexed)
        - z_offset: Override approach height
        - timeout: Detection timeout
        - settle_time: Seconds to wait before capture for robot to settle (default: 1.0)
        - scan_positions: List of pose keys for multi-position averaging (optional)
        """
        # Wait for robot vibrations to settle BEFORE calling vision action
        # This happens in orchestrator, completely outside MTC and action server
        settle_time = min(float(step.get("settle_time", 1.0)), 10.0)
        if settle_time > 0:
            self.get_logger().info(f"Waiting {settle_time:.1f}s for robot to settle before vision capture...")
            time.sleep(settle_time)
            self.get_logger().info("Settle complete, starting vision action")

        goal = VisionMoveToAction.Goal()
        goal.tag_id = int(step.get("tag_id", 0))
        goal.sample_index = int(step.get("sample_index", 1))
        goal.timeout = float(step.get("timeout", 10.0))
        goal.poses_json = poses_json
        goal.detection_type = step.get("detection_type", "marker")
        goal.z_offset = float(step.get("z_offset", self._moveit_manager.cup_z_offset))
        goal.detect_only = bool(step.get("detect_only", False))
        goal.offset_direction = step.get("offset_direction", "")
        goal.offset_distance = float(step.get("offset_distance", 0.0))
        goal.marker_offset_x = float(step.get("marker_offset_x", 0.0))
        goal.marker_offset_y = float(step.get("marker_offset_y", 0.0))
        goal.marker_offset_z = float(step.get("marker_offset_z", 0.0))
        # Pass current gripper's IK frame to avoid stale TF auto-detection
        goal.ik_frame = self._gripper_ik_frame()

        # Handle multi-position scan mode
        # Task JSON: "scan_positions": ["sample_scan_1", "sample_scan_2", "sample_scan_3"]
        scan_position_keys = step.get("scan_positions", [])
        if scan_position_keys:
            poses = json.loads(poses_json)
            scan_positions_flat = []
            valid_positions = 0

            for key in scan_position_keys:
                if key in poses:
                    # Convert degrees to radians (poses in JSON are in degrees)
                    joints_deg = poses[key]
                    joints_rad = [math.radians(j) for j in joints_deg]
                    scan_positions_flat.extend(joints_rad)
                    valid_positions += 1
                else:
                    self.get_logger().warning(
                        f"Scan position '{key}' not found in poses, skipping"
                    )

            if valid_positions > 0:
                goal.scan_positions_flat = scan_positions_flat
                goal.num_scan_positions = valid_positions
                self.get_logger().info(
                    f"Multi-position mode: {valid_positions} scan positions configured"
                )

        success = self._send_and_wait(
            self._vision_client, goal, "vision_moveto", self._timeouts["vision_moveto"]
        )

        # For detect_only, store detected pose for use by subsequent steps
        if success and goal.detect_only and self._last_result is not None:
            pos = list(self._last_result.detected_position)
            ori = list(self._last_result.detected_orientation)
            if len(pos) == 3:
                self._last_detected_position = pos
                self._last_detected_orientation = ori if len(ori) == 4 else None
                self.get_logger().info(
                    f"Stored detected pose: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]"
                )

        return success

    def _call_vision_scan(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the VisionScan action server to batch-scan all markers.

        Scans from multiple positions, detects ALL visible markers at each
        position (with multiple captures per position), averages the poses,
        and caches them. Subsequent vision_moveto calls will use cached poses.

        Task JSON format:
        {
            "task_type": "vision_scan",
            "scan_positions": ["pose_key_1", "pose_key_2", "pose_key_3"],
            "scans_per_position": 3,  // optional, default: 3
            "timeout": 10.0           // optional, per-capture timeout
        }
        """
        goal = VisionScanAction.Goal()
        goal.scans_per_position = int(step.get("scans_per_position", 3))
        goal.timeout = float(step.get("timeout", 10.0))
        goal.poses_json = poses_json

        # Parse scan positions from pose keys
        scan_position_keys = step.get("scan_positions", [])
        if not scan_position_keys:
            self.get_logger().error("vision_scan requires 'scan_positions' list")
            return False

        poses = json.loads(poses_json)
        scan_positions_flat = []
        valid_positions = 0

        for key in scan_position_keys:
            if key in poses:
                # Convert degrees to radians (poses in JSON are in degrees)
                joints_deg = poses[key]
                joints_rad = [math.radians(j) for j in joints_deg]
                scan_positions_flat.extend(joints_rad)
                valid_positions += 1
            else:
                self.get_logger().warning(
                    f"Scan position '{key}' not found in poses, skipping"
                )

        if valid_positions == 0:
            self.get_logger().error("No valid scan positions found")
            return False

        goal.scan_positions_flat = scan_positions_flat
        goal.num_scan_positions = valid_positions

        self.get_logger().info(
            f"VisionScan: {valid_positions} positions × "
            f"{goal.scans_per_position} scans per position"
        )

        return self._send_and_wait(
            self._vision_scan_client, goal, "vision_scan",
            self._timeouts["vision_scan"]
        )

    def _call_pick_sample(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the PickSample action server."""
        # Wait for robot to settle before vision capture
        settle_time = min(float(step.get("settle_time", 1.0)), 10.0)
        if settle_time > 0 and step.get("use_vision", True):
            self.get_logger().info(f"Waiting {settle_time:.1f}s for robot to settle...")
            time.sleep(settle_time)

        gripper_type = step.get("gripper", self._current_gripper)
        gripper_config = self._grippers.get(gripper_type, {})

        goal = PickSampleAction.Goal()
        goal.use_vision = step.get("use_vision", True)
        goal.detection_type = step.get("detection_type", "marker")
        goal.tag_id = int(step.get("tag_id", 0))
        goal.sample_index = int(step.get("sample_index", 1))
        goal.z_offset = float(step.get("z_offset", self._moveit_manager.cup_z_offset))
        goal.scan_pose = step.get("scan_pose", "")
        goal.marker_offset_x = float(step.get("marker_offset_x", 0.0))
        goal.marker_offset_y = float(step.get("marker_offset_y", 0.0))
        goal.marker_offset_z = float(step.get("marker_offset_z", 0.0))
        goal.offset_direction = step.get("offset_direction", "")
        goal.offset_distance = float(step.get("offset_distance", 0.0))
        goal.ik_frame = self._gripper_ik_frame()
        goal.approach_pose = step.get("approach_pose", "")
        goal.target_pose = step.get("target_pose", "")
        goal.gripper_group = gripper_config.get("gripper_group", "")
        goal.gripper_states_json = json.dumps(gripper_config.get("states", {}))
        goal.poses_json = poses_json
        goal.constraints_json = json.dumps(step["constraints"]) if "constraints" in step else ""

        success = self._send_and_wait(
            self._pick_sample_client, goal, "pick_sample",
            self._timeouts["pick_sample"]
        )

        # Store detected position for subsequent steps
        if success and self._last_result is not None:
            pos = list(getattr(self._last_result, 'detected_position', []))
            ori = list(getattr(self._last_result, 'detected_orientation', []))
            if len(pos) == 3:
                self._last_detected_position = pos
                self._last_detected_orientation = ori if len(ori) == 4 else None

            # Check vacuum status from result
            vacuum_ok = getattr(self._last_result, 'vacuum_ok', True)
            if not vacuum_ok:
                self._last_error = (
                    "VACUUM_LOST: ePick reports NO_OBJECT_DETECTED after pick. "
                    "Send vacuum_off then vacuum_on to retry."
                )
                self.get_logger().error(self._last_error)
                return False

            # Arm background vacuum monitor for transport (pick_sample turns
            # vacuum on internally, but VacuumMonitor only tracks end_effector
            # tasks — so we arm it explicitly here)
            if vacuum_ok and self._current_gripper == "epick":
                self._vacuum.armed = True
                self._vacuum.lost = False

        return success

    def _call_place_sample(self, step: Dict[str, Any], poses_json: str) -> bool:
        """Call the PlaceSample action server."""
        settle_time = min(float(step.get("settle_time", 1.0)), 10.0)
        if settle_time > 0 and step.get("use_vision", True):
            self.get_logger().info(f"Waiting {settle_time:.1f}s for robot to settle...")
            time.sleep(settle_time)

        gripper_type = step.get("gripper", self._current_gripper)
        gripper_config = self._grippers.get(gripper_type, {})

        goal = PlaceSampleAction.Goal()
        goal.use_vision = step.get("use_vision", True)
        goal.detection_type = step.get("detection_type", "marker")
        goal.tag_id = int(step.get("tag_id", 0))
        goal.z_offset = float(step.get("z_offset", self._moveit_manager.cup_z_offset))
        goal.scan_pose = step.get("scan_pose", "")
        goal.marker_offset_x = float(step.get("marker_offset_x", 0.0))
        goal.marker_offset_y = float(step.get("marker_offset_y", 0.0))
        goal.marker_offset_z = float(step.get("marker_offset_z", 0.0))
        goal.offset_direction = step.get("offset_direction", "")
        goal.offset_distance = float(step.get("offset_distance", 0.0))
        goal.ik_frame = self._gripper_ik_frame()
        goal.approach_pose = step.get("approach_pose", "")
        goal.target_pose = step.get("target_pose", "")
        goal.gripper_group = gripper_config.get("gripper_group", "")
        goal.gripper_states_json = json.dumps(gripper_config.get("states", {}))
        goal.poses_json = poses_json
        goal.constraints_json = json.dumps(step["constraints"]) if "constraints" in step else ""

        success = self._send_and_wait(
            self._place_sample_client, goal, "place_sample",
            self._timeouts["place_sample"]
        )

        if success and self._last_result is not None:
            pos = list(getattr(self._last_result, 'detected_position', []))
            ori = list(getattr(self._last_result, 'detected_orientation', []))
            if len(pos) == 3:
                self._last_detected_position = pos
                self._last_detected_orientation = ori if len(ori) == 4 else None

        # Disarm vacuum monitor (place_sample turns vacuum off internally)
        if success and self._current_gripper == "epick":
            self._vacuum.armed = False
            self._vacuum.lost = False

        return success

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
