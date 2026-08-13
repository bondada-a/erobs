#!/usr/bin/env python3
"""Coordinate task scripts across MoveIt action servers."""

import json
import math
import threading
import time
from typing import Any

from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from beambot.action_servers.base_action_server import run_server
from beambot_interfaces.action import (
    MTCExecution,
    MoveToAction,
    EndEffectorAction,
    PickSampleAction,
    PlaceSampleAction,
    VisionScanAction,
    VisionTaskAction,
    PipettorAction,
)
from std_srvs.srv import Trigger
from std_msgs.msg import String

from beambot.core.moveit_lifecycle_manager import MoveItLifecycleManager
from beambot.core.vacuum_monitor import VacuumMonitor
from beambot.core.plan_cache import PlanCache
from beambot.core.tool_exchange_manager import ToolExchangeManager
from beambot.stages.move_to_stages import MoveToStages
from beambot.stages.end_effector_stages import EndEffectorStages
from beambot.core.batch_planner import group_into_batches
from beambot.stages.base_stages import wait_for_future
from beambot.core.task_script import parse_task_script


class MTCOrchestratorServer(Node):
    """Coordinate and execute multi-step robot tasks."""

    # Defaults are overridable through timeout.<action> ROS parameters.
    DEFAULT_TIMEOUTS = {
        "moveto": 120.0,
        "end_effector": 30.0,
        "tool_exchange": 180.0,
        "vision_moveto": 60.0,
        "vision_task": 60.0,  # Unified pipeline (issue #88); matches vision_moveto
        "vision_scan": 180.0,  # Batch scan: 3 positions × 3 scans
        "pick_sample": 180.0,
        "place_sample": 180.0,
        "pipettor": 60.0,
    }

    def __init__(self):
        super().__init__("beambot_orchestrator")

        self._executing = False
        self._faulted = False
        self._lock = threading.Lock()
        self._current_gripper = "unknown"
        self._last_error = ""  # Error from last failed action/batch
        self._last_result = None  # Full result from last _send_and_wait
        self._last_detected_position = None  # [x, y, z] from detect_only vision
        self._last_detected_orientation = None  # [x, y, z, w] from detect_only vision

        # Cache serialized plans by start state, goal, gripper, and model revision.
        self._plan_cache = PlanCache(self.get_logger())
        # Live MTC Tasks become invalid after MoveIt relaunches.
        self._last_planned_sol_msg = None

        self._pause_requested = False
        self._is_paused = False
        self._pause_event = threading.Event()
        self._pause_event.set()  # Start in "go" state (not blocked)

        # BEAMBOT_BEAMLINE_CONFIG selects the beamline configuration.
        from beambot.config_loader import load_beamline_config, resolve_beamline_path

        self.declare_parameter("use_mock_hardware", False)
        self.declare_parameter("enable_joystick", False)
        self.declare_parameter("enable_batching", True)
        self.declare_parameter(
            "cup_profile", ""
        )  # Override cup profile (empty = use beamline config default)
        self._use_mock_hardware = self.get_parameter("use_mock_hardware").value
        self._enable_batching = self.get_parameter("enable_batching").value

        config, config_file = load_beamline_config()
        self._poses_file = resolve_beamline_path(
            config.get("poses_file", ""), config_file
        )

        self._grippers = config[
            "grippers"
        ]  # Dict of gripper_name -> {moveit_package, tool_voltage, gripper_group, states}
        self._vision_targets = config.get("vision_targets", {})
        self._robot_ip = config["robot"]["ip"]  # Single source: config file
        self._arm_group = config.get("robot", {}).get(
            "arm_group", "ur_arm"
        )  # For batched execution
        self.get_logger().info(
            f"Loaded beamline: {config['beamline']} (robot: {self._robot_ip})"
        )
        if self._use_mock_hardware:
            self.get_logger().info("Using FAKE HARDWARE (simulation mode)")
        if not self._enable_batching:
            self.get_logger().info(
                "Batching DISABLED - each task executes via action server"
            )

        self._timeouts = {}
        for action_type, default_timeout in self.DEFAULT_TIMEOUTS.items():
            param_name = f"timeout.{action_type}"
            self.declare_parameter(param_name, default_timeout)
            self._timeouts[action_type] = self.get_parameter(param_name).value

        self.get_logger().info(f"Timeouts configured: {self._timeouts}")

        self._callback_group = ReentrantCallbackGroup()

        self._moveit_manager = MoveItLifecycleManager(
            self,
            self._grippers,
            self._robot_ip,
            self._callback_group,
            use_mock_hardware=self._use_mock_hardware,
            enable_joystick=self.get_parameter("enable_joystick").value,
        )

        self._action_server = ActionServer(
            self,
            MTCExecution,
            "beambot_execution",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self._moveto_client = ActionClient(
            self, MoveToAction, "beambot_moveto", callback_group=self._callback_group
        )
        self._endeffector_client = ActionClient(
            self,
            EndEffectorAction,
            "beambot_endeffector",
            callback_group=self._callback_group,
        )
        # Legacy vision_moveto tasks use the unified vision server.
        self._vision_task_client = ActionClient(
            self,
            VisionTaskAction,
            "beambot_vision_task",
            callback_group=self._callback_group,
        )
        self._vision_scan_client = ActionClient(
            self,
            VisionScanAction,
            "beambot_vision_scan",
            callback_group=self._callback_group,
        )
        self._pick_sample_client = ActionClient(
            self,
            PickSampleAction,
            "beambot_pick_sample",
            callback_group=self._callback_group,
        )
        self._place_sample_client = ActionClient(
            self,
            PlaceSampleAction,
            "beambot_place_sample",
            callback_group=self._callback_group,
        )
        self._pipettor_client = ActionClient(
            self,
            PipettorAction,
            "beambot_pipettor",
            callback_group=self._callback_group,
        )

        self._tool_exchange_manager = ToolExchangeManager(
            self,
            self._grippers,
            self._moveit_manager,
            self._callback_group,
            use_mock_hardware=self._use_mock_hardware,
            send_and_wait=self._send_and_wait,
            on_gripper_changed=self._set_current_gripper,
        )

        self._pause_service = self.create_service(
            Trigger,
            "beambot/pause",
            self._pause_callback,
            callback_group=self._callback_group,
        )
        self._resume_service = self.create_service(
            Trigger,
            "beambot/resume",
            self._resume_callback,
            callback_group=self._callback_group,
        )

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._state_publisher = self.create_publisher(
            String, "beambot/execution_state", latched_qos
        )
        self._publish_state("IDLE")

        # Latched for late subscribers.
        self._gripper_publisher = self.create_publisher(
            String, "beambot/current_gripper", latched_qos
        )
        self._publish_gripper(self._current_gripper)

        self._vacuum = VacuumMonitor(self, self._grippers, self._callback_group)

        self.get_logger().info(
            "MTC Orchestrator (Python) started on 'beambot_execution'"
        )
        self.get_logger().info(
            "Pause/Resume services available: beambot/pause, beambot/resume"
        )

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Handle incoming goal requests."""
        with self._lock:
            if self._faulted:
                self.get_logger().error("Goal rejected: orchestrator is faulted")
                return GoalResponse.REJECT
            if self._executing:
                self.get_logger().warning("Goal rejected: another task is executing")
                return GoalResponse.REJECT
            self._executing = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Handle cancel requests."""
        self.get_logger().info(
            "Cancel request received - will stop after active execution unit"
        )
        self._publish_state("CANCELING")
        return CancelResponse.ACCEPT

    def _pause_callback(self, request, response):
        """Request a pause after the active execution unit."""
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
                response.message = (
                    "Pause already requested, waiting for current task to complete"
                )
                return response

            self._pause_requested = True
            self._publish_state("COMPLETING_TASK")
            response.success = True
            response.message = (
                "Pause requested - will pause after current task completes"
            )

        self.get_logger().info("Pause requested - will pause after current task")
        return response

    def _resume_callback(self, request, response):
        """Resume paused execution."""
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
        """Publish execution state."""
        msg = String()
        msg.data = state
        self._state_publisher.publish(msg)

    def _publish_gripper(self, gripper: str):
        """Publish current gripper to latched topic."""
        msg = String()
        msg.data = gripper
        self._gripper_publisher.publish(msg)

    def _set_current_gripper(self, gripper: str):
        self._current_gripper = gripper
        self._publish_gripper(gripper)

    def _handle_pause(
        self,
        feedback: MTCExecution.Feedback,
        goal_handle: ServerGoalHandle,
        completed_steps: int,
        total_steps: int,
        active_step: int | None = None,
    ):
        """Wait for resume or cancellation."""
        with self._lock:
            self._pause_requested = False
            self._is_paused = True
            self._pause_event.clear()  # Block the event

        self._publish_state("PAUSED")
        if active_step is None:
            self.get_logger().info(f"Paused after step {completed_steps}/{total_steps}")
        else:
            self.get_logger().info(
                f"Paused at step {active_step}/{total_steps}; "
                f"completed {completed_steps}/{total_steps}"
            )

        if active_step is not None:
            feedback.current_step = active_step
        feedback.progress_percentage = (
            completed_steps / total_steps * 100.0 if total_steps else 0.0
        )
        feedback.status_message = (
            f"PAUSED - completed {completed_steps}/{total_steps}, waiting for resume"
        )
        goal_handle.publish_feedback(feedback)

        # Poll for cancellation while paused.
        while not self._pause_event.wait(timeout=0.5):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel received while paused")
                with self._lock:
                    self._is_paused = False
                    self._pause_event.set()
                return

            goal_handle.publish_feedback(feedback)

        self.get_logger().info("Execution resumed")
        self._publish_state("RUNNING")

    def _execute_batch(
        self,
        batch_tasks: list[dict[str, Any]],
        poses_json: str,
        dry_run: bool = False,
        cached_plan: dict | None = None,
    ) -> bool:
        """Plan or execute one batch; cached plans bypass planning."""
        if not batch_tasks:
            return True

        task_types = [t.get("task_type", "?") for t in batch_tasks]
        self.get_logger().info(
            f"{'Replaying cached plan for' if cached_plan else 'Executing'} "
            f"batch of {len(batch_tasks)} tasks: {task_types}"
        )

        # Stages share the module-level MTC node.
        moveto_stage = MoveToStages(self, self._arm_group)
        endeffector_stage = EndEffectorStages(self, self._arm_group)

        # Replan after terminal replay failures. A timeout may leave motion active,
        # so never dispatch a second trajectory after REPLAY_TIMEOUT.
        # ponytail: replay skips live-scene collision checks; add isPathValid for
        # dynamic scenes (MTC #198).
        if cached_plan is not None:
            error = moveto_stage.execute_solution_msg(
                cached_plan["sol_msg"], is_replay=True
            )
            if error is None:
                return True
            if error.startswith("REPLAY_TIMEOUT"):
                self._last_error = error
                return False
            self.get_logger().warning(
                f"Cached replay failed ({error}); re-planning fresh"
            )
        task = moveto_stage.create_task_template(f"Batch ({len(batch_tasks)} tasks)")

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

        if dry_run:
            error = moveto_stage.init_and_plan(task, dry_run=True)
            if error is not None:
                self._last_error = error
                return False
            # Cache the serialized solution; live Tasks cannot survive relaunches.
            self._last_planned_sol_msg = moveto_stage.last_sol_msg
            return True

        error = moveto_stage.load_plan_execute(task)
        if error is not None:
            self._last_error = error
            return False
        # Save the fresh solution for later replay.
        self._last_planned_sol_msg = moveto_stage.last_sol_msg
        return True

    def _create_moveto_goal(
        self, step: dict[str, Any], poses_json: str
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
        goal.constraints_json = (
            json.dumps(step["constraints"]) if "constraints" in step else ""
        )
        return goal

    def _create_endeffector_goal(self, step: dict[str, Any]) -> EndEffectorAction.Goal:
        """Create an EndEffectorAction.Goal from task dict."""
        gripper_type = step.get("end_effector_type", self._current_gripper)
        gripper_config = self._grippers.get(gripper_type, {})

        goal = EndEffectorAction.Goal()
        goal.gripper_group = gripper_config.get("gripper_group", "")
        goal.end_effector_action = step.get("end_effector_action", "")
        return goal

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the orchestration goal."""
        try:
            return self._execute(goal_handle)
        except Exception:
            self._faulted = True
            raise
        finally:
            with self._lock:
                if not self._faulted:
                    self._executing = False
            self._publish_state("FAULTED" if self._faulted else "IDLE")

    def _execute(self, goal_handle: ServerGoalHandle) -> MTCExecution.Result:
        """Main execution logic."""
        self.get_logger().info("Executing orchestration goal")

        self._vacuum.reset()
        self._last_detected_position = None
        self._last_detected_orientation = None

        result = MTCExecution.Result()
        feedback = MTCExecution.Feedback()

        try:
            parsed = parse_task_script(
                goal_handle.request.full_json,
                dry_run=bool(getattr(goal_handle.request, "dry_run", False)),
                grippers=self._grippers,
                poses_file=self._poses_file,
                vision_targets=self._vision_targets,
                on_info=self.get_logger().info,
                on_warning=self.get_logger().warning,
            )
        except ValueError as error:
            result.error_message = str(error)
            goal_handle.abort()
            return result

        start_gripper, tasks, poses_json, dry_run = parsed
        task_count = len(tasks)
        result.total_steps = task_count
        if dry_run:
            self.get_logger().info(
                f"DRY-RUN preview enabled — planning {task_count} step(s) "
                f"without moving the robot"
            )

        cached_plan_for_replay: dict | None = None

        self._set_current_gripper(start_gripper)

        # Avoid mutating shared beamline config; empty override uses YAML default.
        self._moveit_manager.cup_override = (
            self.get_parameter("cup_profile").value or ""
        )

        # Start MoveIt with the requested gripper.
        self._update_feedback(
            feedback, goal_handle, 0, task_count, "Initializing MoveIt"
        )

        if not self._moveit_manager.launch_moveit_with_gripper(start_gripper):
            result.error_message = "Failed to initialize MoveIt stack"
            goal_handle.abort()
            return result

        # Model revision prevents replay across collision-geometry changes.
        goal_key = PlanCache.compute_key(
            goal_handle.request.full_json,
            start_gripper,
            self._moveit_manager.model_revision,
            self._moveit_manager.current_arm_joints(),
        )

        self._publish_state("RUNNING")

        # Keep dry-run and live batch boundaries identical.
        batches = group_into_batches(
            tasks,
            enabled=self._enable_batching,
        )
        self.get_logger().info(
            f"Grouped {task_count} tasks into {len(batches)} batches"
        )

        # Cache only one-batch goals; one key cannot represent multiple starts.
        cache_eligible = len(batches) == 1 and batches[0][0] == "batched"
        if cache_eligible and not dry_run:
            cached_plan_for_replay = self._plan_cache.get(goal_key)
            if cached_plan_for_replay is not None:
                self.get_logger().info(
                    "Trajectory cache hit — replaying stored plan without re-planning"
                )

        completed_tasks = 0

        for batch_type, batch_tasks in batches:
            batch_size = len(batch_tasks)

            if goal_handle.is_cancel_requested:
                self.get_logger().warning(
                    f"Task cancelled after step {completed_tasks}/{task_count}"
                )
                result.error_message = "Task was canceled"
                result.completed_steps = completed_tasks
                goal_handle.canceled()
                return result

            if self._pause_requested:
                self._handle_pause(feedback, goal_handle, completed_tasks, task_count)

                if goal_handle.is_cancel_requested:
                    self.get_logger().warning(
                        f"Task cancelled while paused at step {completed_tasks}/{task_count}"
                    )
                    result.error_message = "Task was cancelled while paused"
                    result.completed_steps = completed_tasks
                    goal_handle.canceled()
                    return result

            # Vacuum loss is telemetry-only; abort logic remains disabled.
            # if not dry_run:
            #     vacuum_error = self._vacuum.check_lost()
            #     if vacuum_error:
            #         result.error_message = f"Step {completed_tasks + 1} aborted: {vacuum_error}"
            #         result.completed_steps = completed_tasks
            #         goal_handle.abort()
            #         self._publish_state("IDLE")
            #         return result

            # Verify MoveIt before dispatch.
            if not self._moveit_manager.is_moveit_alive():
                exit_info = self._moveit_manager.get_moveit_exit_info()
                result.error_message = (
                    f"MoveIt crashed before step {completed_tasks + 1}: {exit_info}"
                )
                self.get_logger().error(result.error_message)
                result.completed_steps = completed_tasks
                goal_handle.abort()
                return result

            if batch_type == "batched":
                batch_desc = ", ".join(t.get("task_type", "?") for t in batch_tasks)
                self._update_feedback(
                    feedback,
                    goal_handle,
                    completed_tasks + 1,
                    task_count,
                    f"batch[{batch_size}]: {batch_desc}",
                )

                # UR launch infrastructure owns controller activation. Previous
                # orchestrator activation raced the spawner.

                self._last_planned_sol_msg = None
                ok = self._execute_batch(
                    batch_tasks,
                    poses_json,
                    dry_run=dry_run,
                    cached_plan=cached_plan_for_replay,
                )
                if not ok:
                    if goal_handle.is_cancel_requested or self._last_error.startswith(
                        ("TIMEOUT:", "REPLAY_TIMEOUT:")
                    ):
                        self._faulted = True
                    result.error_message = f"Batch failed at step {completed_tasks + 1}: {self._last_error}"
                    result.completed_steps = completed_tasks
                    goal_handle.abort()
                    return result

                # Store fresh single-batch plans. Replays leave
                # _last_planned_sol_msg unset, avoiding redundant writes.
                if cache_eligible and self._last_planned_sol_msg is not None:
                    self._plan_cache.store(
                        goal_key, self._last_planned_sol_msg, self._current_gripper
                    )
                    self.get_logger().info("Trajectory cached for replay")
                    self._last_planned_sol_msg = None

                if not dry_run:
                    self._vacuum.update_after_tasks(batch_tasks, self._current_gripper)
                completed_tasks += batch_size

            else:
                task = batch_tasks[0]
                task_type = task.get("task_type", "")

                if not task_type:
                    result.error_message = (
                        f"Step {completed_tasks} missing 'task_type' field"
                    )
                    result.completed_steps = completed_tasks
                    goal_handle.abort()
                    return result

                # With batching disabled, route dry runs through planning to avoid
                # hardware motion. Single-task dry runs are not cached.
                if dry_run:
                    self._update_feedback(
                        feedback,
                        goal_handle,
                        completed_tasks + 1,
                        task_count,
                        task_type,
                    )
                    self._last_planned_sol_msg = None
                    if not self._execute_batch([task], poses_json, dry_run=True):
                        result.error_message = (
                            f"{task_type} preview failed: {self._last_error}"
                        )
                        result.completed_steps = completed_tasks
                        goal_handle.abort()
                        return result
                    completed_tasks += 1
                    result.completed_steps = completed_tasks
                    if goal_handle.is_cancel_requested:
                        result.error_message = "Task was canceled"
                        goal_handle.canceled()
                        return result
                    continue

                self._update_feedback(
                    feedback, goal_handle, completed_tasks + 1, task_count, task_type
                )

                if task_type == "pause":
                    self.get_logger().info(
                        f"Pause step {completed_tasks + 1}/{task_count}: entering paused state"
                    )
                    self._handle_pause(
                        feedback,
                        goal_handle,
                        completed_tasks,
                        task_count,
                        active_step=completed_tasks + 1,
                    )

                    if goal_handle.is_cancel_requested:
                        self.get_logger().warning(
                            f"Task cancelled while paused at step {completed_tasks + 1}/{task_count}"
                        )
                        result.error_message = "Task was cancelled while paused"
                        result.completed_steps = completed_tasks
                        goal_handle.canceled()
                        return result

                    completed_tasks += 1
                    result.completed_steps = completed_tasks
                    if completed_tasks < task_count:
                        self.get_logger().info(
                            f"Resumed from pause, continuing with step "
                            f"{completed_tasks + 1}/{task_count}"
                        )
                    else:
                        self.get_logger().info(
                            f"Resumed from final pause step {completed_tasks}/{task_count}"
                        )
                    continue

                if not self._execute_step(task_type, task, poses_json):
                    if (
                        goal_handle.is_cancel_requested
                        or self._faulted
                        or self._last_error.startswith(("TIMEOUT:", "REPLAY_TIMEOUT:"))
                    ):
                        self._faulted = True
                    result.error_message = f"{task_type} failed: {self._last_error}"
                    result.completed_steps = completed_tasks
                    goal_handle.abort()
                    return result

                self._vacuum.update_after_tasks([task], self._current_gripper)
                completed_tasks += 1

            result.completed_steps = completed_tasks

            # Catch cancellation during the final execution unit.
            if goal_handle.is_cancel_requested:
                self.get_logger().warning(
                    f"Task cancelled after step {completed_tasks}/{task_count}"
                )
                result.error_message = "Task was canceled"
                goal_handle.canceled()
                return result

        # Final vacuum loss does not change successful completion.
        # if not dry_run:
        #     vacuum_error = self._vacuum.check_lost()
        #     if vacuum_error:
        #         result.error_message = f"Final step aborted: {vacuum_error}"
        #         result.completed_steps = completed_tasks
        #         goal_handle.abort()
        #         self._publish_state("IDLE")
        #         return result

        result.success = True

        if self._last_detected_position is not None:
            result.detected_position = self._last_detected_position
        if self._last_detected_orientation is not None:
            result.detected_orientation = self._last_detected_orientation

        self._update_feedback(feedback, goal_handle, task_count, task_count, "")
        goal_handle.succeed()
        self.get_logger().info("Orchestration goal completed successfully")
        return result

    def _execute_step(
        self, task_type: str, step: dict[str, Any], poses_json: str
    ) -> bool:
        """Execute a single step by dispatching to the appropriate action server."""
        # UR launch infrastructure owns controller activation.

        self.get_logger().info(f"Executing step: {task_type}")

        if task_type == "moveto":
            return self._call_moveto(step, poses_json)
        elif task_type == "end_effector":
            return self._call_endeffector(step, poses_json)
        elif task_type == "tool_exchange":
            return self._handle_tool_exchange(step, poses_json)
        elif task_type in self._VISION_TASK_TYPES:
            return self._call_vision_task(task_type, step, poses_json)
        elif task_type == "vision_scan":
            return self._call_vision_scan(step, poses_json)
        elif task_type == "pipettor":
            return self._call_pipettor(step, poses_json)
        else:
            self._last_error = f"Unknown task type: '{task_type}'"
            self.get_logger().error(self._last_error)
            return False

    def _send_and_wait(
        self, client: ActionClient, goal, name: str, timeout: float
    ) -> bool:
        """Send a child goal and poll without re-entering the executor."""
        self._last_error = ""

        if hasattr(goal, "robot_model_revision"):
            revision = self._moveit_manager.model_revision
            if not revision:
                self._last_error = f"Cannot send {name}: no active RobotModel revision"
                self.get_logger().error(self._last_error)
                return False
            goal.robot_model_revision = revision

        if not client.wait_for_server(timeout_sec=5.0):
            self._last_error = f"TIMEOUT: {name} action server unavailable (waited 5s)"
            self.get_logger().error(self._last_error)
            return False

        send_future = client.send_goal_async(goal)
        if not wait_for_future(send_future, timeout=10.0):
            self._faulted = True
            self._last_error = f"TIMEOUT: {name} goal acceptance timed out (10s)"
            self.get_logger().error(self._last_error)
            return False

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self._last_error = f"{name} goal was rejected by action server"
            self.get_logger().error(self._last_error)
            return False

        result_future = goal_handle.get_result_async()
        if not wait_for_future(result_future, timeout=timeout):
            self._faulted = True
            self._last_error = f"TIMEOUT: {name} timed out after {timeout}s"
            self.get_logger().error(self._last_error)
            goal_handle.cancel_goal_async()
            return False

        result = result_future.result()
        self._last_result = result.result
        if not result.result.success:
            self._last_error = (
                getattr(result.result, "error_message", "")
                or f"{name} failed (no details)"
            )
            self.get_logger().error(f"{name} failed: {self._last_error}")
        return result.result.success

    def _call_moveto(self, step: dict[str, Any], poses_json: str) -> bool:
        """Call the MoveTo action server."""
        goal = self._create_moveto_goal(step, poses_json)
        return self._send_and_wait(
            self._moveto_client, goal, "moveto", self._timeouts["moveto"]
        )

    def _call_endeffector(self, step: dict[str, Any], poses_json: str) -> bool:
        """Call the EndEffector action server."""
        goal = self._create_endeffector_goal(step)
        return self._send_and_wait(
            self._endeffector_client,
            goal,
            "end_effector",
            self._timeouts["end_effector"],
        )

    def _gripper_ik_frame(self) -> str:
        """Return configured IK tip frame for the current gripper."""
        from beambot.config_loader import gripper_tip_frame

        return gripper_tip_frame(self._current_gripper, default="flange")

    def _gripper_z_offset(self) -> float:
        """Default Z offset for the currently attached gripper (meters)."""
        return float(self._grippers.get(self._current_gripper, {}).get("z_offset", 0.0))

    def _handle_tool_exchange(self, step: dict[str, Any], poses_json: str) -> bool:
        success = self._tool_exchange_manager.exchange(
            step,
            poses_json,
            self._current_gripper,
            self._timeouts["tool_exchange"],
        )
        if not success and self._tool_exchange_manager.last_error:
            self._last_error = self._tool_exchange_manager.last_error
        return success

    def _call_vision_scan(self, step: dict[str, Any], poses_json: str) -> bool:
        """Scan configured poses and cache detected marker poses."""
        goal = VisionScanAction.Goal()
        goal.scans_per_position = int(step.get("scans_per_position", 3))
        goal.timeout = float(step.get("timeout", 10.0))
        goal.poses_json = poses_json

        scan_position_keys = step.get("scan_positions", [])
        if not scan_position_keys:
            self._last_error = "vision_scan requires 'scan_positions' list"
            self.get_logger().error(self._last_error)
            return False

        poses = json.loads(poses_json)
        scan_positions_flat = []
        valid_positions = 0

        for key in scan_position_keys:
            if key in poses:
                # Pose files store joint angles in degrees.
                joints_deg = poses[key]
                joints_rad = [math.radians(j) for j in joints_deg]
                scan_positions_flat.extend(joints_rad)
                valid_positions += 1
            else:
                self.get_logger().warning(
                    f"Scan position '{key}' not found in poses, skipping"
                )

        if valid_positions == 0:
            self._last_error = "No valid scan positions found"
            self.get_logger().error(self._last_error)
            return False

        goal.scan_positions_flat = scan_positions_flat
        goal.num_scan_positions = valid_positions

        self.get_logger().info(
            f"VisionScan: {valid_positions} positions × "
            f"{goal.scans_per_position} scans per position"
        )

        return self._send_and_wait(
            self._vision_scan_client, goal, "vision_scan", self._timeouts["vision_scan"]
        )

    # Legacy task names map to VisionTask defaults; explicit fields override them.
    # Watchdog preserves ePick vacuum state across goals.
    _VISION_PRESETS = {
        "vision_moveto": {"detector": "marker", "goal_computer": "approach_pose"},
        "pick_sample": {
            "detector": "marker",
            "goal_computer": "approach_pose",
            "terminal_action": "grasp",
            "pre_open": True,
            "retreat_from_scan": True,
            "watchdog": "arm",
        },
        "place_sample": {
            "detector": "marker",
            "goal_computer": "approach_pose",
            "terminal_action": "release",
            "retreat_from_scan": True,
            "watchdog": "disarm",
        },
        "pick_spincoater": {
            "detector": "spincoater_sample",
            "goal_computer": "j6_snap",
            "terminal_action": "vacuum_on",
            "scan_pose": "spincoater_scan",
            "default_target_pose": "spincoater_place",
            "forward_distance": 0.003,
        },
        "place_spincoater": {
            "detector": "spincoater_pocket",
            "goal_computer": "j6_snap",
            "terminal_action": "vacuum_off",
            "scan_pose": "spincoater_scan",
            "default_target_pose": "spincoater_place",
            "forward_distance": 0.003,
        },
    }

    # Route canonical and preset names through VisionTask.
    _VISION_TASK_TYPES = frozenset({"vision_task", *_VISION_PRESETS})

    def _call_vision_task(
        self, task_type: str, step: dict[str, Any], poses_json: str
    ) -> bool:
        """Dispatch vision tasks and retain cross-goal pose and vacuum state."""
        preset = self._VISION_PRESETS.get(task_type, {})

        # Non-vision pick/place stays on the sample server.
        if task_type in ("pick_sample", "place_sample") and not step.get(
            "use_vision", True
        ):
            return (
                self._call_pick_sample_hardcoded
                if task_type == "pick_sample"
                else self._call_place_sample_hardcoded
            )(step, poses_json)

        # Explicit fields override preset defaults.
        cfg = {**preset, **step}
        # Canonical detector overrides legacy detection_type.
        if "detector" not in step and "detection_type" in step:
            cfg["detector"] = step["detection_type"]

        # Cap pre-motion settling at 10 seconds.
        settle_time = min(float(cfg.get("settle_time", 1.0)), 10.0)
        if settle_time > 0:
            self.get_logger().info(f"Waiting {settle_time:.1f}s for robot to settle...")
            time.sleep(settle_time)

        goal = self._build_vision_goal(cfg, poses_json)
        success = self._send_and_wait(
            self._vision_task_client, goal, task_type, self._timeouts["vision_task"]
        )

        if success and self._last_result is not None:
            pos = list(getattr(self._last_result, "detected_position", []))
            ori = list(getattr(self._last_result, "detected_orientation", []))
            if len(pos) == 3:
                self._last_detected_position = pos
                self._last_detected_orientation = ori if len(ori) == 4 else None
                self.get_logger().info(
                    f"Stored detected pose: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]"
                )

        # ePick watchdog spans goals, so its state remains in the orchestrator.
        watchdog = preset.get("watchdog", "")
        if success and watchdog and self._current_gripper == "epick":
            if watchdog == "arm" and getattr(self._last_result, "vacuum_ok", True):
                self._vacuum.armed = True
                self._vacuum.lost = False
            elif watchdog == "disarm":
                self._vacuum.armed = False
                self._vacuum.lost = False

        return success

    def _build_vision_goal(self, cfg: dict[str, Any], poses_json: str):
        """Build a VisionTask goal from merged preset and task fields."""
        gripper_config = self._grippers.get(
            cfg.get("gripper", self._current_gripper), {}
        )

        goal = VisionTaskAction.Goal()
        # Selectors. Legacy "detection_type" still maps onto "detector".
        goal.detector = cfg.get("detector", cfg.get("detection_type", "marker"))
        goal.goal_computer = cfg.get("goal_computer", "approach_pose")
        # Detection inputs.
        goal.tag_id = int(cfg.get("tag_id", 0))
        goal.sample_index = int(cfg.get("sample_index", 1))
        goal.timeout = float(cfg.get("timeout", 10.0))
        goal.strategy = cfg.get("strategy", "")
        goal.edge_inset_mm = float(cfg.get("edge_inset_mm", 0.0))
        # Goal-computation inputs.
        goal.z_offset = float(cfg.get("z_offset", self._gripper_z_offset()))
        goal.marker_offset_x = float(cfg.get("marker_offset_x", 0.0))
        goal.marker_offset_y = float(cfg.get("marker_offset_y", 0.0))
        goal.marker_offset_z = float(cfg.get("marker_offset_z", 0.0))
        goal.offset_direction = cfg.get("offset_direction", "")
        goal.offset_distance = float(cfg.get("offset_distance", 0.0))
        # Preserve legacy spincoater target fields.
        goal.target_pose = (
            cfg.get("target_pose")
            or cfg.get("place_pose")
            or cfg.get("pickup_pose")
            or cfg.get("default_target_pose", "")
        )
        goal.k_offset = float(cfg.get("k_offset", 0.0))
        goal.forward_distance = float(cfg.get("forward_distance", 0.0))
        # Execution tail.
        goal.scan_pose = cfg.get("scan_pose", "")
        goal.terminal_action = cfg.get("terminal_action", "")
        goal.pre_open = bool(cfg.get("pre_open", False))
        # Pick/place may retreat to the scan pose.
        if cfg.get("retreat_from_scan"):
            goal.retreat_pose = cfg.get("scan_pose", "")
        else:
            goal.retreat_pose = cfg.get("retreat_pose", "")
        goal.gripper_group = gripper_config.get("gripper_group", "")
        goal.gripper_states_json = json.dumps(gripper_config.get("states", {}))
        # Common.
        goal.ik_frame = self._gripper_ik_frame()
        goal.detect_only = bool(cfg.get("detect_only", False))
        goal.poses_json = poses_json
        goal.constraints_json = (
            json.dumps(cfg["constraints"]) if "constraints" in cfg else ""
        )

        # Flatten multi-position joint poses in radians.
        scan_keys = cfg.get("scan_positions", [])
        if scan_keys:
            poses = json.loads(poses_json) if poses_json else {}
            flat, n = [], 0
            for key in scan_keys:
                if key in poses:
                    flat.extend(math.radians(j) for j in poses[key])
                    n += 1
                else:
                    self.get_logger().warning(
                        f"Scan position '{key}' not found, skipping"
                    )
            if n > 0:
                goal.scan_positions_flat = flat
                goal.num_scan_positions = n
                self.get_logger().info(f"Multi-position mode: {n} scan positions")

        return goal

    def _call_pick_sample_hardcoded(
        self, step: dict[str, Any], poses_json: str
    ) -> bool:
        """Non-vision pick: legacy PickSample server, joint-pose sequence."""
        gripper_config = self._grippers.get(
            step.get("gripper", self._current_gripper), {}
        )
        goal = PickSampleAction.Goal()
        goal.use_vision = False
        goal.approach_pose = step.get("approach_pose", "")
        goal.target_pose = step.get("target_pose", "")
        goal.gripper_group = gripper_config.get("gripper_group", "")
        goal.gripper_states_json = json.dumps(gripper_config.get("states", {}))
        goal.poses_json = poses_json
        goal.constraints_json = (
            json.dumps(step["constraints"]) if "constraints" in step else ""
        )
        return self._send_and_wait(
            self._pick_sample_client, goal, "pick_sample", self._timeouts["pick_sample"]
        )

    def _call_place_sample_hardcoded(
        self, step: dict[str, Any], poses_json: str
    ) -> bool:
        """Non-vision place: legacy PlaceSample server, joint-pose sequence."""
        gripper_config = self._grippers.get(
            step.get("gripper", self._current_gripper), {}
        )
        goal = PlaceSampleAction.Goal()
        goal.use_vision = False
        goal.approach_pose = step.get("approach_pose", "")
        goal.target_pose = step.get("target_pose", "")
        goal.gripper_group = gripper_config.get("gripper_group", "")
        goal.gripper_states_json = json.dumps(gripper_config.get("states", {}))
        goal.poses_json = poses_json
        goal.constraints_json = (
            json.dumps(step["constraints"]) if "constraints" in step else ""
        )
        success = self._send_and_wait(
            self._place_sample_client,
            goal,
            "place_sample",
            self._timeouts["place_sample"],
        )
        if success and self._current_gripper == "epick":
            self._vacuum.armed = False
            self._vacuum.lost = False
        return success

    def _call_pipettor(self, step: dict[str, Any], poses_json: str) -> bool:
        """Call the Pipettor action server."""
        from std_msgs.msg import ColorRGBA

        goal = PipettorAction.Goal()
        goal.operation = step.get("operation", "")
        goal.volume_pct = float(step.get("volume_pct", 0.0))
        goal.poses_json = poses_json

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
        feedback.progress_percentage = (
            (current_step / total_steps) * 100.0 if total_steps > 0 else 0.0
        )
        feedback.status_message = (
            "Task completed" if not task_type else f"Executing: {task_type}"
        )
        feedback.current_gripper = self._current_gripper

        goal_handle.publish_feedback(feedback)


def main(args=None):
    """Run the MTC Orchestrator server."""
    run_server(MTCOrchestratorServer, args)


if __name__ == "__main__":
    main()
