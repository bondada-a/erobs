#!/usr/bin/env python3
"""MTC Orchestrator - coordinates multi-step robot tasks.

Receives task scripts (JSON) and dispatches steps to specialized action servers.
Manages MoveIt lifecycle based on gripper configuration.

Supports beamline-agnostic deployment via beamline configuration files.

Batching optimization: Consecutive simple tasks (moveto, end_effector) are
grouped into a single MTC Task with multiple stages, reducing planning
overhead (~1.5s per task saved).
"""

import hashlib
import json
import math
import os
import threading
import time
from typing import Any

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
from std_srvs.srv import Trigger
from std_msgs.msg import String

from beambot.core.moveit_lifecycle_manager import MoveItLifecycleManager
from beambot.core.vacuum_monitor import VacuumMonitor
from beambot.stages.move_to_stages import MoveToStages
from beambot.stages.end_effector_stages import EndEffectorStages
from beambot.batch_planner import group_into_batches
from beambot.stages.base_stages import wait_for_future


class MTCOrchestratorServer(Node):
    """Action server that coordinates multi-step robot tasks.

    Receives task scripts in JSON format and dispatches individual
    steps to specialized MTC action servers (MoveTo, EndEffector, etc.)

    Supports beamline-agnostic deployment via beamline configuration files.
    """

    # Task types supported in dry_run (plan-only) mode. v1 only previews the
    # task types the orchestrator already plans in-process via the batched
    # path; everything else dispatches to a remote action server and is out of
    # scope for plan-only previewing.
    DRY_RUN_SUPPORTED_TYPES = {"moveto", "end_effector"}

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

        # Plan cache: populated on a successful dry-run; consumed by the next
        # non-dry-run goal whose key matches. Lets the operator preview a
        # plan and then execute exactly that plan instead of re-planning
        # (which OMPL randomization would otherwise randomize).
        # Keys: goal_key (sha256 of normalized goal JSON), gripper,
        #       task (MTC core.Task with solutions populated), stage (BaseStages
        #       instance — needed to call execute_solution).
        self._plan_cache: dict | None = None
        self._plan_cache_lock = threading.Lock()
        # Set by _execute_batch in dry_run mode so _execute() can capture
        # the freshly-planned MTC task into the plan cache.
        self._last_planned_task = None

        # Pause/Resume state
        self._pause_requested = False
        self._is_paused = False
        self._pause_event = threading.Event()
        self._pause_event.set()  # Start in "go" state (not blocked)

        # Load beamline configuration (single source of truth, BEAMBOT_BEAMLINE_CONFIG env var)
        from beambot.config_loader import load_beamline_config, resolve_beamline_path
        self.declare_parameter("use_mock_hardware", False)
        self.declare_parameter("enable_batching", True)
        self.declare_parameter("cup_profile", "")  # Override cup profile (empty = use beamline config default)
        self._use_mock_hardware = self.get_parameter("use_mock_hardware").value
        self._enable_batching = self.get_parameter("enable_batching").value

        config, config_file = load_beamline_config()
        self._poses_file = resolve_beamline_path(config.get("poses_file", ""), config_file)

        self._grippers = config["grippers"]  # Dict of gripper_name -> {moveit_package, tool_voltage, gripper_group, states}
        self._robot_ip = config["robot"]["ip"]  # Single source: config file
        self._arm_group = config.get("robot", {}).get("arm_group", "ur_arm")  # For batched execution
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

        # Warm up the spincoater YOLO model in the background so torch/CUDA init
        # happens during idle startup, not on the executor thread mid-task (which
        # starves the 500Hz control loop and balloons load time from ~9s to ~70s).
        self._warmup_spincoater_model()

    def _warmup_spincoater_model(self):
        """Pre-load the spincoater sample YOLO model in a background daemon thread."""
        def _warmup():
            try:
                import numpy as np
                from beambot.detection.spincoater import _get_sample_model
                self.get_logger().info("Warming up spincoater sample model (background)...")
                model = _get_sample_model()
                # Dummy inference to trigger CUDA kernel compilation / graph build
                # so the first real detection is instant.
                dummy = np.zeros((640, 640, 3), dtype=np.uint8)
                model(dummy, conf=0.5, verbose=False)
                self.get_logger().info("Spincoater sample model ready")
            except Exception as e:  # noqa: BLE001 — warmup is best-effort
                self.get_logger().warning(f"Spincoater model warmup skipped: {e}")

        threading.Thread(target=_warmup, daemon=True).start()

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

    # ---- Plan cache (dry-run -> execute reuse) -------------------------

    def _compute_plan_cache_key(self, full_json: str, gripper: str) -> str:
        """Compute a stable key from goal JSON + gripper.

        Re-serializing parsed JSON with sort_keys=True normalizes whitespace
        and field ordering so byte-identical re-sends always hash the same.
        Falls back to raw bytes if the JSON can't be parsed (cache will miss
        more often, which is fine — invalidation is the safe direction).
        """
        try:
            normalized = json.dumps(json.loads(full_json), sort_keys=True)
        except json.JSONDecodeError:
            normalized = full_json
        h = hashlib.sha256()
        h.update(normalized.encode("utf-8"))
        h.update(b"|")
        h.update(gripper.encode("utf-8"))
        return h.hexdigest()

    def _validate_plan_cache(
        self, goal_key: str, current_gripper: str
    ) -> tuple[bool, str]:
        """Check if the cached plan is still safe to execute.

        Returns:
            (valid, reason). On valid=False, reason is a structured string
            starting with CACHE_<REASON>: which the GUI parses to show a
            friendly message.
        """
        with self._plan_cache_lock:
            cache = self._plan_cache
            if cache is None:
                return False, "CACHE_MISS: No cached plan; planning fresh"
            if cache["goal_key"] != goal_key:
                return False, (
                    "CACHE_KEY_MISMATCH: Cached plan was for a different task. "
                    "Run Dry Run again to preview this task, then Execute."
                )
            if cache["gripper"] != current_gripper:
                return False, (
                    "CACHE_GRIPPER_CHANGED: Tool exchange happened since dry-run. "
                    "Run Dry Run again to preview with the current gripper."
                )

            # No robot-moved check here: MoveIt's trajectory_execution.
            # allowed_start_tolerance (~0.01 rad, on by default — verified in
            # libmoveit_trajectory_execution_manager.so) rejects a stale-start
            # replay BEFORE motion, so a jogged robot fails loudly instead of
            # jumping. ponytail: a friendly "robot moved, re-run Dry Run" message
            # needs the real MoveItErrorCode observed on a jog-then-Execute
            # hardware test — add the ~2-line translation then, not on a guess.
            return True, ""

    def _clear_plan_cache(self, reason: str = "") -> None:
        """Drop the cached plan. Call after execute, on tool_exchange, or
        whenever cached MTC state should not be reused."""
        with self._plan_cache_lock:
            if self._plan_cache is not None:
                if reason:
                    self.get_logger().info(f"Plan cache cleared: {reason}")
                self._plan_cache = None

    # _group_into_batches extracted to beambot.batch_planner.group_into_batches

    def _grasp_breaker_actions(self, gripper: str) -> set[str]:
        """Return end_effector_action values that must NOT be batched for this gripper.

        Currently empty for every gripper: ePick's grasp (vacuum_on) is now
        batched alongside surrounding moves, just like vacuum_off already is.
        The drop-detection watchdog that originally motivated breaking the
        batch is disabled (the 3mm sample is too small to give a reliable
        ObjectDetectionStatus seal signal), so a per-grasp step boundary no
        longer protects anything.

        The plumbing is retained for two reasons: (1) re-adding ePick grasp as
        a breaker is a one-line change here (return the grasp state name read
        from grippers.<gripper>.states.grasp), and (2) if fused grasps drop the
        sample because the suction seal hasn't formed before the arm departs,
        the fix is a short dwell stage after the vacuum_on MoveTo — not
        reverting to per-step execution.
        """
        return set()

    def _execute_batch(
        self,
        batch_tasks: list[dict[str, Any]],
        poses_json: str,
        dry_run: bool = False,
        cached_plan: dict | None = None,
    ) -> bool:
        """Execute a batch of tasks as a single MTC Task.

        Creates one MTC Task, adds stages from each task, then plans and
        executes once. This reduces planning overhead (~1.5s per task saved).

        Args:
            batch_tasks: List of batchable task dictionaries
            poses_json: JSON string with pose definitions
            dry_run: If True, plan only and publish the trajectory for the
                GUI viewer; do not move the robot. On success, populates
                self._plan_cache so a subsequent execute can replay the
                same plan.
            cached_plan: If provided, skip the build+plan step and execute
                cached_plan["task"] directly. Used when an Execute goal hits
                a valid cache populated by a prior dry-run.

        Returns:
            True if all tasks succeeded, False on any failure
        """
        if not batch_tasks:
            return True

        # Log batch info
        task_types = [t.get("task_type", "?") for t in batch_tasks]
        self.get_logger().info(
            f"{'Replaying cached plan for' if cached_plan else 'Executing'} "
            f"batch of {len(batch_tasks)} tasks: {task_types}"
        )

        # Create stage instances (they share MTC node via module-level singleton)
        moveto_stage = MoveToStages(self, self._arm_group)
        endeffector_stage = EndEffectorStages(self, self._arm_group)

        # Replay path: skip task construction and planning, run the cached
        # solution directly.
        if cached_plan is not None:
            error = moveto_stage.execute_solution(cached_plan["task"])
            if error is not None:
                self._last_error = error
                return False
            return True

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

        if dry_run:
            # Plan only — caller stashes (task, stage) into the plan cache
            # so the next execute can replay this exact solution.
            error = moveto_stage.init_and_plan(task, dry_run=True)
            if error is not None:
                self._last_error = error
                return False
            # Stash the planned task on the instance so _execute() can read
            # it after the call returns. Using a private attribute since
            # _execute_batch's bool return type is load-bearing for callers
            # that don't care about caching.
            self._last_planned_task = task
            return True

        # Normal path: plan + execute end-to-end
        error = moveto_stage.load_plan_execute(task)
        if error is not None:
            self._last_error = error
            return False
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
        goal.constraints_json = json.dumps(step["constraints"]) if "constraints" in step else ""
        return goal

    def _create_endeffector_goal(self, step: dict[str, Any]) -> EndEffectorAction.Goal:
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

        start_gripper, tasks, poses_json, dry_run = parsed
        task_count = len(tasks)
        if dry_run:
            self.get_logger().info(
                f"DRY-RUN preview enabled — planning {task_count} step(s) "
                f"without moving the robot"
            )

        # Compute the plan-cache key from the raw goal payload + gripper.
        # On dry-run we'll write the cache; on execute we'll check it.
        goal_key = self._compute_plan_cache_key(
            goal_handle.request.full_json, start_gripper
        )

        # Cache validation for non-dry-run goals: if we have a cache and it
        # matches the goal, we'll replay it; if we have a cache that DOESN'T
        # match, refuse so the operator sees the staleness instead of
        # silently executing a different (re-planned) trajectory.
        cached_plan_for_replay: dict | None = None
        if not dry_run:
            with self._plan_cache_lock:
                has_cache = self._plan_cache is not None
            if has_cache:
                valid, reason = self._validate_plan_cache(goal_key, start_gripper)
                if valid:
                    with self._plan_cache_lock:
                        cached_plan_for_replay = self._plan_cache
                    self.get_logger().info(
                        "Plan cache hit — executing previewed plan without re-planning"
                    )
                elif not reason.startswith("CACHE_MISS"):
                    # Stale cache that doesn't match this goal: refuse.
                    # CACHE_MISS just means "no cache yet" → fall through and
                    # plan fresh, preserving Execute-without-Dry-Run behavior.
                    result.error_message = reason
                    self._clear_plan_cache("stale on execute")
                    goal_handle.abort()
                    self._publish_state("IDLE")
                    return result

        # On a fresh dry-run, drop any prior cache before planning the new one.
        if dry_run:
            self._clear_plan_cache("new dry-run starting")

        # Initialize gripper state
        self._current_gripper = start_gripper
        self._publish_gripper(start_gripper)

        # Apply cup_profile override if parameter was changed via MCP. Set it
        # on the MoveIt manager instead of writing into self._grippers, which
        # is a reference into the shared, cached beamline config (see
        # config_loader.load_beamline_config). Empty/cleared param falls back to
        # the gripper's YAML cup_profile.
        self._moveit_manager.cup_override = self.get_parameter("cup_profile").value or ""

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

        # Group tasks into batches for optimized execution. All end_effector
        # actions (including ePick vacuum_on/off) currently batch with adjacent
        # moves — _grasp_breaker_actions returns no breakers (see its docstring
        # for the rationale and the dwell-stage fallback). The breaker_actions
        # plumbing is retained so grasp-breaking can be re-enabled cheaply.
        # Dry-run and live runs share this same grouping so a previewed plan
        # replays against an identical batch structure.
        breaker_actions = self._grasp_breaker_actions(start_gripper)
        batches = group_into_batches(
            tasks,
            enabled=self._enable_batching,
            breaker_actions=breaker_actions,
        )
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

            # Vacuum-loss abort DISABLED — even if the ePick reports a dropped
            # object, the sequence continues instead of aborting.
            # if not dry_run:
            #     vacuum_error = self._vacuum.check_lost()
            #     if vacuum_error:
            #         result.error_message = f"Step {completed_tasks + 1} aborted: {vacuum_error}"
            #         result.completed_steps = completed_tasks
            #         goal_handle.abort()
            #         self._publish_state("IDLE")
            #         return result

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

                # Controller activation is intentionally NOT done here: the
                # ur_control.launch.py spawner and the UR driver's
                # controller_stopper_node (which restarts controllers it stopped
                # on a connection drop) own it. An earlier in-orchestrator
                # activation helper raced the spawner ("already active" failures)
                # and was removed — re-add only with connection-drop recovery
                # stress-tested on Jazzy.

                self._last_planned_task = None
                ok = self._execute_batch(
                    batch_tasks, poses_json,
                    dry_run=dry_run,
                    cached_plan=cached_plan_for_replay,
                )
                if not ok:
                    result.error_message = f"Batch failed at step {completed_tasks + 1}: {self._last_error}"
                    result.completed_steps = completed_tasks
                    goal_handle.abort()
                    self._publish_state("IDLE")
                    return result

                # On a successful dry-run, stash the planned task so the
                # next non-dry-run goal with the same key can replay it.
                if dry_run and self._last_planned_task is not None:
                    with self._plan_cache_lock:
                        self._plan_cache = {
                            "goal_key": goal_key,
                            "task": self._last_planned_task,
                            "gripper": self._current_gripper,
                        }
                    self.get_logger().info("Plan cached for next execute")
                    self._last_planned_task = None
                # On a successful non-dry-run execute, drop the cache so the
                # next goal plans fresh from the new robot state.
                elif not dry_run:
                    self._clear_plan_cache("after successful execute")

                if not dry_run:
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

                # Dry-run safety net: the parser already rejects unsupported
                # types, but if batching is disabled by parameter, even
                # supported types fall through to this single-task path which
                # would actually execute. Route them through _execute_batch
                # instead so dry_run is honored. Note: caching with batching
                # disabled is best-effort — if a goal contains multiple
                # batches, we cache only the last one. In practice batching
                # is on by default; this is a safety fallback.
                if dry_run:
                    self._update_feedback(
                        feedback, goal_handle, completed_tasks + 1, task_count, task_type
                    )
                    self._last_planned_task = None
                    if not self._execute_batch([task], poses_json, dry_run=True):
                        result.error_message = f"{task_type} preview failed: {self._last_error}"
                        result.completed_steps = completed_tasks
                        goal_handle.abort()
                        self._publish_state("IDLE")
                        return result
                    if self._last_planned_task is not None:
                        with self._plan_cache_lock:
                            self._plan_cache = {
                                "goal_key": goal_key,
                                "task": self._last_planned_task,
                                "gripper": self._current_gripper,
                            }
                        self._last_planned_task = None
                    completed_tasks += 1
                    result.completed_steps = completed_tasks
                    continue

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

        # Final vacuum-loss abort DISABLED — a drop during the last step no
        # longer aborts; the sequence is reported complete regardless.
        # if not dry_run:
        #     vacuum_error = self._vacuum.check_lost()
        #     if vacuum_error:
        #         result.error_message = f"Final step aborted: {vacuum_error}"
        #         result.completed_steps = completed_tasks
        #         goal_handle.abort()
        #         self._publish_state("IDLE")
        #         return result

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

    def _load_poses_registry(self) -> dict:
        """Read the poses YAML registry file. Returns empty dict on failure."""
        if not os.path.exists(self._poses_file):
            return {}
        try:
            with open(self._poses_file, "r") as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warning(f"Failed to read poses registry: {e}")
            return {}

    def _parse_goal(self, goal: MTCExecution.Goal, result: MTCExecution.Result):
        """Parse and validate the goal JSON.

        Returns:
            (start_gripper, tasks, poses_json, dry_run) if valid,
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

        # Dry-run validation: only moveto + end_effector are previewable in v1.
        # Reject upfront so the operator gets a clear message instead of a
        # half-finished preview.
        dry_run = bool(getattr(goal, "dry_run", False))
        if dry_run:
            unsupported = [
                (i, t.get("task_type", "?"))
                for i, t in enumerate(script["tasks"])
                if t.get("task_type", "") not in self.DRY_RUN_SUPPORTED_TYPES
            ]
            if unsupported:
                bad = ", ".join(f"step {i + 1} ({t})" for i, t in unsupported)
                allowed = ", ".join(sorted(self.DRY_RUN_SUPPORTED_TYPES))
                result.error_message = (
                    f"Dry-run not supported for: {bad}. "
                    f"v1 supports only: {allowed}. "
                    f"Run without dry_run (or with use_mock_hardware) for full task types."
                )
                return None

        # Auto-resolve named poses from the registry when not supplied in the goal
        poses = script.get("poses", {})
        pose_keys_needed = set()
        for task in script["tasks"]:
            target = task.get("target", "")
            if target and target not in poses:
                pose_keys_needed.add(target)
            for key in task.get("scan_positions", []):
                if key not in poses:
                    pose_keys_needed.add(key)
            for field in ("scan_pose", "approach_pose", "target_pose",
                          "place_pose", "pickup_pose"):
                val = task.get(field)
                if val and val not in poses:
                    pose_keys_needed.add(val)

        if pose_keys_needed:
            registry = self._load_poses_registry()
            resolved = 0
            for key in pose_keys_needed:
                if key in registry:
                    poses[key] = registry[key]
                    resolved += 1
            if resolved:
                self.get_logger().info(
                    f"Auto-resolved {resolved} pose(s) from registry: "
                    f"{[k for k in pose_keys_needed if k in registry]}"
                )

        return (
            start_gripper,
            script["tasks"],
            json.dumps(poses),
            dry_run,
        )

    # ------------------------------------------------------------------
    def _execute_step(
        self, task_type: str, step: dict[str, Any], poses_json: str
    ) -> bool:
        """Execute a single step by dispatching to the appropriate action server."""
        # Controller activation is intentionally NOT done here — owned by the
        # ur_control.launch.py spawner + UR driver controller_stopper_node
        # (see the batched-execution path in _execute for the full rationale).

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
        elif task_type == "place_spincoater":
            return self._call_place_spincoater(step, poses_json)
        elif task_type == "pick_spincoater":
            return self._call_pick_spincoater(step, poses_json)
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
        if not wait_for_future(send_future, timeout=10.0):
            self._last_error = f"TIMEOUT: {name} goal acceptance timed out (10s)"
            self.get_logger().error(self._last_error)
            return False

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self._last_error = f"{name} goal was rejected by action server"
            self.get_logger().error(self._last_error)
            return False

        # Wait for result with caller-provided timeout
        result_future = goal_handle.get_result_async()
        if not wait_for_future(result_future, timeout=timeout):
            self._last_error = f"TIMEOUT: {name} timed out after {timeout}s"
            self.get_logger().error(self._last_error)
            goal_handle.cancel_goal_async()
            return False

        result = result_future.result()
        self._last_result = result.result
        if not result.result.success:
            # Capture the real error_message from the action server
            self._last_error = getattr(result.result, 'error_message', '') or f"{name} failed (no details)"
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
            self._endeffector_client, goal, "end_effector", self._timeouts["end_effector"]
        )

    def _call_toolexchange(self, step: dict[str, Any], poses_json: str) -> bool:
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

    def _gripper_ik_frame(self) -> str:
        """Return the IK tip frame for the currently attached gripper.

        Reads grippers.<name>.tip_frame from the active beamline YAML. The
        single source of truth lives there so adding a gripper at a new
        beamline is a YAML edit, not a code change.
        """
        from beambot.config_loader import gripper_tip_frame
        return gripper_tip_frame(self._current_gripper, default="flange")

    def _gripper_z_offset(self) -> float:
        """Default Z offset for the currently attached gripper (meters)."""
        return float(self._grippers.get(self._current_gripper, {}).get("z_offset", 0.0))

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
        done = wait_for_future(future, timeout=5.0, poll_interval=0.05)

        self.destroy_client(client)
        if done and future.result().success:
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
        done = wait_for_future(future, timeout=5.0, poll_interval=0.05)
        if done and future.result().success:
            self.get_logger().info("Vision server TF buffer reset")
        else:
            self.get_logger().warning("Vision TF reset did not complete")

    def _handle_tool_exchange(self, step: dict[str, Any], poses_json: str) -> bool:
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
            self._moveit_manager.notify_voltage_change(0)
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
            # Any cached dry-run plan was for the previous gripper's SRDF /
            # collision model — invalidate it so the next execute can't
            # silently replay a plan against the wrong robot model.
            self._clear_plan_cache("tool exchange")

            if not self._moveit_manager.launch_moveit_with_gripper(new_gripper):
                self._last_error = (
                    f"Failed to restart MoveIt after tool exchange "
                    f"({self._current_gripper} → {new_gripper})"
                )
                self.get_logger().error(self._last_error)
                return False

            # Reset vision server TF buffer so it picks up the new URDF frames
            self._reset_vision_tf()

        return True

    def _call_vision_moveto(self, step: dict[str, Any], poses_json: str) -> bool:
        """Call the VisionMoveTo action server.

        Supports:
        - tag_id: ArUco marker ID (for marker detection)
        - detection_type: "marker" (default) or "sample_roi"
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
        goal.z_offset = float(step.get("z_offset", self._gripper_z_offset()))
        goal.detect_only = bool(step.get("detect_only", False))
        goal.offset_direction = step.get("offset_direction", "")
        goal.offset_distance = float(step.get("offset_distance", 0.0))
        goal.marker_offset_x = float(step.get("marker_offset_x", 0.0))
        goal.marker_offset_y = float(step.get("marker_offset_y", 0.0))
        goal.marker_offset_z = float(step.get("marker_offset_z", 0.0))
        # Pass current gripper's IK frame to avoid stale TF auto-detection
        goal.ik_frame = self._gripper_ik_frame()
        # sample_roi detection parameters (only consumed when detection_type="sample_roi")
        goal.strategy = step.get("strategy", "")
        goal.edge_inset_mm = float(step.get("edge_inset_mm", 0.0))

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

    def _call_vision_scan(self, step: dict[str, Any], poses_json: str) -> bool:
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

    def _call_pick_sample(self, step: dict[str, Any], poses_json: str) -> bool:
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
        goal.z_offset = float(step.get("z_offset", self._gripper_z_offset()))
        goal.scan_pose = step.get("scan_pose", "")
        goal.marker_offset_x = float(step.get("marker_offset_x", 0.0))
        goal.marker_offset_y = float(step.get("marker_offset_y", 0.0))
        goal.marker_offset_z = float(step.get("marker_offset_z", 0.0))
        goal.offset_direction = step.get("offset_direction", "")
        goal.offset_distance = float(step.get("offset_distance", 0.0))
        goal.ik_frame = self._gripper_ik_frame()
        goal.strategy = step.get("strategy", "")
        goal.edge_inset_mm = float(step.get("edge_inset_mm", 0.0))
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
            # Vacuum-loss abort DISABLED — a failed vacuum check after pick no
            # longer returns False; the flow continues regardless.
            # if not vacuum_ok:
            #     self._last_error = (
            #         "VACUUM_LOST: ePick reports NO_OBJECT_DETECTED after pick. "
            #         "Send vacuum_off then vacuum_on to retry."
            #     )
            #     self.get_logger().error(self._last_error)
            #     return False

            # Arm background vacuum monitor for transport (pick_sample turns
            # vacuum on internally, but VacuumMonitor only tracks end_effector
            # tasks — so we arm it explicitly here)
            if vacuum_ok and self._current_gripper == "epick":
                self._vacuum.armed = True
                self._vacuum.lost = False

        return success

    def _call_place_sample(self, step: dict[str, Any], poses_json: str) -> bool:
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
        goal.z_offset = float(step.get("z_offset", self._gripper_z_offset()))
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

    def _call_pipettor(self, step: dict[str, Any], poses_json: str) -> bool:
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

    def _call_place_spincoater(self, step: dict[str, Any], poses_json: str) -> bool:
        """Place a sample on the spincoater with vision-guided orientation.

        Sequence:
          1. Move to scan pose (framing the chuck centered for flash-lit 2D capture)
          2. Capture 2D image, detect pocket angle via red-field negative-space method
          3. Compute corrected joint 6 = place_pose[5] + detected_angle + k_offset
          4. Move to placement pose with corrected joint 6 (with Z clearance)
          5. Move forward to contact the surface
          6. Release vacuum

        Task JSON fields:
          scan_pose: str — pose key for the scan position (default "spincoater_scan")
          place_pose: str — pose key for the placement position (default "spincoater_place")
          forward_distance: float — distance in meters to move forward after positioning
                                    (default 0.003 = 3mm)
          k_offset: float — calibration constant in degrees (default 0.0)
          release: bool — whether to release vacuum after placement (default true)
        """
        from beambot.camera.zivid import capture_2d
        from beambot.detection import detect_spincoater_pocket

        scan_pose_key = step.get("scan_pose", "spincoater_scan")
        place_pose_key = step.get("place_pose", "spincoater_place")
        forward_distance = float(step.get("forward_distance", 0.003))
        k_offset = float(step.get("k_offset", 0.0))
        release = step.get("release", True)

        # Resolve poses
        poses = json.loads(poses_json) if poses_json else {}
        scan_joints = poses.get(scan_pose_key)
        place_joints = poses.get(place_pose_key)

        if scan_joints is None or place_joints is None:
            self._last_error = (
                f"place_spincoater: missing pose '{scan_pose_key}' or "
                f"'{place_pose_key}' in poses"
            )
            self.get_logger().error(self._last_error)
            return False

        # Step 1: Move to scan pose
        self.get_logger().info(f"place_spincoater: moving to scan pose '{scan_pose_key}'")
        scan_step = {"target": scan_pose_key, "planning_type": "joint"}
        if not self._call_moveto(scan_step, poses_json):
            self._last_error = "place_spincoater: failed to reach scan pose"
            return False

        # Step 2: 2D capture + pocket detection
        self.get_logger().info("place_spincoater: capturing 2D image...")
        time.sleep(1.0)  # settle time
        image = capture_2d(self, timeout=15.0)
        if image is None:
            self._last_error = "place_spincoater: 2D capture failed"
            self.get_logger().error(self._last_error)
            return False

        detection = detect_spincoater_pocket(image)
        if detection is None:
            self._last_error = "place_spincoater: pocket detection failed"
            self.get_logger().error(self._last_error)
            return False

        pocket_angle = detection["angle_mod90"]
        self.get_logger().info(
            f"place_spincoater: pocket detected — angle_mod90={pocket_angle:.1f}°, "
            f"aspect={detection['aspect']:.2f}, solidity={detection['solidity']:.2f}"
        )

        # Step 3: Compute corrected joint 6
        base_j6 = place_joints[5]
        raw_correction = pocket_angle + k_offset
        # Snap to nearest ±45° (4-fold symmetry)
        correction = raw_correction % 90
        if correction > 45:
            correction -= 90
        corrected_j6 = base_j6 + correction

        self.get_logger().info(
            f"place_spincoater: j6 correction — base={base_j6:.1f}°, "
            f"pocket={pocket_angle:.1f}°, k={k_offset:.1f}°, "
            f"correction={correction:.1f}°, target_j6={corrected_j6:.1f}°"
        )

        # Step 4: Move to placement pose with corrected j6
        corrected_place = list(place_joints)
        corrected_place[5] = corrected_j6
        place_pose_name = "_spincoater_place_corrected"
        poses[place_pose_name] = corrected_place
        corrected_poses_json = json.dumps(poses)

        place_step = {"target": place_pose_name, "planning_type": "joint"}
        if not self._call_moveto(place_step, corrected_poses_json):
            self._last_error = "place_spincoater: failed to reach placement pose"
            return False

        # Step 5: Move forward to contact
        if forward_distance > 0:
            self.get_logger().info(
                f"place_spincoater: moving forward {forward_distance*1000:.1f}mm"
            )
            fwd_step = {"target": "", "direction": "forward", "distance": forward_distance}
            if not self._call_moveto(fwd_step, corrected_poses_json):
                self._last_error = "place_spincoater: forward move failed"
                return False

        # Step 6: Release vacuum
        if release:
            self.get_logger().info("place_spincoater: releasing vacuum")
            release_step = {"end_effector_action": "vacuum_off"}
            if not self._call_endeffector(release_step, corrected_poses_json):
                self._last_error = "place_spincoater: vacuum release failed"
                return False

        self.get_logger().info("place_spincoater: placement complete")
        return True

    def _call_pick_spincoater(self, step: dict[str, Any], poses_json: str) -> bool:
        """Pick a sample from the spincoater with vision-guided orientation.

        Mirrors place_spincoater structure exactly:
          1. Move to scan pose
          2. Capture 2D image, detect sample angle via YOLO segmentation
          3. Move to pickup pose with corrected joint 6
          4. Move forward to contact the sample
          5. Activate vacuum

        Task JSON fields:
          scan_pose: str — pose key for the scan position (default "spincoater_scan")
          pickup_pose: str — pose key for the pickup position (default "spincoater_place")
          forward_distance: float — distance in meters to move forward (default 0.003)
          k_offset: float — calibration constant in degrees (default 0.0)
        """
        from beambot.camera.zivid import capture_2d
        from beambot.detection import detect_spincoater_sample

        scan_pose_key = step.get("scan_pose", "spincoater_scan")
        pickup_pose_key = step.get("pickup_pose", "spincoater_place")
        forward_distance = float(step.get("forward_distance", 0.003))
        k_offset = float(step.get("k_offset", 0.0))

        # Resolve poses
        poses = json.loads(poses_json) if poses_json else {}
        scan_joints = poses.get(scan_pose_key)
        pickup_joints = poses.get(pickup_pose_key)

        if scan_joints is None or pickup_joints is None:
            self._last_error = (
                f"pick_spincoater: missing pose '{scan_pose_key}' or "
                f"'{pickup_pose_key}' in poses"
            )
            self.get_logger().error(self._last_error)
            return False

        # Step 1: Move to scan pose
        self.get_logger().info(f"pick_spincoater: moving to scan pose '{scan_pose_key}'")
        scan_step = {"target": scan_pose_key, "planning_type": "joint"}
        if not self._call_moveto(scan_step, poses_json):
            self._last_error = "pick_spincoater: failed to reach scan pose"
            return False

        # Step 2: 2D capture + sample detection
        self.get_logger().info("pick_spincoater: capturing 2D image...")
        time.sleep(1.0)
        image = capture_2d(self, timeout=15.0)
        if image is None:
            self._last_error = "pick_spincoater: 2D capture failed"
            self.get_logger().error(self._last_error)
            return False

        detection = detect_spincoater_sample(image)
        if detection is None:
            self._last_error = "pick_spincoater: sample detection failed"
            self.get_logger().error(self._last_error)
            return False

        sample_angle = detection["angle_mod90"]
        self.get_logger().info(
            f"pick_spincoater: sample detected — angle_mod90={sample_angle:.1f}°, "
            f"confidence={detection['confidence']:.2f}, center={detection['center_px']}"
        )

        # Step 3: Compute corrected joint 6
        base_j6 = pickup_joints[5]
        raw_correction = sample_angle + k_offset
        correction = raw_correction % 90
        if correction > 45:
            correction -= 90
        corrected_j6 = base_j6 + correction

        self.get_logger().info(
            f"pick_spincoater: j6 correction — base={base_j6:.1f}°, "
            f"sample={sample_angle:.1f}°, k={k_offset:.1f}°, "
            f"correction={correction:.1f}°, target_j6={corrected_j6:.1f}°"
        )

        # Step 4: Move to pickup pose with corrected j6
        corrected_pickup = list(pickup_joints)
        corrected_pickup[5] = corrected_j6
        pickup_pose_name = "_spincoater_pick_corrected"
        poses[pickup_pose_name] = corrected_pickup
        corrected_poses_json = json.dumps(poses)

        pickup_step = {"target": pickup_pose_name, "planning_type": "joint"}
        if not self._call_moveto(pickup_step, corrected_poses_json):
            self._last_error = "pick_spincoater: failed to reach pickup pose"
            return False

        # Step 5: Move forward to contact
        if forward_distance > 0:
            self.get_logger().info(
                f"pick_spincoater: moving forward {forward_distance*1000:.1f}mm"
            )
            fwd_step = {"target": "", "direction": "forward", "distance": forward_distance}
            if not self._call_moveto(fwd_step, corrected_poses_json):
                self._last_error = "pick_spincoater: forward move failed"
                return False

        # Step 6: Activate vacuum
        self.get_logger().info("pick_spincoater: activating vacuum")
        grasp_step = {"end_effector_action": "vacuum_on"}
        if not self._call_endeffector(grasp_step, corrected_poses_json):
            self._last_error = "pick_spincoater: vacuum activation failed"
            return False

        self.get_logger().info("pick_spincoater: pickup complete")
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
