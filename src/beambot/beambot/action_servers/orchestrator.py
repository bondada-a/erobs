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
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

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

        # Trajectory cache: multi-entry, keyed on (start joints, goal, gripper).
        # A planned move is stored as a serialized Solution msg; a later goal
        # with the same key replays it instead of re-planning (see PlanCache).
        self._plan_cache = PlanCache(self.get_logger())
        # Set by _execute_batch after a successful plan so _execute() can store
        # the serialized Solution msg (not the live Task — it dangles after a
        # MoveIt relaunch) into the trajectory cache.
        self._last_planned_sol_msg = None

        # Pause/Resume state
        self._pause_requested = False
        self._is_paused = False
        self._pause_event = threading.Event()
        self._pause_event.set()  # Start in "go" state (not blocked)

        # Load beamline configuration (single source of truth, BEAMBOT_BEAMLINE_CONFIG env var)
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
            self,
            self._grippers,
            self._robot_ip,
            self._callback_group,
            use_mock_hardware=self._use_mock_hardware,
            enable_joystick=self.get_parameter("enable_joystick").value,
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
            self, MoveToAction, "beambot_moveto", callback_group=self._callback_group
        )
        self._endeffector_client = ActionClient(
            self,
            EndEffectorAction,
            "beambot_endeffector",
            callback_group=self._callback_group,
        )
        # Unified vision pipeline (issue #88). vision_moveto routes here,
        # replacing the retired beambot_vision_moveto server.
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

        # Pause/Resume services
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

        # Execution state publisher
        self._state_publisher = self.create_publisher(
            String, "beambot/execution_state", latched_qos
        )
        self._publish_state("IDLE")

        # Current gripper publisher (latched so late subscribers get last value)
        self._gripper_publisher = self.create_publisher(
            String, "beambot/current_gripper", latched_qos
        )
        self._publish_gripper(self._current_gripper)

        # Vacuum monitoring (ePick grasp verification)
        self._vacuum = VacuumMonitor(self, self._grippers, self._callback_group)

        self.get_logger().info(
            "MTC Orchestrator (Python) started on 'beambot_execution'"
        )
        self.get_logger().info(
            "Pause/Resume services available: beambot/pause, beambot/resume"
        )

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

                self.get_logger().info(
                    "Warming up spincoater sample model (background)..."
                )
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

        States: IDLE, RUNNING, PAUSED, COMPLETING_TASK, CANCELING, FAULTED
        """
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
        current_step: int,
        total_steps: int,
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
        feedback.status_message = (
            f"PAUSED - completed {current_step}/{total_steps}, waiting for resume"
        )
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
                GUI viewer; do not move the robot. On success, the planned
                solution is captured (self._last_planned_sol_msg) so the
                caller can store it in the trajectory cache.
            cached_plan: If provided, skip the build+plan step and replay
                cached_plan["sol_msg"] directly via /execute_task_solution.
                Used when an Execute goal's (start, goal, gripper) key hits
                the cache.

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
        # solution directly. On a SAFE replay failure (server gone after a
        # relaunch, goal rejected, or a terminal start-tolerance/control error —
        # no motion left active) we do NOT dead-end the move: fall through to a
        # fresh plan+execute so the cache is never worse than no cache. But on
        # REPLAY_TIMEOUT the goal may still be ACTIVE on the controller, so we
        # abort instead of re-dispatching a second overlapping trajectory.
        # ponytail: replay is collision-blind — it does NOT re-check the cached
        # path against the live scene. Safe for a static cell; if the scene can
        # change between identical moves, gate this on a PlanningScene
        # isPathValid check (MTC issue #198 pattern) before replaying.
        if cached_plan is not None:
            error = moveto_stage.execute_solution_msg(cached_plan["sol_msg"])
            if error is None:
                return True
            if error.startswith("REPLAY_TIMEOUT"):
                self._last_error = error
                return False
            self.get_logger().warning(
                f"Cached replay failed ({error}); re-planning fresh"
            )
            # fall through to build + plan + execute below

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
            # Plan only — caller stores the serialized solution into the
            # trajectory cache so the next matching execute replays it.
            error = moveto_stage.init_and_plan(task, dry_run=True)
            if error is not None:
                self._last_error = error
                return False
            # Stash the serialized Solution msg (not the live Task — it holds a
            # RobotModel reference and dangles after a MoveIt relaunch) so
            # _execute() can cache it after this bool-returning call returns.
            self._last_planned_sol_msg = moveto_stage.last_sol_msg
            return True

        # Normal path: plan + execute end-to-end
        error = moveto_stage.load_plan_execute(task)
        if error is not None:
            self._last_error = error
            return False
        # Capture the freshly-planned solution so _execute() caches it for a
        # future identical move to replay instead of re-planning.
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
        # Use current gripper if not specified in task
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

        # Reset state for new goal
        self._vacuum.reset()
        self._last_detected_position = None
        self._last_detected_orientation = None

        result = MTCExecution.Result()
        feedback = MTCExecution.Feedback()

        # Parse and validate goal
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
        if dry_run:
            self.get_logger().info(
                f"DRY-RUN preview enabled — planning {task_count} step(s) "
                f"without moving the robot"
            )

        # Compute the trajectory-cache key from (current start joints, goal
        # payload, gripper). Current joints come from the live /joint_states
        # cache (None under mock hardware / before the first message → key
        # degrades to goal+gripper). The robot is at rest here, so these joints
        # are the move's start state; the same key recomputed on a later
        # identical move from the same start hits this entry.
        goal_key = PlanCache.compute_key(
            goal_handle.request.full_json,
            start_gripper,
            self._moveit_manager.current_arm_joints(),
        )

        # The cache lookup is deferred until after batching (below): only a goal
        # that groups into exactly ONE batched batch is cacheable, because a
        # single per-goal key cannot distinguish multiple batches' distinct
        # start/goal states.
        cached_plan_for_replay: dict | None = None

        # Initialize gripper state
        self._set_current_gripper(start_gripper)

        # Apply cup_profile override if parameter was changed via MCP. Set it
        # on the MoveIt manager instead of writing into self._grippers, which
        # is a reference into the shared, cached beamline config (see
        # config_loader.load_beamline_config). Empty/cleared param falls back to
        # the gripper's YAML cup_profile.
        self._moveit_manager.cup_override = (
            self.get_parameter("cup_profile").value or ""
        )

        # Step 1: Launch MoveIt for the gripper configuration
        self._update_feedback(
            feedback, goal_handle, 0, task_count, "Initializing MoveIt"
        )

        if not self._moveit_manager.launch_moveit_with_gripper(start_gripper):
            result.error_message = "Failed to initialize MoveIt stack"
            goal_handle.abort()
            return result

        # Publish running state before starting task execution
        self._publish_state("RUNNING")

        # Dry-run and live runs share grouping so previews replay against the
        # same batch structure.
        batches = group_into_batches(
            tasks,
            enabled=self._enable_batching,
        )
        self.get_logger().info(
            f"Grouped {task_count} tasks into {len(batches)} batches"
        )

        # Trajectory-cache eligibility: cache ONLY when the whole goal is a
        # single batched batch. A goal with any breaker (vision, pick/place,
        # pipettor, tool_exchange) or with batching disabled splits into
        # multiple batches that would all share this one per-goal key — storing
        # under it would let the last batch overwrite the first and replay the
        # wrong move (see #97 review). Those goals skip the cache and plan
        # fresh. This still covers the repeated single-move case (A->B->A) the
        # cache exists for. A single batched batch also implies no mid-goal tool
        # exchange, so start_gripper == _current_gripper throughout — the key's
        # gripper and the stored solution's gripper can't desync.
        cache_eligible = len(batches) == 1 and batches[0][0] == "batched"
        if cache_eligible and not dry_run:
            cached_plan_for_replay = self._plan_cache.get(goal_key)
            if cached_plan_for_replay is not None:
                self.get_logger().info(
                    "Trajectory cache hit — replaying stored plan without re-planning"
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
                result.error_message = (
                    f"MoveIt crashed before step {completed_tasks + 1}: {exit_info}"
                )
                self.get_logger().error(result.error_message)
                result.completed_steps = completed_tasks
                goal_handle.abort()
                return result

            if batch_type == "batched":
                # Execute batch of batchable tasks as single MTC Task
                batch_desc = ", ".join(t.get("task_type", "?") for t in batch_tasks)
                self._update_feedback(
                    feedback,
                    goal_handle,
                    completed_tasks + 1,
                    task_count,
                    f"batch[{batch_size}]: {batch_desc}",
                )

                # Controller activation is intentionally NOT done here: the
                # ur_control.launch.py spawner and the UR driver's
                # controller_stopper_node (which restarts controllers it stopped
                # on a connection drop) own it. An earlier in-orchestrator
                # activation helper raced the spawner ("already active" failures)
                # and was removed — re-add only with connection-drop recovery
                # stress-tested on Jazzy.

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

                # Cache the planned trajectory (serialized Solution msg) under
                # this key so a future identical move replays it instead of
                # re-planning. Only when cache_eligible (single batched batch),
                # so a multi-batch goal can never store distinct moves under one
                # shared key. Applies to both a dry-run preview and a fresh
                # execute. A cache *hit* replays without planning, leaving
                # _last_planned_sol_msg None, so this correctly skips re-storing
                # an entry that already exists — and never evicts it on execute.
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
                # Execute single task via action server (non-batchable)
                task = batch_tasks[0]
                task_type = task.get("task_type", "")

                if not task_type:
                    result.error_message = (
                        f"Step {completed_tasks} missing 'task_type' field"
                    )
                    result.completed_steps = completed_tasks
                    goal_handle.abort()
                    return result

                # Dry-run safety net: the parser already rejects unsupported
                # types, but if batching is disabled by parameter, even
                # supported types fall through to this single-task path which
                # would actually execute. Route them through _execute_batch
                # instead so dry_run is honored. This path is NOT cache_eligible
                # (a single/non-batched batch), so no trajectory is stored here —
                # the cache only operates on single-batched-batch goals.
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

                if not self._execute_step(task_type, task, poses_json):
                    if (
                        goal_handle.is_cancel_requested
                        or self._faulted
                        or self._last_error.startswith("REPLAY_TIMEOUT:")
                    ):
                        self._faulted = True
                    result.error_message = f"{task_type} failed: {self._last_error}"
                    result.completed_steps = completed_tasks
                    goal_handle.abort()
                    return result

                self._vacuum.update_after_tasks([task], self._current_gripper)
                completed_tasks += 1

            result.completed_steps = completed_tasks

            # Graceful cancellation finishes the active execution unit, then
            # stops before dispatching more work. This post-check is required
            # for the final batch, where there is no next boundary check.
            if goal_handle.is_cancel_requested:
                self.get_logger().warning(
                    f"Task cancelled after step {completed_tasks}/{task_count}"
                )
                result.error_message = "Task was canceled"
                goal_handle.canceled()
                return result

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

        self._update_feedback(feedback, goal_handle, task_count, task_count, "")
        goal_handle.succeed()
        self.get_logger().info("Orchestration goal completed successfully")
        return result

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
        self,
        client: ActionClient,
        goal,
        name: str,
        timeout: float,
        *,
        deadline: float | None = None,
    ) -> bool:
        """Send a goal to an action client and wait for result.

        Uses polling instead of spin_until_future_complete to avoid
        executor conflicts (this callback is already being spun by
        the MultiThreadedExecutor).

        On failure, stores error_message in self._last_error for propagation
        to the orchestrator result.
        """
        self._last_error = ""
        started_at = deadline - timeout if deadline is not None else None

        def remaining(default: float) -> float:
            return (
                max(0.0, deadline - time.monotonic())
                if deadline is not None
                else default
            )

        def timeout_error(stage: str) -> str:
            elapsed = (
                time.monotonic() - started_at if started_at is not None else timeout
            )
            return f"TIMEOUT: {name} stage={stage} elapsed={elapsed:.3f}s"

        # Wait for server
        server_timeout = remaining(5.0)
        if server_timeout <= 0 or not client.wait_for_server(
            timeout_sec=server_timeout
        ):
            self._last_error = timeout_error("server_discovery")
            self.get_logger().error(self._last_error)
            return False

        # Send goal and wait for acceptance
        if deadline is not None and hasattr(goal, "timeout"):
            goal.timeout = remaining(timeout)
            if goal.timeout <= 0:
                self._last_error = timeout_error("goal_submission")
                self.get_logger().error(self._last_error)
                return False
        send_future = client.send_goal_async(goal)
        acceptance_timeout = remaining(10.0)
        if acceptance_timeout <= 0 or not wait_for_future(
            send_future, timeout=acceptance_timeout
        ):
            self._faulted = True
            self._last_error = timeout_error("goal_acceptance")
            self.get_logger().error(self._last_error)
            return False

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self._last_error = f"{name} goal was rejected by action server"
            self.get_logger().error(self._last_error)
            return False

        # Wait for result with caller-provided timeout
        result_future = goal_handle.get_result_async()
        result_timeout = remaining(timeout)
        if result_timeout <= 0 or not wait_for_future(
            result_future, timeout=result_timeout
        ):
            self._last_error = timeout_error("result_wait")
            self.get_logger().error(self._last_error)
            self._publish_state("CANCELING")

            cancel_future = goal_handle.cancel_goal_async()
            if not wait_for_future(cancel_future, timeout=5.0):
                self._faulted = True
                self._last_error += "; cancellation acceptance timed out"
                return False

            canceling = getattr(cancel_future.result(), "goals_canceling", [])
            if not canceling and not result_future.done():
                self._faulted = True
                self._last_error += "; cancellation rejected while child active"
                return False

            # Cleanup is allowed to exceed the operation deadline. Once
            # cancellation is accepted, retain ownership until the child is
            # terminal instead of reporting IDLE while hardware is active.
            while not result_future.done():
                time.sleep(0.01)
            terminal = result_future.result()
            safe_statuses = {GoalStatus.STATUS_CANCELED}
            if not canceling:
                safe_statuses.add(GoalStatus.STATUS_SUCCEEDED)
            status = getattr(terminal, "status", None)
            if status is not None and status not in safe_statuses:
                self._faulted = True
                self._last_error += f"; child terminal status={status}"
            child_error = getattr(
                getattr(terminal, "result", None), "error_message", ""
            )
            if canceling and child_error and not any(
                marker in child_error for marker in ("TIMEOUT:", "CANCELED:")
            ):
                self._faulted = True
                self._last_error += f"; child cleanup failed: {child_error}"
            return False

        result = result_future.result()
        self._last_result = result.result
        if not result.result.success:
            # Capture the real error_message from the action server
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
            self._vision_scan_client, goal, "vision_scan", self._timeouts["vision_scan"]
        )

    # Preset aliases (issue #88): each legacy vision task_type expands to a set
    # of VisionTask params. The task JSON's own fields override the preset (merge
    # order in _call_vision_task), so {"task_type":"pick_sample","tag_id":5} works
    # the same as the fully-explicit {"task_type":"vision_task","detector":
    # "marker","goal_computer":"approach_pose","terminal_action":"grasp",...}.
    # Adding a vision task type is one entry here — no new handler.
    #   watchdog: "arm" after a successful grasp / "disarm" after release / "".
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

    # Task types routed to the unified vision pipeline: the canonical
    # "vision_task" (params straight from the JSON) plus the preset aliases.
    _VISION_TASK_TYPES = frozenset({"vision_task", *_VISION_PRESETS})

    def _call_vision_task(
        self, task_type: str, step: dict[str, Any], poses_json: str
    ) -> bool:
        """Single handler for every vision-guided task (issue #88).

        Builds one VisionTask goal from the preset for `task_type` (or, for
        task_type=="vision_task", straight from the step), sends it to the one
        vision_task server, then runs the only two things that must stay
        orchestrator-side: detect_only pose caching and the cross-task vacuum
        watchdog (it outlives the goal — armed at pick, checked through
        transport, disarmed at place). ALL motion is server-side.
        """
        preset = self._VISION_PRESETS.get(task_type, {})

        # Hardcoded (non-vision) pick/place is a detection-free joint sequence —
        # it doesn't fit the detect-first pipeline, so it stays on the legacy
        # sample server.
        if task_type in ("pick_sample", "place_sample") and not step.get(
            "use_vision", True
        ):
            return (
                self._call_pick_sample_hardcoded
                if task_type == "pick_sample"
                else self._call_place_sample_hardcoded
            )(step, poses_json)

        # Merge: preset defaults < explicit task-JSON fields.
        cfg = {**preset, **step}

        goal = self._build_vision_goal(cfg, poses_json)
        timeout = goal.timeout
        if not math.isfinite(timeout) or timeout <= 0:
            self._last_error = (
                "PIPELINE_CONFIG_ERROR: timeout must be positive and finite"
            )
            self.get_logger().error(self._last_error)
            return False
        started_at = time.monotonic()
        deadline = started_at + timeout

        # Settle (orchestrator-side, before any server motion), capped at 10s.
        settle_time = min(float(cfg.get("settle_time", 1.0)), 10.0)
        if settle_time > 0:
            self.get_logger().info(f"Waiting {settle_time:.1f}s for robot to settle...")
            time.sleep(min(settle_time, max(0.0, deadline - time.monotonic())))
            if time.monotonic() >= deadline:
                self._last_error = (
                    "TIMEOUT: vision_task stage=orchestrator_settle "
                    f"elapsed={time.monotonic() - started_at:.3f}s"
                )
                self.get_logger().error(self._last_error)
                return False

        success = self._send_and_wait(
            self._vision_task_client,
            goal,
            task_type,
            timeout,
            deadline=deadline,
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

        # Cross-task vacuum watchdog (ePick only). The server reports vacuum_ok;
        # the watchdog state persists across the transport steps between pick and
        # place, so it must live here, not in the per-goal server.
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
        """Assemble a VisionTask goal from a merged preset+step config dict.

        Orchestrator-owned fields (gripper config, IK frame) are filled here;
        everything else comes from cfg with sane defaults.
        """
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
        goal.timeout = float(cfg.get("timeout", self._timeouts["vision_task"]))
        goal.strategy = cfg.get("strategy", "")
        goal.edge_inset_mm = float(cfg.get("edge_inset_mm", 0.0))
        # Goal-computation inputs.
        goal.z_offset = float(cfg.get("z_offset", self._gripper_z_offset()))
        goal.marker_offset_x = float(cfg.get("marker_offset_x", 0.0))
        goal.marker_offset_y = float(cfg.get("marker_offset_y", 0.0))
        goal.marker_offset_z = float(cfg.get("marker_offset_z", 0.0))
        goal.offset_direction = cfg.get("offset_direction", "")
        goal.offset_distance = float(cfg.get("offset_distance", 0.0))
        # j6_snap (spincoater): accept legacy place_pose/pickup_pose for target.
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
        # retreat_from_scan: pick/place retreat to the scan pose after the action.
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

        # Multi-position scan averaging (vision_moveto): flatten pose keys -> radians.
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


if __name__ == "__main__":
    main()
