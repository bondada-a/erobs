"""VisionTaskStages — the unified vision pipeline's run() entry point (issue #88).

Fixed pipeline: (pre-scan) -> settle -> DETECT -> COMPUTE-GOAL -> EXECUTE ->
(vacuum check). The DETECT and COMPUTE-GOAL stages are name-keyed plugins;
EXECUTE is one dispatch over the MotionTarget union. All motion code is LIFTED
from VisionEngine / the pick-place stages, never rewritten — so the #51 IK-jitter
dodge, the Pilz PTP->LIN approach fallback, and the fused approach+grasp+retreat
trajectory behave identically to the legacy handlers.

Wires three migrations:
  vision_moveto  : marker/sample_roi -> approach_pose -> CartesianTarget (bare)
  spincoater     : spincoater_* -> j6_snap -> JointTarget (no IK)
  pick/place     : marker/sample_roi -> approach_pose -> CartesianTarget with a
                   fused grasp+retreat tail; orchestrator arms the vacuum
                   watchdog from result.vacuum_ok.
"""

import json
import threading
from dataclasses import dataclass, field
from typing import Any

from geometry_msgs.msg import PoseStamped
from moveit.task_constructor import core, stages

import beambot.pipeline  # noqa: F401 — registers built-in plugins on import
from beambot.pipeline.motion_target import CartesianTarget, JointTarget
from beambot.pipeline.registry import get_detector, get_goal_computer
from beambot.pipeline.vision_engine import VisionEngine
from beambot.stages.base_stages import apply_constraints, parse_constraints


@dataclass
class VisionTaskContext:
    """Everything a detector / goal_computer needs, built once per goal."""

    goal: Any
    vision: VisionEngine
    scan_positions: list | None = None
    # Set by a goal_computer when detect_only short-circuits; read back by run().
    detect_only_pose: Any = field(default=None)
    # Set by a goal_computer to report a hard failure (distinct from a None
    # target meaning "cache-only / detect_only, succeed without moving").
    error: str | None = field(default=None)


class VisionTaskStages:
    """Runs the unified vision pipeline. Owns a VisionEngine for delegation."""

    def __init__(self, rclpy_node, **vision_kwargs):
        self.rclpy_node = rclpy_node
        self.logger = rclpy_node.get_logger()
        self._vision = VisionEngine(rclpy_node, **vision_kwargs)
        # Surfaced to the server's _execute for result population.
        self.last_detected_pose = None
        self.vacuum_ok = True
        self.goal = None  # current goal, set per-run for executor helpers

    # ----- TF reset passthrough (parity with VisionActionServer) -------------
    def reset_tf(self):
        self._vision.reset_tf()

    # ----- pipeline ----------------------------------------------------------
    def run(self, goal) -> "str | None":
        """Execute the pipeline. Returns None on success, an error string else."""
        self.last_detected_pose = None
        self.vacuum_ok = True
        self.goal = goal
        vision = self._vision

        # Stage 0a: optional pre-scan move (pick/place fuse open-gripper + move
        # to scan pose here; vision_moveto/spincoater leave scan_pose empty
        # because the orchestrator already positioned the arm).
        error = self._move_to_scan(goal)
        if error is not None:
            return error

        # Stage 0b: settle (vibration damping before capture).
        if vision._settle_time > 0:
            self.logger.info(
                f"Waiting {vision._settle_time:.2f}s for robot to settle..."
            )
            import time

            time.sleep(vision._settle_time)
            self.logger.info("Settle complete, starting detection")

        ctx = VisionTaskContext(goal=goal, vision=vision)
        ctx.scan_positions = self._parse_scan_positions(goal)

        # Stage 1: DETECT (plugin)
        detector_name = goal.detector or "marker"
        goal_computer_name = goal.goal_computer or "approach_pose"
        try:
            detector = get_detector(detector_name)
            goal_computer = get_goal_computer(goal_computer_name)
        except KeyError as e:
            return f"PIPELINE_CONFIG_ERROR: {e}"

        detection = detector(ctx)
        if detection is None:
            return (
                f"DETECTION_FAILED: detector '{detector_name}' found nothing "
                f"(tag {goal.tag_id}, timeout {goal.timeout}s)"
            )

        # Stage 2: COMPUTE-GOAL (plugin) -> MotionTarget | None
        target = goal_computer(detection, ctx)

        if ctx.error is not None:
            return f"GOAL_COMPUTE_FAILED: {ctx.error}"
        # detect_only short-circuit: computer returned None and stashed the pose.
        if ctx.detect_only_pose is not None:
            self.last_detected_pose = ctx.detect_only_pose
            return None
        if target is None:
            return None  # cache-only / nothing to execute

        # Stage 3: EXECUTE — one dispatch over the MotionTarget union.
        error = self._execute_motion_target(target)
        if error is not None:
            return error

        # Stage 4: post-grasp vacuum check (pick only — when a grasp happened on
        # an ePick). Mirrors PickSampleStages: never aborts, just reports.
        if isinstance(target, CartesianTarget) and target.grasp_state:
            self.vacuum_ok = self._check_vacuum()
            self.logger.info(f"vacuum_ok={self.vacuum_ok}")
        return None

    def _execute_motion_target(self, target) -> "str | None":
        """Dispatch on the union tag. Each arm LIFTS existing motion code.

        Adding a new arm (e.g. SequenceTarget for pipettor) is a new migration,
        not a rewrite of these.
        """
        if isinstance(target, CartesianTarget):
            return self._execute_cartesian(target)

        if isinstance(target, JointTarget):
            return self._execute_joints(target)

        return f"PIPELINE_ERROR: no executor arm for {type(target).__name__}"

    def _execute_joints(self, target: JointTarget) -> "str | None":
        """Corrected joint move (NO IK), then optional forward-contact + terminal.

        The three steps are SEPARATE plan/execute cycles, matching the legacy
        spincoater handlers exactly (they were three orchestrator calls). The
        joint move uses make_move_to_named_stage(planner=None) — the Pilz-PTP ->
        OMPL fallback, byte-for-byte the path planning_type="joint" takes, so the
        #51 jitter dodge is preserved (no pose, no IK).
        """
        vision = self._vision

        # Step 1: corrected joint move (verbatim, no IK).
        pose_key = "_vision_task_joint_target"
        poses = {pose_key: list(target.joints_deg)}
        task = vision.create_task_template("Vision Task Joint Move")
        stage = vision.make_move_to_named_stage(
            "corrected joint move", pose_key, poses, planner=None
        )
        if stage is None:
            return "PIPELINE_ERROR: failed to build joint move stage"
        task.add(stage)
        error = vision.load_plan_execute(task)
        if error:
            return f"corrected joint move failed: {error}"

        # Step 2: forward-contact move (optional).
        if target.forward_distance > 0:
            self.logger.info(f"moving forward {target.forward_distance * 1000:.1f}mm")
            fwd_task = vision.create_task_template("Vision Task Forward")
            fwd_stage = vision.create_relative_move_stage(
                "forward contact",
                "forward",
                target.forward_distance,
            )
            fwd_task.add(fwd_stage)
            error = vision.load_plan_execute(fwd_task)
            if error:
                return f"forward move failed: {error}"

        # Step 3: terminal gripper action (optional).
        if target.terminal_state:
            self.logger.info(f"terminal gripper: {target.terminal_state}")
            term_task = vision.create_task_template("Vision Task Terminal")
            term_stage = vision.make_gripper_stage(
                "terminal",
                vision.make_joint_interpolation_planner(),
                target.gripper_group,
                target.terminal_state,
            )
            if term_stage:
                term_task.add(term_stage)
                error = vision.load_plan_execute(term_task)
                if error:
                    return f"terminal action failed: {error}"

        return None

    def _execute_cartesian(self, target: CartesianTarget) -> "str | None":
        """Approach via IK->PTP with LIN fallback, optionally fused with a
        grasp + retreat into ONE MTC task.

        Lifted verbatim from PickSampleStages/PlaceSampleStages Task-2: the
        approach Fallbacks(PTP joint goal -> LIN cartesian), the gripper stage,
        and the retreat are added to a single task so MoveIt plans+executes one
        continuous trajectory — preserving the verified pick/place motion. When
        grasp_state is empty (vision_moveto) this is a bare approach move.
        """
        vision = self._vision
        goal = self.goal
        constraints = parse_constraints(
            json.loads(goal.constraints_json)
            if getattr(goal, "constraints_json", "")
            else None
        )

        # Bare approach (vision_moveto): no gripper, no retreat — delegate to the
        # exact existing path so behavior is byte-for-byte unchanged.
        if not target.grasp_state and not target.retreat_pose_key:
            return vision._move_to_approach(target.pose, ik_frame=target.ik_frame)

        # Fused task: approach + gripper + retreat (pick/place).
        joint_goal = vision.compute_deterministic_ik(target.pose, target.ik_frame)
        task = vision.create_task_template("Vision Task Grasp")

        approach_fb = core.Fallbacks("approach")
        if joint_goal is not None:
            ptp = stages.MoveTo("approach [PTP]", vision.make_pilz_planner("PTP"))
            ptp.group = vision.arm_group
            vision._set_ik_frame(ptp)
            ptp.setGoal(joint_goal)
            apply_constraints(ptp, constraints)
            approach_fb.add(ptp)
        lin = stages.MoveTo("approach [LIN]", vision.make_pilz_planner("LIN"))
        lin.group = vision.arm_group
        ik_frame_pose = PoseStamped()
        ik_frame_pose.header.frame_id = target.ik_frame
        lin.ik_frame = ik_frame_pose
        lin.setGoal(target.pose)
        apply_constraints(lin, constraints)
        approach_fb.add(lin)
        task.add(approach_fb)

        if target.grasp_state:
            grasp = vision.make_gripper_stage(
                "gripper",
                vision.make_joint_interpolation_planner(),
                target.gripper_group,
                target.grasp_state,
            )
            if grasp:
                task.add(grasp)

        if target.retreat_pose_key:
            poses = vision.parse_poses(goal.poses_json) or {}
            retreat = vision.make_move_to_named_stage(
                "retreat",
                target.retreat_pose_key,
                poses,
                constraints=constraints,
            )
            if not retreat:
                return (
                    f"Pose '{target.retreat_pose_key}' not found or invalid (retreat)"
                )
            task.add(retreat)

        return vision.load_plan_execute(task)

    def _move_to_scan(self, goal) -> "str | None":
        """Optional pre-detection move: open gripper (pick) then move to scan.

        Only runs when goal.scan_pose is set — pick/place fuse positioning here;
        vision_moveto/spincoater leave it empty (orchestrator already positioned).
        """
        scan_pose = getattr(goal, "scan_pose", "") or ""
        if not scan_pose:
            return None

        vision = self._vision
        poses = vision.parse_poses(goal.poses_json)
        if poses is None:
            return "Failed to parse poses_json"
        constraints = parse_constraints(
            json.loads(goal.constraints_json)
            if getattr(goal, "constraints_json", "")
            else None
        )
        states = (
            json.loads(goal.gripper_states_json)
            if getattr(goal, "gripper_states_json", "")
            else {}
        )

        task = vision.create_task_template("Position for Task")
        if getattr(goal, "pre_open", False) and states.get("release"):
            open_stage = vision.make_gripper_stage(
                "open gripper",
                vision.make_joint_interpolation_planner(),
                goal.gripper_group,
                states["release"],
            )
            if open_stage:
                task.add(open_stage)
        scan_stage = vision.make_move_to_named_stage(
            "scan position",
            scan_pose,
            poses,
            constraints=constraints,
        )
        if not scan_stage:
            return f"Pose '{scan_pose}' not found or invalid (scan position)"
        task.add(scan_stage)
        error = vision.load_plan_execute(task)
        return f"Position failed: {error}" if error else None

    def _check_vacuum(self) -> bool:
        """Lifted from PickSampleStages: one-shot /object_detection_status read.

        Returns True if an object is detected or no ePick is connected.
        """
        try:
            from epick_msgs.msg import ObjectDetectionStatus
        except ImportError:
            return True

        msg_holder = [None]
        event = threading.Event()

        def _on_status(msg):
            msg_holder[0] = msg
            event.set()

        sub = self.rclpy_node.create_subscription(
            ObjectDetectionStatus,
            "/object_detection_status",
            _on_status,
            10,
        )
        event.wait(timeout=1.0)
        self.rclpy_node.destroy_subscription(sub)

        if msg_holder[0] is None:
            return True  # No ePick connected
        NO_OBJECT = 3
        detected = msg_holder[0].status != NO_OBJECT
        if not detected:
            self.logger.warning(
                "VACUUM_LOST: ePick reports NO_OBJECT_DETECTED after pick"
            )
        return detected

    def _parse_scan_positions(self, goal) -> "list | None":
        """Multi-position averaging: unflatten [j1..j6, ...] into [[6], ...]."""
        num = getattr(goal, "num_scan_positions", 0)
        if num <= 0:
            return None
        flat = list(getattr(goal, "scan_positions_flat", []))
        if len(flat) != num * 6:
            self.logger.warning(
                f"Invalid scan_positions_flat length: {len(flat)}, expected "
                f"{num * 6}. Falling back to single-position."
            )
            return None
        self.logger.info(f"Multi-position mode enabled: {num} scan positions")
        return [flat[i * 6 : (i + 1) * 6] for i in range(num)]
