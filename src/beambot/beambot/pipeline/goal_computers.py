"""Built-in goal computers for the vision-task pipeline (issue #88).

A goal computer turns a detection into a MotionTarget. It is PURE w.r.t. the
robot — it reads the detection + ctx and emits a target; it does not plan or
move (that's the executor's job). Keeping compute and execute separate is what
makes a computer swappable and (for the future j6_snap) unit-testable.

Contract: compute(detection, ctx) -> MotionTarget | None
           (None = detect_only short-circuit; the server returns the pose.)
"""

import json

from beambot.pipeline.motion_target import CartesianTarget, JointTarget, snap_j6
from beambot.pipeline.registry import register_goal_computer


@register_goal_computer("approach_pose")
def compute_approach_pose(detection, ctx):
    """Detection pose -> 6-DOF CartesianTarget.

    Delegates the geometry to VisionEngine.compute_approach_pose (marker-frame
    offset, z_offset, straight-down orientation) and _apply_flange_offset — the
    exact computation vision_moveto/pick/place use today. Emits a CartesianTarget
    the executor runs via IK -> Pilz-PTP with Pilz-LIN fallback.
    """
    vision = ctx.vision
    goal = ctx.goal

    approach, active_ik_frame = vision.compute_approach_pose(
        detection,
        goal.z_offset,
        marker_offset_x=goal.marker_offset_x,
        marker_offset_y=goal.marker_offset_y,
        marker_offset_z=goal.marker_offset_z,
        ik_frame_override=goal.ik_frame or "",
    )

    if goal.offset_direction and goal.offset_distance > 0:
        approach = vision._apply_flange_offset(
            approach, goal.offset_direction, goal.offset_distance
        )

    if goal.detect_only:
        ctx.detect_only_pose = approach
        pos = approach.pose.position
        vision.logger.info(
            f"Detect-only: returning approach pose "
            f"[{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}] in {approach.header.frame_id}"
        )
        return None

    # Optional fused grasp/retreat tail (pick/place). terminal_action names the
    # SRDF state key in gripper_states_json; the executor builds the one fused
    # task. vision_moveto leaves terminal_action empty -> bare approach.
    grasp_state = ""
    terminal = getattr(goal, "terminal_action", "") or ""
    if terminal:
        states = (
            json.loads(goal.gripper_states_json) if goal.gripper_states_json else {}
        )
        grasp_state = states.get(
            terminal, terminal
        )  # accept state-key or raw SRDF name

    return CartesianTarget(
        pose=approach,
        ik_frame=active_ik_frame,
        grasp_state=grasp_state,
        gripper_group=getattr(goal, "gripper_group", "") or "",
        retreat_pose_key=getattr(goal, "retreat_pose", "") or "",
    )


@register_goal_computer("j6_snap")
def compute_j6_snap(detection, ctx):
    """Detected angle -> JointTarget with joint 6 corrected (spincoater).

    Reads angle_mod90 from the detection dict, the base pose's joint 6 from the
    pose registry (goal.target_pose, in degrees), applies the pure snap_j6
    correction, and emits a JointTarget executed verbatim (NO IK). This is the
    unified replacement for the byte-identical j6 math in both spincoater
    handlers.
    """
    goal = ctx.goal
    vision = ctx.vision

    base_pose_key = goal.target_pose
    poses = json.loads(goal.poses_json) if goal.poses_json else {}
    base_joints = poses.get(base_pose_key)
    if base_joints is None:
        ctx.error = (
            f"j6_snap: base pose '{base_pose_key}' not found in poses "
            f"(available: {sorted(poses)})"
        )
        return None

    angle = detection["angle_mod90"]
    base_j6 = base_joints[5]
    corrected_j6 = snap_j6(base_j6, angle, goal.k_offset)

    vision.logger.info(
        f"j6_snap: base={base_j6:.1f}°, angle={angle:.1f}°, "
        f"k={goal.k_offset:.1f}° -> target_j6={corrected_j6:.1f}°"
    )

    corrected = list(base_joints)
    corrected[5] = corrected_j6

    # Optional tail: forward-contact move + terminal gripper (spincoater).
    terminal = getattr(goal, "terminal_action", "") or ""
    terminal_state = ""
    if terminal:
        states = (
            json.loads(goal.gripper_states_json) if goal.gripper_states_json else {}
        )
        terminal_state = states.get(terminal, terminal)  # state-key or raw SRDF name

    return JointTarget(
        joints_deg=corrected,
        forward_distance=getattr(goal, "forward_distance", 0.0),
        terminal_state=terminal_state,
        gripper_group=getattr(goal, "gripper_group", "") or "",
    )
