"""Motion targets — what a GoalComputer emits and the one executor consumes (#88).

"Execute the goal" is NOT one operation in disguise. Each target is the NATIVE
representation of one existing motion path, and the executor dispatches on type
(isinstance) and never down-converts:

  - CartesianTarget : a 6-DOF base_link pose -> deterministic IK -> Pilz-PTP,
                      with a Pilz-LIN-from-pose fallback. Needs TF + IK.
  - JointTarget     : a literal joint vector executed verbatim. NO IK, NO TF —
                      how the spincoater dodges KDL IK jitter (#51). It has no
                      pose field, so nothing can be tempted to FK-rotate-IK it.

The set is closed on purpose: motion mechanisms are bounded and safety-critical.
A genuinely new one (force-servoed, dual-arm) is a deliberate core edit (new
class + new executor arm), not a plugin registration — the OPEN/churning axes
are the detectors and goal_computers. A pipettor SequenceTarget would land the
same way if/when that migration happens.
"""

from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped


@dataclass(frozen=True)
class CartesianTarget:
    """A 6-DOF approach pose in base_link, executed via IK -> Pilz-PTP with a
    Pilz-LIN-from-pose fallback. `ik_frame` is the gripper tip frame; empty =
    auto-detect from TF.

    Optional fused tail (pick/place): when grasp_state is set, the executor
    builds ONE MTC task — approach + gripper(grasp_state) + retreat(retreat_pose)
    — matching the verified pick/place trajectory exactly (a single planned
    motion, not three separate plan/execute cycles). When grasp_state is empty
    (vision_moveto), it's a bare approach move.

    The grasp/retreat data rides WITH the target so the executor can plan the
    whole sequence in one task; the gripper planner + poses come from the ctx
    the executor already holds.
    """

    kind = "cartesian"
    pose: PoseStamped
    ik_frame: str = ""
    # Fused tail (pick/place). All optional; empty/None => bare approach.
    grasp_state: str = ""  # SRDF gripper state to actuate after approach
    gripper_group: str = ""  # MoveIt gripper group for the gripper stage
    retreat_pose_key: str = ""  # pose key to retreat to after grasp ("" => none)


@dataclass(frozen=True)
class JointTarget:
    """A literal joint vector (degrees) executed VERBATIM — no IK, no TF.

    This is the spincoater path: a hardcoded place/pick pose with joint 6
    corrected by the detected angle. It has NO pose field by design, so no code
    can FK-rotate-IK it back into the jitter that the joint-space move exists to
    avoid (#51). The executor runs it through make_move_to_named_stage(
    planner=None) -> the Pilz-PTP -> OMPL fallback, matching planning_type=
    "joint" exactly.

    Optional tail (spincoater): after the corrected joint move, move forward to
    contact (forward_distance) then actuate the gripper (terminal_state). All
    optional; defaults => bare joint move.
    """

    kind = "joints"
    joints_deg: list
    forward_distance: float = (
        0.0  # meters to move forward after positioning (0 => none)
    )
    terminal_state: str = ""  # SRDF gripper state to actuate at the end ("" => none)
    gripper_group: str = ""  # MoveIt gripper group for the terminal stage


def snap_j6(base_j6_deg: float, angle_deg: float, k_offset_deg: float = 0.0) -> float:
    """Correct joint 6 for a detected 4-fold-symmetric feature (spincoater).

    The pocket/sample is square (90° symmetry), so the minimal rotation that
    aligns the gripper is the detected angle reduced into (-45, 45]:
        correction = (angle + k_offset) mod 90, shifted into (-45, 45]
        corrected_j6 = base_j6 + correction

    Pure function — byte-for-byte the math that was duplicated in both
    _call_pick_spincoater and _call_place_spincoater, now tested in isolation.

    Args:
        base_j6_deg: joint-6 value of the hardcoded place/pick pose (degrees).
        angle_deg: detected feature angle, expected in [0, 90) (angle_mod90).
        k_offset_deg: calibration constant (degrees).

    Returns:
        Corrected joint-6 value in degrees.
    """
    correction = (angle_deg + k_offset_deg) % 90
    if correction > 45:
        correction -= 90
    return base_j6_deg + correction
