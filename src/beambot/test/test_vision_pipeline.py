"""Tests for the unified vision pipeline plumbing (issue #88).

Hardware-free: the detector/computer plugins delegate to VisionEngine (which
needs ROS + a camera), so these cover the parts that DON'T — registry lookup,
the MotionTarget union, and the detect/compute/execute dispatch in
VisionTaskStages.run() — with a fake vision handle. The real motion path is
gated by hardware verification, by design.
"""

import types

import pytest

# Runs under a sourced ROS 2 env (like the repo's other tests via colcon test):
# motion_target imports geometry_msgs.PoseStamped, and the dispatch tests import
# VisionTaskStages -> VisionEngine, which needs ROS message types at import time.
from beambot.pipeline.motion_target import CartesianTarget, JointTarget, snap_j6
from beambot.pipeline.registry import (
    get_detector,
    get_goal_computer,
    register_detector,
)


def test_unknown_detector_raises_with_available_keys():
    """A typo must fail loudly listing the registered keys, not silently no-op."""
    with pytest.raises(KeyError) as exc:
        get_detector("marrker")
    assert "marrker" in str(exc.value)
    assert "marker" in str(exc.value)  # the real one is listed


def test_unknown_goal_computer_raises():
    with pytest.raises(KeyError) as exc:
        get_goal_computer("nope")
    assert "approach_pose" in str(exc.value)


def test_builtin_plugins_registered():
    """Importing beambot.pipeline must register the v1 built-ins."""
    import beambot.pipeline  # noqa: F401

    assert get_detector("marker") is not None
    assert get_detector("sample_roi") is not None
    assert get_goal_computer("approach_pose") is not None


def test_duplicate_registration_rejected():
    with pytest.raises(ValueError):

        @register_detector("marker")  # already taken
        def _dupe(ctx):
            return None


def test_cartesian_target_carries_pose_and_frame():
    """The only v1 motion variant: a pose + ik_frame, tagged 'cartesian'."""
    sentinel = object()
    t = CartesianTarget(pose=sentinel, ik_frame="epick_tip")
    assert t.kind == "cartesian"
    assert t.pose is sentinel
    assert t.ik_frame == "epick_tip"


# ---- VisionTaskStages dispatch, with a fake VisionEngine (no ROS) ----------


class _FakeVision:
    """Stands in for VisionEngine: records calls, returns canned values."""

    def __init__(self):
        self._settle_time = 0.0  # skip the sleep
        self.logger = types.SimpleNamespace(
            info=lambda *a, **k: None, warning=lambda *a, **k: None
        )
        self.moved_to = None
        self.joint_moved_to = None
        self.joint_planner = "UNSET"
        self.detect_return = "DETECTION"
        # pose-shaped stub: detect_only logging reads approach.pose.position.{x,y,z}
        _position = types.SimpleNamespace(x=0.1, y=0.2, z=0.3)
        self.approach_pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(position=_position),
            header=types.SimpleNamespace(frame_id="base_link"),
        )
        self.approach_return = (self.approach_pose, "epick_tip")

    # detector delegates
    def get_cached_pose(self, tag_id):
        return None

    def detect_and_transform_tag(self, tag_id, timeout):
        return self.detect_return

    # computer delegates
    def compute_approach_pose(self, detection, z_offset, **kw):
        return self.approach_return

    def _apply_flange_offset(self, approach, direction, distance):
        return approach

    # cartesian executor delegate (bare approach path)
    def _move_to_approach(self, pose, ik_frame=""):
        self.moved_to = (pose, ik_frame)
        return None  # success

    # fused-task executor delegates (pick/place grasp+retreat path)
    arm_group = "ur_arm"

    def compute_deterministic_ik(self, pose, ik_frame):
        return {"shoulder_pan_joint": 0.0}  # non-None -> PTP arm added

    def make_pilz_planner(self, mode):
        return object()

    def make_joint_interpolation_planner(self):
        return object()

    def _set_ik_frame(self, stage):
        pass

    def parse_poses(self, poses_json):
        import json as _json

        return _json.loads(poses_json) if poses_json else {}

    def make_gripper_stage(self, label, planner, group, state):
        self.gripper_actions = getattr(self, "gripper_actions", [])
        self.gripper_actions.append(state)
        return object() if state else None

    # joint executor delegates (JointTarget path)
    def create_task_template(self, name):
        self.tasks_built = getattr(self, "tasks_built", [])
        self.tasks_built.append(name)
        return types.SimpleNamespace(add=lambda stage: None)

    def make_move_to_named_stage(
        self, label, pose_key, poses, planner=None, constraints=None
    ):
        # Record the joint vector the executor handed us (the corrected pose).
        self.joint_moved_to = poses[pose_key]
        self.joint_planner = planner  # must be None -> Pilz-PTP→OMPL fallback
        self.named_stages = getattr(self, "named_stages", [])
        self.named_stages.append((label, pose_key))
        return object()  # non-None stage

    def load_plan_execute(self, task):
        return None  # success


# MoveTo / Fallbacks are real moveit_task_constructor types the fused executor
# builds. They construct fine under a sourced ROS env; the fake's task.add is a
# no-op so nothing is planned. No stubbing needed.


def _make_stages(fake):
    """Build a VisionTaskStages without running its real __init__ (needs ROS)."""
    from beambot.stages.vision_task_stages import VisionTaskStages

    s = VisionTaskStages.__new__(VisionTaskStages)
    s.logger = fake.logger
    s._vision = fake
    s.rclpy_node = types.SimpleNamespace(
        create_subscription=lambda *a, **k: object(),
        destroy_subscription=lambda *a, **k: None,
    )
    s.last_detected_pose = None
    s.vacuum_ok = True
    s.goal = None
    return s


def _goal(**over):
    g = types.SimpleNamespace(
        detector="marker",
        goal_computer="approach_pose",
        tag_id=7,
        sample_index=1,
        timeout=5.0,
        z_offset=0.0,
        detect_only=False,
        offset_direction="",
        offset_distance=0.0,
        marker_offset_x=0.0,
        marker_offset_y=0.0,
        marker_offset_z=0.0,
        ik_frame="",
        strategy="",
        edge_inset_mm=0.0,
        num_scan_positions=0,
        scan_positions_flat=[],
        target_pose="",
        k_offset=0.0,
        poses_json="",
        terminal_action="",
        gripper_group="",
        gripper_states_json="",
        pre_open=False,
        retreat_pose="",
        scan_pose="",
        constraints_json="",
    )
    g.__dict__.update(over)
    return g


def test_run_happy_path_executes_cartesian():
    """marker -> approach_pose -> CartesianTarget -> _move_to_approach."""
    fake = _FakeVision()
    stages = _make_stages(fake)
    err = stages.run(_goal())
    assert err is None
    assert fake.moved_to == (
        fake.approach_pose,
        "epick_tip",
    )  # executor lifted the pose


def test_run_detection_failed_does_not_move():
    fake = _FakeVision()
    fake.detect_return = None
    stages = _make_stages(fake)
    err = stages.run(_goal())
    assert err is not None and "DETECTION_FAILED" in err
    assert fake.moved_to is None


def test_run_detect_only_returns_pose_without_moving():
    fake = _FakeVision()
    stages = _make_stages(fake)
    err = stages.run(_goal(detect_only=True))
    assert err is None
    assert stages.last_detected_pose is fake.approach_pose
    assert fake.moved_to is None  # no motion on detect_only


def test_run_unknown_detector_is_config_error():
    fake = _FakeVision()
    stages = _make_stages(fake)
    err = stages.run(_goal(detector="bogus"))
    assert err is not None and "PIPELINE_CONFIG_ERROR" in err
    assert fake.moved_to is None


def test_scan_positions_parsed_when_valid():
    fake = _FakeVision()
    stages = _make_stages(fake)
    # 2 positions * 6 joints = 12 values
    flat = [float(i) for i in range(12)]
    parsed = stages._parse_scan_positions(
        _goal(num_scan_positions=2, scan_positions_flat=flat)
    )
    assert parsed == [[0.0, 1.0, 2.0, 3.0, 4.0, 5.0], [6.0, 7.0, 8.0, 9.0, 10.0, 11.0]]


def test_scan_positions_rejected_on_length_mismatch():
    fake = _FakeVision()
    stages = _make_stages(fake)
    parsed = stages._parse_scan_positions(
        _goal(num_scan_positions=2, scan_positions_flat=[1.0, 2.0])
    )
    assert parsed is None  # falls back to single-position


# ---- snap_j6: the previously-untested, byte-duplicated spincoater math ------
# Reduces a detected 4-fold-symmetric angle into the minimal (-45, 45] rotation
# and adds it to the base joint-6. base_j6=0 isolates the correction term.


class TestSnapJ6:
    def test_zero_angle_no_correction(self):
        assert snap_j6(100.0, 0.0) == 100.0

    def test_small_angle_added_directly(self):
        assert snap_j6(0.0, 30.0) == pytest.approx(30.0)

    def test_exactly_45_stays_positive(self):
        # 45 is the boundary: not > 45, so no wrap.
        assert snap_j6(0.0, 45.0) == pytest.approx(45.0)

    def test_just_over_45_wraps_negative(self):
        # 46 mod 90 = 46 > 45 -> 46 - 90 = -44 (rotate the short way).
        assert snap_j6(0.0, 46.0) == pytest.approx(-44.0)

    def test_near_90_is_small_negative(self):
        assert snap_j6(0.0, 89.0) == pytest.approx(-1.0)

    def test_90_folds_to_zero(self):
        # 90 mod 90 = 0 (full 4-fold symmetry period).
        assert snap_j6(10.0, 90.0) == pytest.approx(10.0)

    def test_base_j6_offset_applied(self):
        assert snap_j6(-163.0, 30.0) == pytest.approx(-133.0)

    def test_k_offset_shifts_before_snap(self):
        # (40 + 10) mod 90 = 50 > 45 -> 50 - 90 = -40.
        assert snap_j6(0.0, 40.0, k_offset_deg=10.0) == pytest.approx(-40.0)

    def test_matches_legacy_inline_formula(self):
        # Cross-check against the exact arithmetic both old handlers ran.
        for base in (-163.11, 0.0, 90.0):
            for angle in (0.0, 12.3, 44.9, 45.1, 73.0, 89.99):
                for k in (0.0, 5.0):
                    raw = angle + k
                    corr = raw % 90
                    if corr > 45:
                        corr -= 90
                    assert snap_j6(base, angle, k) == pytest.approx(base + corr)


# ---- j6_snap goal computer + JointTarget executor (spincoater path) ---------


def test_j6_snap_emits_jointtarget_with_corrected_j6():
    """spincoater detection dict -> JointTarget; joint 6 snapped, NO IK."""
    import json
    from beambot.pipeline.goal_computers import compute_j6_snap

    base = [10.0, 20.0, 30.0, 40.0, 50.0, -163.0]
    ctx = types.SimpleNamespace(
        goal=_goal(
            target_pose="spincoater_place",
            k_offset=0.0,
            poses_json=json.dumps({"spincoater_place": base}),
        ),
        vision=_FakeVision(),
        error=None,
    )
    target = compute_j6_snap({"angle_mod90": 30.0}, ctx)
    assert isinstance(target, JointTarget)
    # joints 0-4 untouched; joint 5 corrected by +30.
    assert target.joints_deg[:5] == base[:5]
    assert target.joints_deg[5] == pytest.approx(-133.0)


def test_j6_snap_missing_base_pose_sets_error():
    from beambot.pipeline.goal_computers import compute_j6_snap

    ctx = types.SimpleNamespace(
        goal=_goal(target_pose="nonexistent", poses_json="{}"),
        vision=_FakeVision(),
        error=None,
    )
    target = compute_j6_snap({"angle_mod90": 10.0}, ctx)
    assert target is None
    assert ctx.error is not None and "nonexistent" in ctx.error


def test_jointtarget_executor_moves_verbatim_no_ik():
    """The JointTarget arm hands the raw joint vector to make_move_to_named_stage
    with planner=None (Pilz-PTP→OMPL) and never calls the IK/cartesian path."""
    fake = _FakeVision()
    stages = _make_stages(fake)
    corrected = [10.0, 20.0, 30.0, 40.0, 50.0, -133.0]
    err = stages._execute_motion_target(JointTarget(joints_deg=corrected))
    assert err is None
    assert fake.joint_moved_to == corrected  # executed verbatim
    assert fake.joint_planner is None  # fallback chain, not a forced planner
    assert fake.moved_to is None  # cartesian/IK path NOT taken


# ---- PR3: pick/place fused approach+grasp+retreat tail ----------------------


def test_approach_pose_bare_when_no_terminal():
    """vision_moveto: no terminal_action -> CartesianTarget with empty tail."""
    from beambot.pipeline.goal_computers import compute_approach_pose

    ctx = types.SimpleNamespace(
        goal=_goal(),
        vision=_FakeVision(),
        error=None,
        detect_only_pose=None,
    )
    target = compute_approach_pose("DET", ctx)
    assert isinstance(target, CartesianTarget)
    assert target.grasp_state == "" and target.retreat_pose_key == ""


def test_approach_pose_carries_grasp_tail_for_pick():
    """pick: terminal_action='grasp' resolves to the SRDF grasp state + retreat."""
    import json
    from beambot.pipeline.goal_computers import compute_approach_pose

    ctx = types.SimpleNamespace(
        goal=_goal(
            terminal_action="grasp",
            gripper_group="epick_gripper",
            gripper_states_json=json.dumps(
                {"grasp": "vacuum_on", "release": "vacuum_off"}
            ),
            retreat_pose="sample_scan",
        ),
        vision=_FakeVision(),
        error=None,
        detect_only_pose=None,
    )
    target = compute_approach_pose("DET", ctx)
    assert target.grasp_state == "vacuum_on"  # resolved via gripper_states_json
    assert target.gripper_group == "epick_gripper"
    assert target.retreat_pose_key == "sample_scan"


def test_bare_cartesian_uses_move_to_approach_not_fused():
    """Empty tail -> the executor takes the delegated bare-approach path."""
    fake = _FakeVision()
    stages = _make_stages(fake)
    stages.goal = _goal()
    err = stages._execute_cartesian(
        CartesianTarget(pose=fake.approach_pose, ik_frame="epick_tip")
    )
    assert err is None
    assert fake.moved_to == (fake.approach_pose, "epick_tip")  # bare path
    assert not getattr(fake, "gripper_actions", [])  # no gripper stage built


def _patch_mtc_stages(monkeypatch):
    """Replace the C++ MoveTo/Fallbacks with record-only fakes.

    The fused executor builds real moveit_task_constructor stages, whose C++
    bindings validate the planner at construction — a plain fake can't satisfy
    that. These stand-ins let us assert the executor's WIRING (which stages it
    builds) without invoking MoveIt. The bare-approach and JointTarget paths
    don't need this (they delegate to fake methods).
    """
    import beambot.stages.vision_task_stages as vts

    class _FakeStage:
        def __init__(self, *a, **k):
            self.group = None
            self.ik_frame = None

        def setGoal(self, g):
            pass

    class _FakeFallbacks:
        def __init__(self, name):
            self.added = []

        def add(self, s):
            self.added.append(s)

    monkeypatch.setattr(vts.stages, "MoveTo", _FakeStage)
    monkeypatch.setattr(vts.core, "Fallbacks", _FakeFallbacks)
    monkeypatch.setattr(vts, "apply_constraints", lambda *a, **k: None)


def test_fused_cartesian_builds_grasp_and_retreat(monkeypatch):
    """Pick tail: one task with approach + gripper(grasp) + retreat to scan."""
    import json

    _patch_mtc_stages(monkeypatch)
    fake = _FakeVision()
    stages = _make_stages(fake)
    stages.goal = _goal(poses_json=json.dumps({"sample_scan": [0, 0, 0, 0, 0, 0]}))
    target = CartesianTarget(
        pose=fake.approach_pose,
        ik_frame="epick_tip",
        grasp_state="vacuum_on",
        gripper_group="epick_gripper",
        retreat_pose_key="sample_scan",
    )
    err = stages._execute_cartesian(target)
    assert err is None
    assert fake.gripper_actions == ["vacuum_on"]  # gripper actuated
    assert ("retreat", "sample_scan") in fake.named_stages  # retreat added
    assert fake.moved_to is None  # NOT the bare path


def test_pre_open_scan_opens_gripper_then_moves():
    """pick pre-scan: open gripper (release state) then move to scan pose."""
    import json

    fake = _FakeVision()
    stages = _make_stages(fake)
    goal = _goal(
        scan_pose="sample_scan",
        pre_open=True,
        gripper_group="epick_gripper",
        gripper_states_json=json.dumps({"grasp": "vacuum_on", "release": "vacuum_off"}),
        poses_json=json.dumps({"sample_scan": [0, 0, 0, 0, 0, 0]}),
    )
    err = stages._move_to_scan(goal)
    assert err is None
    assert fake.gripper_actions == ["vacuum_off"]  # opened before scan
    assert ("scan position", "sample_scan") in fake.named_stages


def test_no_scan_move_when_scan_pose_empty():
    """vision_moveto/spincoater: orchestrator positioned already -> skip pre-scan."""
    fake = _FakeVision()
    stages = _make_stages(fake)
    err = stages._move_to_scan(_goal(scan_pose=""))
    assert err is None
    assert not getattr(fake, "named_stages", [])  # nothing built


def test_full_pick_run_sets_vacuum_ok_and_executes_fused(monkeypatch):
    """End-to-end run() for a pick: detect -> fused grasp tail -> vacuum check."""
    import json

    _patch_mtc_stages(monkeypatch)
    fake = _FakeVision()
    fake._settle_time = 0.0
    stages = _make_stages(fake)
    goal = _goal(
        scan_pose="sample_scan",
        pre_open=True,
        terminal_action="grasp",
        gripper_group="epick_gripper",
        gripper_states_json=json.dumps({"grasp": "vacuum_on", "release": "vacuum_off"}),
        retreat_pose="sample_scan",
        poses_json=json.dumps({"sample_scan": [0, 0, 0, 0, 0, 0]}),
    )
    err = stages.run(goal)
    assert err is None
    assert "vacuum_on" in fake.gripper_actions  # grasp happened
    assert stages.vacuum_ok is True  # _check_vacuum ran (no ePick -> True)


# ---- orchestrator preset expansion: the client-facing contract -------------
# The six legacy task_type names are preset aliases that expand, via
# _build_vision_goal, to the same VisionTask goal a fully-explicit
# {"task_type":"vision_task", ...} would produce.


def _orchestrator():
    """A bare MTCOrchestratorServer with just the attrs _build_vision_goal needs
    (no rclpy.init / node). Mirrors the lightweight-instance pattern above."""
    from beambot.action_servers.orchestrator import MTCOrchestratorServer

    o = MTCOrchestratorServer.__new__(MTCOrchestratorServer)
    o._current_gripper = "epick"
    o._grippers = {
        "epick": {
            "gripper_group": "epick_gripper",
            "states": {"grasp": "vacuum_on", "release": "vacuum_off"},
        }
    }
    o._gripper_ik_frame = lambda: "epick_tip"
    o._gripper_z_offset = lambda: -0.007
    o.get_logger = lambda: types.SimpleNamespace(
        info=lambda *a, **k: None,
        warning=lambda *a, **k: None,
        error=lambda *a, **k: None,
    )
    return o


def _merged_goal(o, task_type, step):
    """Replicate _call_vision_task's preset+step merge, then build the goal."""
    preset = o._VISION_PRESETS.get(task_type, {})
    cfg = {**preset, **step}
    return o._build_vision_goal(cfg, step.get("poses_json", ""))


def test_preset_pick_sample_expands_to_grasp_goal():
    o = _orchestrator()
    g = _merged_goal(o, "pick_sample", {"tag_id": 5})
    assert g.detector == "marker"
    assert g.goal_computer == "approach_pose"
    # terminal_action carries the state-KEY ("grasp"); the server's goal_computer
    # resolves it to the SRDF state via gripper_states_json. Build sets it verbatim.
    assert g.terminal_action == "grasp"
    assert g.tag_id == 5
    assert g.pre_open is True
    assert g.gripper_group == "epick_gripper"
    assert g.retreat_pose == ""  # retreat_from_scan but no scan_pose given here


def test_preset_place_spincoater_expands_to_j6_snap():
    o = _orchestrator()
    g = _merged_goal(o, "place_spincoater", {"k_offset": 2.0})
    assert g.detector == "spincoater_pocket"
    assert g.goal_computer == "j6_snap"
    assert g.terminal_action == "vacuum_off"
    assert g.scan_pose == "spincoater_scan"
    assert g.target_pose == "spincoater_place"  # from default_target_pose
    assert g.k_offset == pytest.approx(2.0)
    assert g.forward_distance == pytest.approx(0.003)


def test_explicit_step_overrides_preset():
    """A field in the task JSON beats the preset default (merge order)."""
    o = _orchestrator()
    g = _merged_goal(o, "pick_sample", {"detector": "sample_roi", "tag_id": 9})
    assert g.detector == "sample_roi"  # step won over preset's "marker"


def test_canonical_vision_task_needs_no_preset():
    """task_type='vision_task' builds straight from explicit fields."""
    o = _orchestrator()
    g = _merged_goal(
        o,
        "vision_task",
        {
            "detector": "marker",
            "goal_computer": "approach_pose",
            "tag_id": 3,
        },
    )
    assert g.detector == "marker"
    assert g.goal_computer == "approach_pose"
    assert g.tag_id == 3


def test_all_legacy_names_are_routed():
    """Every retired wrapper's task_type still resolves (preset alias present)."""
    o = _orchestrator()
    for name in (
        "vision_moveto",
        "pick_sample",
        "place_sample",
        "pick_spincoater",
        "place_spincoater",
    ):
        assert name in o._VISION_TASK_TYPES
    assert "vision_task" in o._VISION_TASK_TYPES
