"""Tests for the unified vision pipeline plumbing (issue #88).

Hardware-free: the vision_method/computer plugins delegate to TargetLocalizer (which
needs ROS + a camera), so these cover the parts that DON'T — function lookup,
the MotionTarget union, and the detect/compute/execute dispatch in
VisionTask.run() — with a fake vision handle. The real motion path is
gated by hardware verification, by design.
"""

import types
from unittest.mock import MagicMock

import pytest

# Runs under a sourced ROS 2 env (like the repo's other tests via colcon test):
# approach_strategies imports geometry_msgs.PoseStamped, and the dispatch tests import
# VisionTask -> TargetLocalizer, which needs ROS message types at import time.
from beambot.vision.approach_strategies import (
    CartesianTarget,
    JointTarget,
    get_approach_strategy,
    snap_j6,
)
from beambot.vision.vision_methods import get_vision_method


def test_unknown_vision_method_raises_with_available_keys():
    """A typo must fail loudly listing the registered keys, not silently no-op."""
    with pytest.raises(KeyError) as exc:
        get_vision_method("marrker")
    assert "marrker" in str(exc.value)
    assert "marker" in str(exc.value)  # the real one is listed


def test_unknown_approach_strategy_raises():
    with pytest.raises(KeyError) as exc:
        get_approach_strategy("nope")
    assert "approach_pose" in str(exc.value)


def test_builtin_functions_mapped():
    """Map each supported name to its intended function."""
    from beambot.vision import vision_methods, approach_strategies

    assert vision_methods.VISION_METHODS == {
        "marker": vision_methods.detect_marker,
        "sample_roi": vision_methods.detect_sample_roi,
        "spincoater_pocket": vision_methods.detect_spincoater_pocket,
        "spincoater_sample": vision_methods.detect_spincoater_sample,
    }
    assert approach_strategies.APPROACH_STRATEGIES == {
        "approach_pose": approach_strategies.compute_approach_pose,
        "j6_snap": approach_strategies.compute_j6_snap,
    }


def test_tf_reset_replaces_static_frames_and_releases_old_subscriptions():
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener

    from beambot.vision.target_localizer import TargetLocalizer

    vision = TargetLocalizer.__new__(TargetLocalizer)
    vision.logger = MagicMock()
    vision.rclpy_node = MagicMock()
    vision.rclpy_node.create_subscription.side_effect = lambda *a, **kw: object()
    vision._tf_buffer = Buffer()
    vision._tf_listener = TransformListener(vision._tf_buffer, vision.rclpy_node)

    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "old_tip"
    transform.transform.rotation.w = 1.0

    try:
        for _ in range(3):
            old_buffer = vision._tf_buffer
            old_listener = vision._tf_listener
            old_listener.static_callback(TFMessage(transforms=[transform]))
            assert old_buffer.can_transform("base_link", "old_tip", Time())
            vision.rclpy_node.destroy_subscription.reset_mock()

            vision.reset_tf()

            assert vision._tf_buffer is not old_buffer
            assert not vision._tf_buffer.can_transform("base_link", "old_tip", Time())
            destroy = vision.rclpy_node.destroy_subscription
            assert destroy.call_count == 2
            destroy.assert_any_call(old_listener.tf_sub)
            destroy.assert_any_call(old_listener.tf_static_sub)
            vision._tf_listener.static_callback(TFMessage(transforms=[transform]))
            assert vision._tf_buffer.can_transform("base_link", "old_tip", Time())
    finally:
        vision._tf_listener.unregister()


def test_collision_object_update_is_acknowledged_by_move_group():
    from moveit_msgs.msg import CollisionObject

    from beambot.vision.target_localizer import TargetLocalizer

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = types.SimpleNamespace(success=True)
    client = MagicMock()
    client.wait_for_service.return_value = True
    client.call_async.return_value = future
    vision = TargetLocalizer.__new__(TargetLocalizer)
    vision._apply_scene_client = client

    obj = CollisionObject(id="sample")
    vision._apply_collision_object(obj)

    request = client.call_async.call_args.args[0]
    assert request.scene.world.collision_objects == [obj]


def test_collision_object_update_fails_when_service_is_unavailable():
    from beambot.vision.target_localizer import TargetLocalizer

    vision = TargetLocalizer.__new__(TargetLocalizer)
    vision._apply_scene_client = MagicMock()
    vision._apply_scene_client.wait_for_service.return_value = False

    with pytest.raises(RuntimeError, match="service unavailable"):
        vision._apply_collision_object(object())


def test_collision_object_update_fails_when_service_call_times_out(monkeypatch):
    from beambot.vision import target_localizer
    from beambot.vision.target_localizer import TargetLocalizer

    client = MagicMock()
    client.wait_for_service.return_value = True
    vision = TargetLocalizer.__new__(TargetLocalizer)
    vision._apply_scene_client = client
    monkeypatch.setattr(target_localizer, "wait_for_future", lambda *_args, **_kw: False)

    with pytest.raises(RuntimeError, match="call timed out"):
        vision._apply_collision_object(object())


def test_collision_object_update_fails_when_move_group_rejects_it():
    from beambot.vision.target_localizer import TargetLocalizer

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = types.SimpleNamespace(success=False)
    client = MagicMock()
    client.wait_for_service.return_value = True
    client.call_async.return_value = future
    vision = TargetLocalizer.__new__(TargetLocalizer)
    vision._apply_scene_client = client

    with pytest.raises(RuntimeError, match="rejected collision-object update"):
        vision._apply_collision_object(object())


def test_flange_offset_fails_instead_of_returning_unshifted_pose():
    from beambot.vision.target_localizer import TargetLocalizer

    vision = TargetLocalizer.__new__(TargetLocalizer)
    with pytest.raises(ValueError, match="Unknown flange offset direction"):
        vision._apply_flange_offset(object(), "sideways", 0.1)


def test_flange_offset_fails_when_tf_is_unavailable():
    from tf2_ros import TransformException

    from beambot.vision.target_localizer import TargetLocalizer

    vision = TargetLocalizer.__new__(TargetLocalizer)
    vision._tf_buffer = MagicMock()
    vision._tf_buffer.lookup_transform.side_effect = TransformException("missing")

    with pytest.raises(RuntimeError, match="Failed to look up flange TF"):
        vision._apply_flange_offset(object(), "forward", 0.1)


@pytest.mark.parametrize("joint_goal", [None, {"wrist_3_joint": 0.2}])
@pytest.mark.parametrize("ik_frame", ["", "epick_tip"])
@pytest.mark.parametrize("error", [None, "planning failed"])
@pytest.mark.parametrize("with_constraints", [False, True])
def test_move_to_approach_preserves_planner_frame_and_result(
    monkeypatch, joint_goal, ik_frame, error, with_constraints
):
    """Keep PTP/LIN selection, frame handling and execution results unchanged."""
    from geometry_msgs.msg import PoseStamped
    from beambot.vision import target_localizer

    vision = target_localizer.TargetLocalizer.__new__(target_localizer.TargetLocalizer)
    vision.logger = MagicMock()
    vision.arm_group = "ur_arm"
    vision._detect_current_gripper = MagicMock(
        return_value=types.SimpleNamespace(ik_frame="epick_tip")
    )
    vision.compute_deterministic_ik = MagicMock(return_value=joint_goal)
    vision.create_task_template = MagicMock()
    vision.make_pilz_planner = MagicMock()
    vision._set_ik_frame = MagicMock()
    vision.load_plan_execute = MagicMock(return_value=error)
    move_to = MagicMock()
    monkeypatch.setattr(target_localizer.stages, "MoveTo", move_to)
    approach = PoseStamped()
    constraints = object() if with_constraints else None

    assert vision._move_to_approach(approach, ik_frame, constraints=constraints) == error
    vision.compute_deterministic_ik.assert_called_once_with(approach, "epick_tip")
    assert vision._detect_current_gripper.call_count == (0 if ik_frame else 1)
    vision.create_task_template.assert_called_once_with("Vision Move")
    vision.make_pilz_planner.assert_called_once_with(
        "LIN" if joint_goal is None else "PTP"
    )
    move_to.assert_called_once_with("move to tag", vision.make_pilz_planner.return_value)
    stage = move_to.return_value
    assert stage.group == "ur_arm"
    if with_constraints:
        assert stage.path_constraints is constraints
    else:
        assert "path_constraints" not in vars(stage)
    if joint_goal is None:
        assert stage.ik_frame.header.frame_id == "epick_tip"
        vision._set_ik_frame.assert_not_called()
        stage.setGoal.assert_called_once_with(approach)
    else:
        vision._set_ik_frame.assert_called_once_with(stage)
        stage.setGoal.assert_called_once_with(joint_goal)
    task = vision.create_task_template.return_value
    task.add.assert_called_once_with(stage)
    vision.load_plan_execute.assert_called_once_with(task)


def test_cartesian_target_carries_pose_and_frame():
    """Preserve the supplied pose and tool frame."""
    sentinel = object()
    t = CartesianTarget(pose=sentinel, ik_frame="epick_tip")
    assert t.pose is sentinel
    assert t.ik_frame == "epick_tip"


# ---- VisionTask dispatch, with a fake TargetLocalizer (no ROS) ----------


class _FakeVision:
    """Stands in for TargetLocalizer: records calls, returns canned values."""

    def __init__(self):
        self._settle_time = 0.0  # skip the sleep
        self.logger = types.SimpleNamespace(
            info=lambda *a, **k: None, warning=lambda *a, **k: None
        )
        self.moved_to = None
        self.joint_moved_to = None
        self.joint_planner = "UNSET"
        self.submitted_tasks = []
        self.detect_return = "DETECTION"
        # pose-shaped stub: detect_only logging reads approach.pose.position.{x,y,z}
        _position = types.SimpleNamespace(x=0.1, y=0.2, z=0.3)
        self.approach_pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(position=_position),
            header=types.SimpleNamespace(frame_id="base_link"),
        )
        self.approach_return = (self.approach_pose, "epick_tip")

    # vision_method delegates
    def get_cached_pose(self, tag_id):
        return None

    def detect_and_transform_tag(self, tag_id, timeout):
        return self.detect_return

    def detect_and_transform_sample_roi(self, **kwargs):
        self.sample_roi_calls = getattr(self, "sample_roi_calls", 0) + 1
        return self.detect_return

    # computer delegates
    def compute_approach_pose(self, detection, z_offset, **kw):
        return self.approach_return

    def _apply_flange_offset(self, approach, direction, distance):
        return approach

    # cartesian executor delegate (bare approach path)
    def _move_to_approach(self, pose, ik_frame="", constraints=None):
        self.moved_to = (pose, ik_frame)
        self.approach_constraints = constraints
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
        return types.SimpleNamespace(name=label, state=state) if state else None

    # joint executor delegates (JointTarget path)
    def create_task_template(self, name):
        self.tasks_built = getattr(self, "tasks_built", [])
        self.tasks_built.append(name)
        stages = []
        return types.SimpleNamespace(name=name, stages=stages, add=stages.append)

    def make_move_to_named_stage(
        self, label, pose_key, poses, planner=None, constraints=None
    ):
        # Record the joint vector the executor handed us (the corrected pose).
        self.joint_moved_to = poses[pose_key]
        self.joint_planner = planner  # must be None -> Pilz-PTP→OMPL fallback
        self.named_constraints = constraints
        self.named_stages = getattr(self, "named_stages", [])
        self.named_stages.append((label, pose_key))
        return types.SimpleNamespace(name=label, joints=poses[pose_key])

    def load_plan_execute(self, task):
        self.submitted_tasks.append(types.SimpleNamespace(name=task.name, stages=list(task.stages)))
        return None  # success


def _make_stages(fake):
    """Build a VisionTask without running its real __init__ (needs ROS)."""
    from beambot.motion.vision_task import VisionTask

    s = VisionTask.__new__(VisionTask)
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
        vision_method="marker",
        approach_strategy="approach_pose",
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


def test_run_unknown_vision_method_is_config_error():
    fake = _FakeVision()
    stages = _make_stages(fake)
    err = stages.run(_goal(vision_method="bogus"))
    assert err is not None and "PIPELINE_CONFIG_ERROR" in err
    assert fake.moved_to is None


@pytest.mark.parametrize(
    ("strategy", "edge_inset_mm"),
    [
        ("centre", 6.5),
        ("nearest", 6.5),
        ("nearest_edge", -1.0),
        ("nearest_edge", float("nan")),
        ("nearest_edge", float("inf")),
    ],
)
def test_invalid_sample_roi_config_stops_before_detection_and_motion(
    strategy, edge_inset_mm
):
    fake = _FakeVision()
    stages = _make_stages(fake)
    err = stages.run(
        _goal(
            vision_method="sample_roi",
            strategy=strategy,
            edge_inset_mm=edge_inset_mm,
            scan_pose="sample_scan",
            poses_json='{"sample_scan": [0, 0, 0, 0, 0, 0]}',
        )
    )

    assert err is not None and "PIPELINE_CONFIG_ERROR" in err
    assert not hasattr(fake, "sample_roi_calls")
    assert not getattr(fake, "named_stages", [])


def test_invalid_flange_offset_stops_before_detection_and_motion():
    fake = _FakeVision()
    stages = _make_stages(fake)
    err = stages.run(
        _goal(
            offset_direction="sideways",
            offset_distance=0.1,
            scan_pose="sample_scan",
            poses_json='{"sample_scan": [0, 0, 0, 0, 0, 0]}',
        )
    )

    assert err is not None and "PIPELINE_CONFIG_ERROR" in err
    assert not getattr(fake, "named_stages", [])
    assert fake.moved_to is None


@pytest.mark.parametrize("stage_name", ["PickSampleTask", "PlaceSampleTask"])
def test_legacy_sample_actions_reject_invalid_flange_offset(stage_name):
    from beambot.motion import pick_sample_task, place_sample_task

    stage_type = getattr(
        pick_sample_task if stage_name == "PickSampleTask" else place_sample_task,
        stage_name,
    )
    stage = stage_type.__new__(stage_type)

    error = stage.run(
        types.SimpleNamespace(offset_direction="sideways", offset_distance=0.1)
    )

    assert "Unknown flange offset direction" in error


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


@pytest.mark.parametrize(
    "base, angle, offset, expected",
    [
        (100, 0, 0, 100),
        (0, 30, 0, 30),
        (0, 45, 0, 45),
        (0, 46, 0, -44),
        (0, 89, 0, -1),
        (10, 90, 0, 10),
        (0, -1, 0, -1),
        (0, -45, 0, 45),
        (0, -46, 0, 44),
        (10, -180, 0, 10),
        (-163, 30, 0, -133),
        (0, 40, 10, -40),
    ],
)
def test_snap_j6_uses_the_nearest_symmetric_angle(base, angle, offset, expected):
    assert snap_j6(base, angle, offset) == pytest.approx(expected)


# ---- j6_snap approach strategy + JointTarget executor (spincoater path) ---------


def test_j6_snap_emits_jointtarget_with_corrected_j6():
    """spincoater detection dict -> JointTarget; joint 6 snapped, NO IK."""
    import json
    from beambot.vision.approach_strategies import compute_j6_snap

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
    from beambot.vision.approach_strategies import compute_j6_snap

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
    fake.create_relative_move_stage = MagicMock(return_value=object())
    stages = _make_stages(fake)
    stages.goal = _goal(
        constraints_json='{"joint_constraints": ['
        '{"joint_name": "wrist_1_joint", "position": 90}]}'
    )
    corrected = [10.0, 20.0, 30.0, 40.0, 50.0, -133.0]
    err = stages._execute_motion_target(JointTarget(
        joints_deg=corrected, forward_distance=0.02,
    ))
    assert err is None
    assert fake.joint_moved_to == corrected  # executed verbatim
    assert fake.joint_planner is None  # fallback chain, not a forced planner
    assert fake.moved_to is None  # cartesian/IK path NOT taken
    assert fake.named_constraints.joint_constraints[0].joint_name == "wrist_1_joint"
    fake.create_relative_move_stage.assert_called_once_with(
        "forward contact", "forward", 0.02, constraints=fake.named_constraints,
    )


# ---- PR3: pick/place fused approach+grasp+retreat tail ----------------------


def test_approach_pose_bare_when_no_terminal():
    """vision_moveto: no terminal_action -> CartesianTarget with empty tail."""
    from beambot.vision.approach_strategies import compute_approach_pose

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
    from beambot.vision.approach_strategies import compute_approach_pose

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
    stages.goal = _goal(
        constraints_json='{"joint_constraints": ['
        '{"joint_name": "wrist_1_joint", "position": 90}]}'
    )
    err = stages._execute_cartesian(
        CartesianTarget(pose=fake.approach_pose, ik_frame="epick_tip")
    )
    assert err is None
    assert fake.moved_to == (fake.approach_pose, "epick_tip")  # bare path
    assert fake.approach_constraints.joint_constraints[0].joint_name == "wrist_1_joint"
    assert not getattr(fake, "gripper_actions", [])  # no gripper stage built


def _patch_mtc_stages(monkeypatch):
    """Replace the C++ MoveTo/Fallbacks with record-only fakes.

    The fused executor builds real moveit_task_constructor stages, whose C++
    bindings validate the planner at construction — a plain fake can't satisfy
    that. These stand-ins let us assert the executor's WIRING (which stages it
    builds) without invoking MoveIt. The bare-approach and JointTarget paths
    don't need this (they delegate to fake methods).
    """
    import beambot.motion.vision_task as vts

    class _FakeStage:
        def __init__(self, name, planner):
            self.name = name
            self.group = None
            self.ik_frame = None

        def setGoal(self, g):
            self.goal = g

    class _FakeFallbacks:
        def __init__(self, name):
            self.name = name
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
    task, = fake.submitted_tasks
    assert [stage.name for stage in task.stages] == ["approach", "gripper", "retreat"]
    approach, grasp, retreat = task.stages
    assert [stage.name for stage in approach.added] == ["approach [PTP]", "approach [LIN]"]
    assert approach.added[0].goal == {"shoulder_pan_joint": 0.0}
    assert approach.added[1].goal is fake.approach_pose
    assert grasp.state == "vacuum_on"
    assert retreat.joints == [0, 0, 0, 0, 0, 0]
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
    task, = fake.submitted_tasks
    assert [stage.name for stage in task.stages] == ["open gripper", "scan position"]
    assert task.stages[0].state == "vacuum_off"
    assert task.stages[1].joints == [0, 0, 0, 0, 0, 0]


def test_no_scan_move_when_scan_pose_empty():
    """vision_moveto/spincoater: orchestrator positioned already -> skip pre-scan."""
    fake = _FakeVision()
    stages = _make_stages(fake)
    err = stages._move_to_scan(_goal(scan_pose=""))
    assert err is None
    assert not getattr(fake, "named_stages", [])  # nothing built


@pytest.mark.parametrize(
    ("terminal_action", "expected_state"),
    [("grasp", "vacuum_on"), ("release", "vacuum_off")],
)
def test_full_gripper_run_skips_vacuum_check(
    monkeypatch, terminal_action, expected_state
):
    """Pick/place executes without reading vacuum status."""
    import json

    _patch_mtc_stages(monkeypatch)
    fake = _FakeVision()
    fake._settle_time = 0.0
    stages = _make_stages(fake)
    stages.vacuum_ok = False
    stages._check_vacuum = MagicMock(
        side_effect=AssertionError("Vacuum check is disabled")
    )
    goal = _goal(
        scan_pose="sample_scan",
        pre_open=terminal_action == "grasp",
        terminal_action=terminal_action,
        gripper_group="epick_gripper",
        gripper_states_json=json.dumps({"grasp": "vacuum_on", "release": "vacuum_off"}),
        retreat_pose="sample_scan",
        poses_json=json.dumps({"sample_scan": [0, 0, 0, 0, 0, 0]}),
    )
    err = stages.run(goal)
    assert err is None
    assert expected_state in fake.gripper_actions
    assert stages.vacuum_ok is True
    stages._check_vacuum.assert_not_called()


# ---- orchestrator preset expansion: the client-facing contract -------------
# The legacy task_type names are preset aliases that expand, via
# core.vision_goals, to the same VisionTask goal a fully-explicit
# {"task_type":"vision_task", ...} would produce.


def _orchestrator():
    """A bare BeambotOrchestratorServer with just the attrs vision dispatch needs
    (no rclpy.init / node). Mirrors the lightweight-instance pattern above."""
    from beambot.action_servers.orchestrator import BeambotOrchestratorServer

    o = BeambotOrchestratorServer.__new__(BeambotOrchestratorServer)
    o._current_gripper = "epick"
    o._grippers = {
        "epick": {
            "gripper_group": "epick_gripper",
            "states": {"grasp": "vacuum_on", "release": "vacuum_off"},
            "z_offset": -0.007,
        }
    }
    o._gripper_ik_frame = lambda: "epick_tip"
    o.get_logger = lambda: types.SimpleNamespace(
        info=lambda *a, **k: None,
        warning=lambda *a, **k: None,
        error=lambda *a, **k: None,
    )
    return o


def _dispatched_goal(o, task_type, step):
    """Capture the real dispatch, with only the child action boundary replaced."""
    o._vision_task_client = object()
    o._timeouts = {"vision_task": 0}
    o._send_and_wait = MagicMock(return_value=False)
    o._call_vision_task(task_type, {"settle_time": 0, **step}, step.get("poses_json", ""))
    o._send_and_wait.assert_called_once()
    client, goal, name, timeout = o._send_and_wait.call_args.args
    assert (client, name, timeout) == (o._vision_task_client, task_type, 0)
    return goal


def test_preset_pick_sample_expands_to_grasp_goal():
    o = _orchestrator()
    g = _dispatched_goal(o, "pick_sample", {"tag_id": 5})
    assert g.vision_method == "marker"
    assert g.approach_strategy == "approach_pose"
    # terminal_action carries the state-KEY ("grasp"); the server's approach_strategy
    # resolves it to the SRDF state via gripper_states_json. Build sets it verbatim.
    assert g.terminal_action == "grasp"
    assert g.tag_id == 5
    assert g.pre_open is True
    assert g.gripper_group == "epick_gripper"
    assert g.retreat_pose == ""  # retreat_from_scan but no scan_pose given here
    assert g.ik_frame == "epick_tip"
    assert g.z_offset == pytest.approx(-0.007)  # current gripper default


def test_preset_place_spincoater_expands_to_j6_snap():
    o = _orchestrator()
    g = _dispatched_goal(o, "place_spincoater", {"k_offset": 2.0})
    assert g.vision_method == "spincoater_pocket"
    assert g.approach_strategy == "j6_snap"
    assert g.terminal_action == "vacuum_off"
    assert g.scan_pose == "spincoater_scan"
    assert g.target_pose == "spincoater_place"  # from default_target_pose
    assert g.k_offset == pytest.approx(2.0)
    assert g.forward_distance == pytest.approx(0.003)


def test_explicit_step_overrides_preset():
    """A field in the task JSON beats the preset default (merge order)."""
    o = _orchestrator()
    g = _dispatched_goal(o, "pick_sample", {"vision_method": "sample_roi", "tag_id": 9})
    assert g.vision_method == "sample_roi"  # step won over preset's "marker"


@pytest.mark.parametrize(
    ("step", "vision_method"),
    [
        ({"settle_time": 0}, "marker"),
        ({"vision_method": "sample_roi", "settle_time": 0}, "sample_roi"),
        ({"detection_type": "sample_roi", "settle_time": 0}, "sample_roi"),
        ({"detector": "sample_roi", "settle_time": 0}, "sample_roi"),
        (
            {"detector": "marker", "detection_type": "sample_roi", "settle_time": 0},
            "marker",
        ),
        (
            {"vision_method": "marker", "detector": "sample_roi", "settle_time": 0},
            "marker",
        ),
    ],
)
def test_vision_moveto_selector_precedence(step, vision_method):
    """Legacy task fields override presets; canonical fields override legacy ones."""
    o = _orchestrator()
    assert _dispatched_goal(o, "vision_moveto", step).vision_method == vision_method


def test_legacy_goal_computer_key_maps_to_approach_strategy():
    """Deprecated goal_computer still selects the approach strategy."""
    o = _orchestrator()
    g = _dispatched_goal(o, "vision_task", {"goal_computer": "j6_snap", "tag_id": 3})
    assert g.approach_strategy == "j6_snap"


def test_canonical_vision_task_needs_no_preset():
    """task_type='vision_task' builds straight from explicit fields."""
    o = _orchestrator()
    g = _dispatched_goal(
        o,
        "vision_task",
        {
            "vision_method": "marker",
            "approach_strategy": "approach_pose",
            "tag_id": 3,
        },
    )
    assert g.vision_method == "marker"
    assert g.approach_strategy == "approach_pose"
    assert g.tag_id == 3


def test_all_legacy_names_are_routed():
    """Every retired wrapper's task_type still resolves (preset alias present)."""
    from beambot.utils.task_parser import SUPPORTED_TASK_TYPES
    from beambot.utils.vision_goals import VISION_TASK_TYPES

    for name in (
        "vision_moveto",
        "pick_sample",
        "place_sample",
        "pick_spincoater",
        "place_spincoater",
    ):
        assert name in VISION_TASK_TYPES
    assert "vision_task" in VISION_TASK_TYPES
    # The parser accepts every name the orchestrator routes.
    assert VISION_TASK_TYPES <= SUPPORTED_TASK_TYPES


def test_scan_positions_flatten_to_radians_and_skip_unknown():
    import json
    import math

    from beambot.utils.vision_goals import build_vision_goal

    warnings = []
    poses = {"a": [0, 90, 0, 0, 0, 180], "b": [45, 0, 0, 0, 0, 0]}
    g = build_vision_goal(
        {"scan_positions": ["a", "missing", "b"]},
        json.dumps(poses),
        grippers={},
        current_gripper="none",
        ik_frame="flange",
        on_warning=warnings.append,
    )
    assert g.num_scan_positions == 2
    assert list(g.scan_positions_flat) == pytest.approx(
        [math.radians(j) for j in poses["a"] + poses["b"]]
    )
    assert len(warnings) == 1 and "missing" in warnings[0]
