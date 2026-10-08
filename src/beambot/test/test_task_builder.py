"""Tests for pure functions in beambot.motion.task_builder."""

import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, call

import pytest

import beambot.config_loader as config_loader
import beambot.motion.task_builder as task_builder
from beambot.motion.task_builder import (
    MtcTaskBuilder,
    joints_from_degrees,
    parse_constraints,
    DIRECTION_VECTORS,
    DEFAULT_JOINT_NAMES,
)


def test_mtc_node_is_initialized_once_on_first_use(monkeypatch):
    node = object()
    initialize = Mock()
    create_node = Mock(return_value=node)
    monkeypatch.setattr(task_builder, "_mtc_node", None)
    monkeypatch.setattr(task_builder.rclcpp, "init", initialize)
    monkeypatch.setattr(task_builder.rclcpp, "Node", create_node)

    assert task_builder._get_mtc_node() is node
    assert task_builder._get_mtc_node() is node
    initialize.assert_called_once_with()
    create_node.assert_called_once_with("beambot_mtc", task_builder._options)


def test_wait_for_future(monkeypatch):
    cases = [
        ([True], [0.0], 1.0, {}, True, []),
        ([False, True], [0.0, 0.0], 1.0, {}, True, [call(0.01)]),
        ([False, False], [0.0, 0.0, 1.0], 1.0, {}, False, [call(0.01)]),
        ([False], [0.0, 0.0], 0.0, {}, False, []),
        ([False, True], [0.0, 0.0], 1.0, {"poll_interval": 0.05}, True, [call(0.05)]),
    ]
    for done, times, timeout, kwargs, expected, sleeps in cases:
        future = Mock()
        future.done.side_effect = done
        clock = SimpleNamespace(monotonic=Mock(side_effect=times), sleep=Mock())
        monkeypatch.setattr(task_builder, "time", clock)
        assert task_builder.wait_for_future(future, timeout, **kwargs) is expected
        assert clock.sleep.call_args_list == sleeps
        future.result.assert_not_called()


@pytest.mark.parametrize("use_fallback", [False, True])
@pytest.mark.parametrize("move_kind", ["relative", "named_stomp", "named_ompl"])
def test_move_stage_preserves_setup(use_fallback, move_kind, monkeypatch):
    class Fallbacks(list):
        def __init__(self, name):
            super().__init__()
            self.name = name

        add = list.append

    monkeypatch.setattr(task_builder.core, "Fallbacks", Fallbacks)
    def create_stage(name, planner):
        return SimpleNamespace(name=name, planner=planner, setDirection=Mock(), setGoal=Mock())

    monkeypatch.setattr(task_builder.stages, "MoveRelative", create_stage)
    monkeypatch.setattr(task_builder.stages, "MoveTo", create_stage)
    instance = MtcTaskBuilder.__new__(MtcTaskBuilder)
    instance.arm_group = "ur_arm"
    instance.ik_frame = "tool_tip"
    instance._pipeline = "stomp" if move_kind == "named_stomp" else "ompl"
    instance._ompl_timeout = 5.0
    lin, cartesian, ptp, pipeline, explicit = (object() for _ in range(5))
    instance.make_pilz_planner = Mock(side_effect={"LIN": lin, "PTP": ptp}.__getitem__)
    instance.make_cartesian_planner = Mock(return_value=cartesian)
    instance.make_pipeline_planner = Mock(return_value=pipeline)
    constraints = object()

    if move_kind == "relative":
        result = instance.create_relative_move_stage(
            "approach", "forward", 0.05,
            planner=None if use_fallback else explicit, constraints=constraints,
        )
        expected = [
            ("approach [Pilz LIN]", lin),
            ("approach [CartesianPath]", cartesian),
            ("approach [Pilz PTP]", ptp),
        ]
    else:
        pose = [10, -20, 30, -40, 50, -60]
        result = instance.make_move_to_named_stage(
            "approach", "home", {"home": pose},
            planner=None if use_fallback else explicit, constraints=constraints,
        )
        expected = [
            ("approach [Pilz PTP]", ptp),
            (f"approach [{instance._pipeline.upper()}]", pipeline),
        ]
        assert instance._pin_goal_joints == joints_from_degrees(pose)

    assert isinstance(result, Fallbacks) == use_fallback
    assert result.name == "approach"
    children = list(result) if use_fallback else [result]
    if not use_fallback:
        expected = [("approach", explicit)]
        instance.make_pilz_planner.assert_not_called()
        instance.make_cartesian_planner.assert_not_called()
        instance.make_pipeline_planner.assert_not_called()
    assert [(stage.name, stage.planner) for stage in children] == expected
    for stage in children:
        assert stage.group == "ur_arm"
        assert stage.ik_frame.header.frame_id == "tool_tip"
        assert stage.path_constraints is constraints
        if move_kind == "relative":
            stage.setDirection.assert_called_once()
            direction = stage.setDirection.call_args.args[0]
            assert direction.header.frame_id == "tool_tip"
            assert (direction.vector.x, direction.vector.y, direction.vector.z) == (0.05, 0.0, 0.0)
        else:
            stage.setGoal.assert_called_once_with(joints_from_degrees(pose))
        expected_timeout = 5.0 if stage.planner is pipeline else None
        assert getattr(stage, "timeout", None) == expected_timeout


def test_preview_publisher_is_reused_per_node():
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    from trajectory_msgs.msg import JointTrajectoryPoint

    trajectory = task_builder.RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ["wrist_1_joint"]
    point = JointTrajectoryPoint(positions=[0.2])
    point.time_from_start.sec = 2
    trajectory.joint_trajectory.points = [point]
    solution = SimpleNamespace(sub_trajectory=[SimpleNamespace(trajectory=trajectory)])
    nodes = [SimpleNamespace(create_publisher=Mock(return_value=Mock())) for _ in range(2)]

    for node in nodes:
        for _ in range(5):
            instance = MtcTaskBuilder.__new__(MtcTaskBuilder)
            instance.rclpy_node = node
            instance.logger = Mock()
            instance._publish_preview_trajectory(solution, "preview")
            instance.logger.warning.assert_not_called()

        node.create_publisher.assert_called_once()
        message_type, topic, qos = node.create_publisher.call_args.args
        assert message_type is task_builder.DisplayTrajectory
        assert topic == "/beambot/preview_trajectory"
        assert qos.depth == 1
        assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert qos.reliability == ReliabilityPolicy.RELIABLE
        publisher = node.create_publisher.return_value
        assert publisher.publish.call_count == 5
        for call in publisher.publish.call_args_list:
            msg = call.args[0]
            assert msg.trajectory == [trajectory]
            assert msg.trajectory_start.joint_state.name == ["wrist_1_joint"]
            assert list(msg.trajectory_start.joint_state.position) == [0.2]

    assert nodes[0]._preview_trajectory_pub is not nodes[1]._preview_trajectory_pub


@pytest.mark.parametrize(
    "outcome", ["success", "unavailable", "timeout", "request_error", "create_error"]
)
def test_ik_client_is_destroyed_on_exit(outcome, monkeypatch):
    joint_state = task_builder.JointState(
        name=list(DEFAULT_JOINT_NAMES), position=[math.radians(10.123)] * 6
    )
    response = SimpleNamespace(
        error_code=SimpleNamespace(val=1),
        solution=SimpleNamespace(joint_state=joint_state),
    )
    client = Mock()
    client.wait_for_service.return_value = outcome != "unavailable"
    # Client.call returns None when the reply does not arrive in time.
    client.call.return_value = None if outcome == "timeout" else response
    if outcome == "request_error":
        client.call.side_effect = RuntimeError("request failed")

    subscription = object()

    def subscribe(msg_type, topic, callback, depth):
        callback(joint_state)
        return subscription

    node = SimpleNamespace(
        create_subscription=Mock(side_effect=subscribe), destroy_subscription=Mock(),
        create_client=Mock(return_value=client), destroy_client=Mock(),
    )
    if outcome == "create_error":
        node.create_client.side_effect = RuntimeError("creation failed")
    instance = MtcTaskBuilder.__new__(MtcTaskBuilder)
    instance.rclpy_node = node
    instance.logger = Mock()
    instance.arm_group = "ur_arm"
    approach = task_builder.PoseStamped()

    result = instance.compute_deterministic_ik(approach, "tool_tip")

    expected = dict(zip(joint_state.name, joint_state.position)) if outcome == "success" else None
    assert result == expected
    node.destroy_subscription.assert_called_once_with(subscription)
    node.create_client.assert_called_once_with(task_builder.GetPositionIK, "/compute_ik")
    if outcome == "create_error":
        node.destroy_client.assert_not_called()
    else:
        node.destroy_client.assert_called_once_with(client)
        client.wait_for_service.assert_called_once_with(timeout_sec=2.0)
    if outcome not in {"create_error", "unavailable"}:
        client.call.assert_called_once()
        assert client.call.call_args.kwargs == {"timeout_sec": 5.0}
        request = client.call.call_args.args[0].ik_request
        assert request.group_name == "ur_arm"
        assert request.ik_link_name == "tool_tip"
        assert request.pose_stamped == approach
        assert request.robot_state.joint_state.name == list(DEFAULT_JOINT_NAMES)
        assert list(request.robot_state.joint_state.position) == [math.radians(10.12)] * 6
        assert request.timeout.sec == 1


@pytest.mark.parametrize("beamline, pipeline", [("cms", "stomp"), ("lix", "ompl")])
def test_pipeline_planner_uses_forwarded_beamline_config(beamline, pipeline, monkeypatch):
    descriptions = Path(__file__).parents[2] / "custom-ur-descriptions"
    monkeypatch.setattr(
        config_loader, "load_beamline_config",
        lambda: ({"robot": {"moveit_config_package": f"{beamline}_moveit_config"}}, ""),
    )
    monkeypatch.setattr(
        "ament_index_python.packages.get_package_share_path",
        lambda package: descriptions / package,
    )
    monkeypatch.setattr(
        task_builder, "_options",
        SimpleNamespace(arguments=config_loader.build_pipeline_param_args()),
    )
    calls = []
    planner = SimpleNamespace()

    def create_planner(node, name, **kwargs):
        calls.append((node, name, kwargs))
        return planner

    monkeypatch.setattr(task_builder.core, "PipelinePlanner", create_planner)
    mtc_node = object()
    monkeypatch.setattr(task_builder, "_get_mtc_node", lambda: mtc_node)
    instance = MtcTaskBuilder(SimpleNamespace(get_logger=lambda: None))

    assert instance.make_pipeline_planner() is planner
    assert calls == [
        (mtc_node, pipeline, {"planner_id": "RRTstar"} if pipeline == "ompl" else {})
    ]


def test_new_task_preserves_its_endpoint_after_a_named_move(monkeypatch):
    monkeypatch.setattr(task_builder.core, "Task", Mock())
    monkeypatch.setattr(task_builder.stages, "CurrentState", Mock())
    instance = MtcTaskBuilder.__new__(MtcTaskBuilder)
    instance.rclpy_node = SimpleNamespace()
    instance._mtc_node = object()
    instance._task_planner_cache = {}
    instance.logger = Mock()
    instance._pin_goal_joints = {"wrist_1_joint": 0.2}

    instance.create_task_template("next forward move")
    point = SimpleNamespace(positions=[0.5], velocities=[], accelerations=[])
    trajectory = SimpleNamespace(joint_trajectory=SimpleNamespace(
        joint_names=["wrist_1_joint"], points=[point],
    ))
    instance._pin_endpoints(SimpleNamespace(
        sub_trajectory=[SimpleNamespace(trajectory=trajectory)],
    ))

    assert point.positions == [0.5]
    instance.logger.warning.assert_not_called()


def test_robot_model_cache_is_revision_aware(monkeypatch):
    created = []
    loads = []

    class _Node:
        _robot_model_revision = "one"

    class _Task:
        def __init__(self, *_a, **_k):
            self.model = None
            created.append(self)

        def loadRobotModel(self, _node, _description="robot_description"):
            self.model = object()
            loads.append(self.model)

        def getRobotModel(self):
            return self.model

        def setRobotModel(self, model):
            self.model = model

        def add(self, _stage):
            pass

    monkeypatch.setattr(task_builder, "_model_cache", {})
    monkeypatch.setattr(task_builder.core, "Task", _Task)
    monkeypatch.setattr(task_builder.stages, "CurrentState", lambda _name: object())

    instance = MtcTaskBuilder.__new__(MtcTaskBuilder)
    instance.rclpy_node = _Node()
    instance._task_planner_cache = {}
    instance._mtc_node = object()

    instance.create_task_template("one")
    instance.create_task_template("two")
    instance.rclpy_node._robot_model_revision = "two"
    instance.create_task_template("three")
    instance.rclpy_node._robot_model_revision = ""
    instance.create_task_template("unversioned-one")
    instance.create_task_template("unversioned-two")
    instance.rclpy_node._moveit_manager = SimpleNamespace(model_revision="three")
    instance.create_task_template("orchestrator-one")
    instance.create_task_template("orchestrator-two")

    assert len(loads) == 5
    assert created[0].model is created[1].model
    assert created[2].model is not created[1].model
    assert created[3].model is not created[2].model
    assert created[4].model is not created[3].model
    assert created[5].model is created[6].model


def test_persistent_server_loads_managed_revisions_from_verified_root(monkeypatch):
    revisions = [
        "beambot_robot_models__hande",
        "beambot_robot_models__none",
        "beambot_robot_models__epick__3mm_dia_external_air",
    ]
    loads = []

    class _Task:
        def __init__(self, *_a, **_k):
            pass

        def loadRobotModel(self, _node, description="robot_description"):
            loads.append(description)
            self.model = object()

        def getRobotModel(self):
            return self.model

        def setRobotModel(self, model):
            self.model = model

        def add(self, _stage):
            pass

    monkeypatch.setattr(task_builder, "_model_cache", {})
    monkeypatch.setattr(task_builder.core, "Task", _Task)
    monkeypatch.setattr(task_builder.stages, "CurrentState", lambda _name: object())

    instance = MtcTaskBuilder.__new__(MtcTaskBuilder)
    instance.rclpy_node = SimpleNamespace(_robot_model_revision="")
    instance._task_planner_cache = {}
    instance._mtc_node = object()

    for revision in revisions:
        instance.rclpy_node._robot_model_revision = revision
        instance.create_task_template(revision)

    # Each new revision reloads from the verified root; only the latest is cached.
    assert loads == [task_builder.VERIFIED_MODEL_DESCRIPTION] * len(revisions)
    assert task_builder._model_cache.keys() == {revisions[-1]}


def test_verified_model_root_has_kinematics_and_acceleration_limits():
    root = task_builder.VERIFIED_MODEL_DESCRIPTION
    args = task_builder._options.arguments

    assert any(
        arg.startswith(f"{root}_kinematics.ur_arm.kinematics_solver:=")
        for arg in args
    )
    assert any(
        arg.startswith(
            f"{root}_planning.joint_limits.shoulder_pan_joint.max_acceleration:="
        )
        for arg in args
    )


# ---------------------------------------------------------------------------
# joints_from_degrees
# ---------------------------------------------------------------------------

class TestJointsFromDegrees:

    def test_zeros(self):
        result = joints_from_degrees([0, 0, 0, 0, 0, 0])
        assert all(abs(v) < 1e-10 for v in result.values())

    def test_known_values(self):
        result = joints_from_degrees([0, -90, 90, -90, -90, 0])
        assert abs(result["shoulder_pan_joint"] - 0.0) < 1e-10
        assert abs(result["shoulder_lift_joint"] - math.radians(-90)) < 1e-10
        assert abs(result["elbow_joint"] - math.radians(90)) < 1e-10

    def test_returns_all_joints(self):
        result = joints_from_degrees([10, 20, 30, 40, 50, 60])
        assert set(result.keys()) == set(DEFAULT_JOINT_NAMES)

    def test_wrong_length_raises(self):
        with pytest.raises(ValueError, match="Expected 6"):
            joints_from_degrees([0, 0, 0])

    def test_empty_raises(self):
        with pytest.raises(ValueError):
            joints_from_degrees([])

    def test_too_many_raises(self):
        with pytest.raises(ValueError):
            joints_from_degrees([0] * 7)

    def test_360_degrees(self):
        result = joints_from_degrees([360, 0, 0, 0, 0, 0])
        assert abs(result["shoulder_pan_joint"] - math.radians(360)) < 1e-10

    def test_negative_values(self):
        result = joints_from_degrees([-180, -90, -45, -30, -15, -5])
        assert abs(result["shoulder_pan_joint"] - math.radians(-180)) < 1e-10
        assert abs(result["wrist_3_joint"] - math.radians(-5)) < 1e-10


# ---------------------------------------------------------------------------
# parse_constraints
# ---------------------------------------------------------------------------

class TestParseConstraints:

    def test_none_returns_none(self):
        assert parse_constraints(None) is None

    def test_empty_dict_returns_none(self):
        assert parse_constraints({}) is None

    def test_empty_lists_returns_none(self):
        assert parse_constraints({"joint_constraints": [], "orientation_constraints": []}) is None

    def test_joint_constraint(self):
        result = parse_constraints({
            "joint_constraints": [{
                "joint_name": "wrist_3_joint",
                "position": 90.0,
                "tolerance_above": 5.0,
                "tolerance_below": 5.0,
                "weight": 1.0,
            }]
        })
        assert result is not None
        assert len(result.joint_constraints) == 1
        jc = result.joint_constraints[0]
        assert jc.joint_name == "wrist_3_joint"
        assert abs(jc.position - math.radians(90.0)) < 1e-10
        assert abs(jc.tolerance_above - math.radians(5.0)) < 1e-10
        assert abs(jc.tolerance_below - math.radians(5.0)) < 1e-10
        assert jc.weight == 1.0

    def test_joint_constraint_defaults(self):
        """tolerance and weight should use defaults if omitted."""
        result = parse_constraints({
            "joint_constraints": [{
                "joint_name": "elbow_joint",
                "position": 0.0,
            }]
        })
        jc = result.joint_constraints[0]
        assert abs(jc.tolerance_above - math.radians(1.0)) < 1e-10
        assert abs(jc.tolerance_below - math.radians(1.0)) < 1e-10
        assert jc.weight == 1.0

    def test_orientation_constraint(self):
        result = parse_constraints({
            "orientation_constraints": [{
                "link_name": "flange",
                "frame_id": "base_link",
                "orientation": [180, 0, 0],
                "tolerance": [5, 5, 360],
                "weight": 1.0,
            }]
        })
        assert result is not None
        assert len(result.orientation_constraints) == 1
        oc = result.orientation_constraints[0]
        assert oc.link_name == "flange"
        assert oc.header.frame_id == "base_link"
        assert abs(oc.absolute_x_axis_tolerance - math.radians(5)) < 1e-10
        assert abs(oc.absolute_z_axis_tolerance - math.radians(360)) < 1e-10
        # Both quaternion signs describe the same 180-degree rotation about X.
        q = oc.orientation
        assert abs(q.x) == pytest.approx(1.0)
        assert (q.y, q.z, q.w) == pytest.approx((0.0, 0.0, 0.0), abs=1e-10)

    def test_multiple_joint_constraints(self):
        result = parse_constraints({
            "joint_constraints": [
                {"joint_name": "wrist_1_joint", "position": 0.0},
                {"joint_name": "wrist_3_joint", "position": 45.0},
            ]
        })
        assert len(result.joint_constraints) == 2
        assert result.joint_constraints[0].joint_name == "wrist_1_joint"
        assert result.joint_constraints[1].joint_name == "wrist_3_joint"

    def test_mixed_constraints(self):
        result = parse_constraints({
            "joint_constraints": [
                {"joint_name": "wrist_3_joint", "position": 0.0},
            ],
            "orientation_constraints": [
                {"link_name": "flange", "orientation": [0, 0, 0], "tolerance": [10, 10, 10]},
            ],
        })
        assert len(result.joint_constraints) == 1
        assert len(result.orientation_constraints) == 1


# ---------------------------------------------------------------------------
# DIRECTION_VECTORS
# ---------------------------------------------------------------------------

class TestDirectionVectors:

    def test_all_unit_vectors(self):
        """All direction vectors should have magnitude 1."""
        for name, (x, y, z) in DIRECTION_VECTORS.items():
            mag = math.sqrt(x**2 + y**2 + z**2)
            assert abs(mag - 1.0) < 1e-10, f"{name} has magnitude {mag}"

    def test_opposite_pairs(self):
        """forward/backward, left/right, up/down should be negatives."""
        pairs = [
            ("forward", "backward"),
            ("left", "right"),
            ("up", "down"),
        ]
        for a, b in pairs:
            va = DIRECTION_VECTORS[a]
            vb = DIRECTION_VECTORS[b]
            for i in range(3):
                assert abs(va[i] + vb[i]) < 1e-10, f"{a}[{i}] + {b}[{i}] != 0"

    def test_aliases(self):
        """Axis aliases should match named directions."""
        assert DIRECTION_VECTORS["x"] == DIRECTION_VECTORS["forward"]
        assert DIRECTION_VECTORS["-x"] == DIRECTION_VECTORS["backward"]

    def test_has_all_six_directions(self):
        for name in ["forward", "backward", "left", "right", "up", "down"]:
            assert name in DIRECTION_VECTORS
