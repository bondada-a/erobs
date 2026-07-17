"""Tests for pure functions in beambot.stages.base_stages."""

import math
from types import SimpleNamespace

import pytest

import beambot.config_loader as config_loader
import beambot.stages.base_stages as base_stages
from beambot.stages.base_stages import (
    BaseStages,
    joints_from_degrees,
    parse_constraints,
    DIRECTION_VECTORS,
    DEFAULT_JOINT_NAMES,
)


def test_robot_model_cache_is_revision_aware(monkeypatch):
    created = []
    loads = []

    class _Node:
        _robot_model_revision = "one"

    class _Task:
        def __init__(self):
            self.model = None
            created.append(self)

        def enableIntrospection(self, _enabled):
            pass

        def loadRobotModel(self, _node, _description="robot_description"):
            self.model = object()
            loads.append(self.model)

        def getRobotModel(self):
            return self.model

        def setRobotModel(self, model):
            self.model = model

        def add(self, _stage):
            pass

    monkeypatch.setattr(base_stages, "_model_cache", {})
    monkeypatch.setattr(base_stages.core, "Task", _Task)
    monkeypatch.setattr(base_stages.stages, "CurrentState", lambda _name: object())

    instance = BaseStages.__new__(BaseStages)
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


def test_persistent_server_loads_each_managed_revision_from_its_bound_topic(monkeypatch):
    topics = {
        "beambot_robot_models__hande__one__robot_description": (
            "hande_gripper", {"robotiq_hande_end"}
        ),
        "beambot_robot_models__none__two__robot_description": ("", set()),
        "beambot_robot_models__epick__three__robot_description": (
            "epick_gripper", {"epick_tip", "epick_suction_cup"}
        ),
    }
    loads = []

    class _Group:
        def __init__(self, links):
            self.link_model_names = links

    class _Model:
        def __init__(self, group, links):
            self.group = group
            self.links = links

        def has_joint_model_group(self, group):
            return group == self.group

        def get_joint_model_group(self, _group):
            return _Group(self.links)

    class _Task:
        def enableIntrospection(self, _enabled):
            pass

        def loadRobotModel(self, _node, description="robot_description"):
            loads.append(description)
            self.model = _Model(*topics[description])

        def getRobotModel(self):
            return self.model

        def setRobotModel(self, model):
            self.model = model

        def add(self, _stage):
            pass

    grippers = {
        "hande": {"gripper_group": "hande_gripper", "tip_frame": "robotiq_hande_end"},
        "none": {"gripper_group": "", "tip_frame": "flange"},
        "epick": {"gripper_group": "epick_gripper", "tip_frame": "epick_tip"},
    }
    monkeypatch.setattr(config_loader, "load_beamline_config", lambda: ({"grippers": grippers}, ""))
    monkeypatch.setattr(base_stages, "_model_cache", {})
    monkeypatch.setattr(base_stages.core, "Task", _Task)
    monkeypatch.setattr(base_stages.stages, "CurrentState", lambda _name: object())

    instance = BaseStages.__new__(BaseStages)
    instance.rclpy_node = SimpleNamespace(_robot_model_revision="")
    instance._task_planner_cache = {}
    instance._mtc_node = object()

    for revision in topics:
        instance.rclpy_node._robot_model_revision = revision
        instance.create_task_template(revision)

    assert loads == list(topics)
    assert base_stages._model_cache.keys() == {loads[-1]}
    assert base_stages._model_cache[loads[-1]].has_joint_model_group("epick_gripper")


def test_mismatched_managed_model_is_not_cached(monkeypatch):
    revision = "beambot_robot_models__epick__wrong__robot_description"

    class _WrongModel:
        def has_joint_model_group(self, group):
            return group == "hande_gripper"

    class _Task:
        def enableIntrospection(self, _enabled):
            pass

        def loadRobotModel(self, _node, _description="robot_description"):
            self.model = _WrongModel()

        def getRobotModel(self):
            return self.model

        def add(self, _stage):
            pass

    monkeypatch.setattr(
        config_loader,
        "load_beamline_config",
        lambda: ({"grippers": {"epick": {
            "gripper_group": "epick_gripper", "tip_frame": "epick_tip"
        }}}, ""),
    )
    monkeypatch.setattr(base_stages, "_model_cache", {})
    monkeypatch.setattr(base_stages.core, "Task", _Task)

    instance = BaseStages.__new__(BaseStages)
    instance.rclpy_node = SimpleNamespace(_robot_model_revision=revision)
    instance._task_planner_cache = {}
    instance._mtc_node = object()

    with pytest.raises(RuntimeError, match="missing group 'epick_gripper'"):
        instance.create_task_template("wrong")
    assert base_stages._model_cache == {}


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
        # Quaternion from [180, 0, 0] RPY should be ~[1, 0, 0, 0] or [-1, 0, 0, 0]
        q_mag = math.sqrt(oc.orientation.x**2 + oc.orientation.y**2 +
                          oc.orientation.z**2 + oc.orientation.w**2)
        assert abs(q_mag - 1.0) < 1e-5

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
