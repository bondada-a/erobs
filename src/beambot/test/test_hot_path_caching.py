"""Configuration caching and model lifecycle regressions without hardware.

Repeated YAML reads in the ~500 Hz joint-state callback once delayed planning.
Check loader calls as well as values: correct values alone miss that regression.
Managers bypass __init__ so they do not create ROS subscriptions or processes.
"""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

import beambot.config_loader as config_loader
import beambot.utils.moveit_lifecycle_manager as lifecycle_manager
from beambot.utils.moveit_lifecycle_manager import MoveItLifecycleManager


HOT_PATH_ACCESSES = 3  # Repetition exposes uncached work without a timing benchmark.


@pytest.fixture(autouse=True)
def _reset_config_cache():
    """Clear the memoized beamline config around every test.

    load_beamline_config() now caches its parse for the process lifetime. In a
    test process that's shared state: one test priming the cache (or pointing
    the env var elsewhere) would bleed into the next. Resetting before and after
    each test keeps them independent — and is what makes the env-clear fallback
    test below observe a real (uncached) lookup rather than a stale value.
    """
    config_loader.reset_beamline_config_cache()
    yield
    config_loader.reset_beamline_config_cache()


def test_joint_state_callback_filters_by_joint_name():
    manager = MoveItLifecycleManager.__new__(MoveItLifecycleManager)
    manager._arm_joint_names = ("a", "b", "c")
    manager._joint_positions = {}

    for position in range(HOT_PATH_ACCESSES):
        manager._joint_state_cb(SimpleNamespace(
            name=["gripper", "c", "a", "b"], position=[99, 3, position, 2]
        ))

    assert manager._joint_positions == {"a": HOT_PATH_ACCESSES - 1, "b": 2, "c": 3}


def _revision_manager(grippers):
    """Manager whose launches always succeed against a live fake process."""
    manager = MoveItLifecycleManager.__new__(MoveItLifecycleManager)
    manager._grippers = grippers
    manager.cup_override = ""
    manager._moveit_process = None
    manager._model_revision = ""
    manager._use_mock_hardware = True
    manager._logger = Mock()
    manager._publish_verified_model = Mock(return_value=True)

    process = SimpleNamespace(poll=lambda: None)

    def launch(_gripper, _cup_profile):
        manager._moveit_process = process
        return True

    def kill():
        manager._moveit_process = None
        manager._model_revision = ""

    manager._attempt_launch = Mock(side_effect=launch)
    manager.kill_current_process = Mock(side_effect=kill)
    return manager


def test_model_revision_changes_only_when_model_configuration_changes():
    manager = _revision_manager({"epick": {"cup_profile": "3mm_dia_external_air"}})

    assert manager.launch_moveit_with_gripper("epick")
    assert manager.model_revision == "beambot_robot_models__epick__3mm_dia_external_air"

    assert manager.launch_moveit_with_gripper("epick")
    assert manager._attempt_launch.call_count == 1
    assert manager._publish_verified_model.call_count == 1

    manager.cup_override = "3mm_dia"
    assert manager.launch_moveit_with_gripper("epick")
    assert manager.model_revision == "beambot_robot_models__epick__3mm_dia"
    assert manager.kill_current_process.call_count == 1
    assert manager._attempt_launch.call_args.args == ("epick", "3mm_dia")
    assert manager._publish_verified_model.call_count == 2


def test_cup_profile_only_applies_to_grippers_that_define_one():
    manager = _revision_manager({"hande": {}, "epick": {"cup_profile": "3mm_dia"}})
    manager.cup_override = "7mm_dia"

    assert manager.launch_moveit_with_gripper("hande")
    assert manager.model_revision == "beambot_robot_models__hande"
    assert manager._attempt_launch.call_args.args == ("hande", "")

    # Relaunching the same gripper later reuses the same revision.
    assert manager.launch_moveit_with_gripper("epick")
    assert manager.model_revision == "beambot_robot_models__epick__7mm_dia"
    assert manager.launch_moveit_with_gripper("hande")
    assert manager.model_revision == "beambot_robot_models__hande"


def test_failed_model_launch_has_no_revision():
    manager = MoveItLifecycleManager.__new__(MoveItLifecycleManager)
    manager._grippers = {"epick": {}}
    manager.cup_override = ""
    manager._moveit_process = None
    manager._model_revision = ""
    manager._use_mock_hardware = True
    manager._logger = Mock()
    manager._attempt_launch = Mock(return_value=False)

    assert not manager.launch_moveit_with_gripper("epick")
    assert manager.model_revision == ""


def test_verified_descriptions_reuse_managed_publishers():
    urdf = "<robot><link name=\"epick_tip\"/></robot>"
    srdf = "<robot><group name=\"epick_gripper\"/></robot>"

    class _Publisher:
        def __init__(self, topic):
            self.topic = topic
            self.messages = []

        def publish(self, message):
            self.messages.append(message.data)

    class _Node:
        def __init__(self):
            self.publishers = []

        def create_publisher(self, _message_type, topic, _qos):
            publisher = _Publisher(topic)
            self.publishers.append(publisher)
            return publisher

    manager = MoveItLifecycleManager.__new__(MoveItLifecycleManager)
    manager._node = _Node()
    manager._logger = Mock()
    manager._model_description_publishers = (
        manager._node.create_publisher(
            object, lifecycle_manager.VERIFIED_MODEL_DESCRIPTION, object()
        ),
        manager._node.create_publisher(
            object,
            f"{lifecycle_manager.VERIFIED_MODEL_DESCRIPTION}_semantic",
            object(),
        ),
    )
    manager._wait_for_verified_model = Mock(return_value=(urdf, srdf))

    assert manager._publish_verified_model("epick")
    assert [publisher.topic for publisher in manager._node.publishers] == [
        lifecycle_manager.VERIFIED_MODEL_DESCRIPTION,
        f"{lifecycle_manager.VERIFIED_MODEL_DESCRIPTION}_semantic",
    ]
    assert manager._node.publishers[0].messages == [urdf]
    assert manager._node.publishers[1].messages == [srdf]


def test_model_readiness_rejects_empty_descriptions(monkeypatch):
    class _Future:
        def done(self):
            return True

        def result(self):
            return SimpleNamespace(values=["<robot/>", ""])

    client = SimpleNamespace(
        wait_for_services=lambda timeout_sec: True,
        get_parameters=lambda _names: _Future(),
    )
    manager = MoveItLifecycleManager.__new__(MoveItLifecycleManager)
    manager._move_group_parameters = client
    manager._logger = Mock()
    monkeypatch.setattr(lifecycle_manager, "parameter_value_to_python", lambda value: value)

    assert manager._wait_for_verified_model("epick", timeout_sec=0.01) is None
    assert "empty robot descriptions" in manager._logger.error.call_args.args[0]


class TestConfigLoaderContract:
    """Lock in the property the hot-path fix relies on: arm_joint_names()
    returns the canonical 6-DOF ordering even when the env/YAML is absent
    (so an isolated import or a test path never crashes the callback)."""

    def test_arm_joint_names_falls_back_without_config(self):
        # With the env var cleared, arm_joint_names() must still return the
        # standard UR order rather than raising — the callback path depends on
        # this never throwing.
        with patch.dict("os.environ", {}, clear=True):
            names = config_loader.arm_joint_names()
        assert names == list(config_loader._DEFAULT_ARM_JOINTS)
        assert len(names) == 6


@pytest.mark.parametrize(
    "beamline, pipelines",
    [
        ("cms", {"ompl", "pilz_industrial_motion_planner", "stomp"}),
        pytest.param(
            "lix",
            {"ompl", "pilz_industrial_motion_planner"},
            marks=pytest.mark.xfail(
                strict=True, reason="LIX hande payload_mass/payload_cog not yet measured"
            ),
        ),
    ],
)
def test_pipeline_params_match_beamline_planners(beamline, pipelines, monkeypatch):
    source_root = Path(__file__).parents[2]
    monkeypatch.setenv(
        "BEAMBOT_BEAMLINE_CONFIG",
        str(source_root / "beambot" / "config" / f"{beamline}_beamline.yaml"),
    )
    with patch(
        "ament_index_python.packages.get_package_share_path",
        return_value=(
            source_root / "custom-ur-descriptions" / f"{beamline}_moveit_config"
        ),
    ):
        args = config_loader.build_pipeline_param_args()

    params = dict(arg.split(":=", 1) for arg in args[1::2])
    assert {key.split(".", 1)[0] for key in params} == pipelines
    for pipeline in pipelines:
        assert "ValidateSolution" in params[f"{pipeline}.response_adapters"]
    assert "geometric::RRTConnect" in params[
        "ompl.planner_configs.RRTConnectkConfigDefault.type"
    ]


@pytest.mark.parametrize(
    "missing", ["ompl_planning.yaml", "pilz_industrial_motion_planner_planning.yaml"]
)
def test_missing_required_pipeline_config_fails(missing, tmp_path):
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    for filename in (
        "ompl_planning.yaml", "pilz_industrial_motion_planner_planning.yaml"
    ):
        if filename != missing:
            (config_dir / filename).write_text("planning_plugins: []\n")
    with patch(
        "ament_index_python.packages.get_package_share_path",
        return_value=tmp_path,
    ), pytest.raises(FileNotFoundError, match=missing):
        config_loader.build_pipeline_param_args()


_CONFIG_TEMPLATE = """\
beamline: test
robot:
  ip: 192.0.2.1
  moveit_config_package: test_moveit_config
  description_package: test_robot_description
  arm_joints: {joints}
grippers: {{}}
"""


class TestBeamlineConfigCaching:
    """load_beamline_config() must parse the YAML at most once per process, no
    matter how many callers (or how high-frequency a callback) read it. This is
    the root-level guard: every config helper funnels through this function, so
    caching it once is what keeps the 500 Hz path off the disk."""

    @staticmethod
    def _write_config(tmp_path, monkeypatch):
        """Write a minimal valid beamline YAML and point the env var at it.

        Beamline-neutral on purpose — the loader's contract is "parse once",
        independent of any particular beamline's content.
        """
        cfg = tmp_path / "beamline.yaml"
        cfg.write_text(_CONFIG_TEMPLATE.format(joints="[a, b, c]"))
        monkeypatch.setenv("BEAMBOT_BEAMLINE_CONFIG", str(cfg))
        return cfg

    def test_parses_disk_at_most_once_across_many_calls(self, tmp_path, monkeypatch):
        self._write_config(tmp_path, monkeypatch)
        calls = {"n": 0}
        real = config_loader._load_beamline_config_uncached

        def counting_load():
            calls["n"] += 1
            return real()

        with patch.object(
            config_loader, "_load_beamline_config_uncached", counting_load
        ):
            results = [
                config_loader.load_beamline_config()
                for _ in range(HOT_PATH_ACCESSES)
            ]

        assert calls["n"] <= 1, (
            f"load_beamline_config() parsed the YAML {calls['n']} times over "
            f"{HOT_PATH_ACCESSES} calls; it must memoize and parse at most once. "
            f"This is the planning-latency regression guard at the root loader."
        )
        # Every caller gets the SAME object (a shared, read-only singleton),
        # not a fresh copy — this is what makes downstream reads free.
        assert all(r is results[0] for r in results)

    def test_reset_forces_a_reparse(self, tmp_path, monkeypatch):
        # The test-isolation hook must actually drop the memo so a later load
        # re-reads (e.g. a test pointing the env var at a different beamline).
        config = self._write_config(tmp_path, monkeypatch)
        assert config_loader.load_beamline_config()[0]["robot"]["arm_joints"] == ["a", "b", "c"]
        config.write_text(_CONFIG_TEMPLATE.format(joints="[replacement]"))
        config_loader.reset_beamline_config_cache()
        assert config_loader.load_beamline_config()[0]["robot"]["arm_joints"] == ["replacement"]

    def test_load_failure_is_not_cached(self, tmp_path, monkeypatch):
        # A failed load (env var unset) must NOT poison the cache: the exception
        # propagates and the next successful call still works. This is what
        # keeps the arm_joint_names() try/except fallback viable.
        with patch.dict("os.environ", {}, clear=True):
            with pytest.raises(config_loader.BeamlineConfigError):
                config_loader.load_beamline_config()
        self._write_config(tmp_path, monkeypatch)
        assert config_loader.arm_joint_names() == ["a", "b", "c"]
