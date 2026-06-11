"""Regression guards for the planning-latency bug class: expensive work
(disk reads, YAML parsing) on a high-frequency hot path.

Background
----------
A goal→motion latency of 20-30s was traced to `MoveItLifecycleManager.ARM_JOINTS`
being a @property that re-read AND re-parsed the beamline YAML from disk on
EVERY access — and it is accessed from `_joint_state_cb`, which fires per
/joint_states message (~500 Hz). Across the executor's worker pool this
produced a continuous storm of yaml.safe_load() calls that thrashed the GIL
and starved MTC planning (proven via py-spy --native dumps). The fix memoizes
the joint set once into a frozenset.

These tests encode the *invariant* that broke ("the per-message path must not
touch disk repeatedly"), not just the resulting value — a value-only assertion
would pass on both the buggy and fixed code. The mechanism is verified to FAIL
on the old uncached implementation (5000 accesses -> 5000 disk reads) and PASS
on the fixed one (5000 accesses -> 1 disk read).

Pure-Python, no ROS spin and no hardware: we instantiate the manager via
__new__ to exercise the property without running __init__ (which needs a live
rclpy Node and creates subscriptions).
"""

from unittest.mock import patch

import pytest

import beambot.config_loader as config_loader
from beambot.core.moveit_lifecycle_manager import MoveItLifecycleManager


# Number of times the ~500 Hz callback is simulated to hit the hot path.
# Large enough that an uncached implementation racks up an unmistakable
# disk-read count, while a cached one stays at exactly 1.
HOT_PATH_ACCESSES = 5000


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


@pytest.fixture
def manager_without_init():
    """A MoveItLifecycleManager whose __init__ has NOT run.

    The ARM_JOINTS property depends only on the optional `_arm_joints_cache`
    attribute (resolved via getattr default), so a bare instance is enough to
    exercise it. This avoids needing a real ROS Node, callback groups, or
    /joint_states subscriptions.
    """
    return MoveItLifecycleManager.__new__(MoveItLifecycleManager)


class TestArmJointsCaching:
    """ARM_JOINTS must parse the beamline YAML at most once, no matter how
    many times the (500 Hz) joint-state callback reads it."""

    def test_hot_path_reads_disk_at_most_once(self, manager_without_init):
        calls = {"n": 0}
        real = config_loader.arm_joint_names

        def counting_loader():
            calls["n"] += 1
            return real()

        # Patch where the property looks it up. The property does a local
        # `from beambot.config_loader import arm_joint_names`, which binds to
        # the module attribute at call time, so patching the module attr works.
        with patch.object(config_loader, "arm_joint_names", counting_loader):
            results = [manager_without_init.ARM_JOINTS for _ in range(HOT_PATH_ACCESSES)]

        assert calls["n"] <= 1, (
            f"ARM_JOINTS hit the disk-backed config loader {calls['n']} times over "
            f"{HOT_PATH_ACCESSES} accesses; the ~500 Hz /joint_states callback must "
            f"read it at most once (memoized). This is the planning-latency regression."
        )
        assert len(results) == HOT_PATH_ACCESSES

    def test_returns_frozenset_for_o1_membership(self, manager_without_init):
        # A frozenset (not a tuple/list) makes the per-message `name in ARM_JOINTS`
        # test O(1) instead of O(n). Part of the fix; cheap to lock in.
        assert isinstance(manager_without_init.ARM_JOINTS, frozenset)

    def test_cache_is_stable_object(self, manager_without_init):
        # Repeated reads return the same cached object, not a fresh copy.
        first = manager_without_init.ARM_JOINTS
        second = manager_without_init.ARM_JOINTS
        assert first is second

    def test_contains_expected_arm_joints(self, manager_without_init):
        # Sanity: the cached set is the real UR 6-DOF arm joints, so the
        # membership filter in _joint_state_cb actually matches.
        joints = manager_without_init.ARM_JOINTS
        assert "shoulder_pan_joint" in joints
        assert "wrist_3_joint" in joints
        assert len(joints) == 6


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
        cfg.write_text("beamline: test\nrobot:\n  arm_joints: [a, b, c]\n")
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
        self._write_config(tmp_path, monkeypatch)
        first = config_loader.load_beamline_config()
        config_loader.reset_beamline_config_cache()
        second = config_loader.load_beamline_config()
        # Same content, but a distinct object: proof the reparse happened.
        assert first is not second

    def test_load_failure_is_not_cached(self):
        # A failed load (env var unset) must NOT poison the cache: the exception
        # propagates and the next successful call still works. This is what
        # keeps the arm_joint_names() try/except fallback viable.
        with patch.dict("os.environ", {}, clear=True):
            with pytest.raises(config_loader.BeamlineConfigError):
                config_loader.load_beamline_config()
        # Cache stayed empty — a subsequent call is free to retry.
        assert config_loader._config_cache is None
