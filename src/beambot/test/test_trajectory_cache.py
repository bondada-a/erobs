"""Tests for beambot.utils.trajectory_cache (multi-entry trajectory cache, no ROS)."""

import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace

from beambot.utils.trajectory_cache import (
    MAX_ENTRIES,
    TrajectoryCache,
    normalize_joints,
)
from beambot.utils.task_parser import parse_task_script


class _FakeLogger:
    """Stand-in for a ROS logger; TrajectoryCache only calls .info()."""

    def info(self, *args, **kwargs):
        pass


def _cache(max_entries: int = MAX_ENTRIES) -> TrajectoryCache:
    return TrajectoryCache(_FakeLogger(), max_entries=max_entries)


_SIX = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
_REVISION = "model-one"


# ---- normalize_joints -----------------------------------------------------

def test_normalize_keeps_whole_turn_offsets_distinct():
    # UR joints span +/-2*pi: 0.5 and 0.5 + 2*pi are different start states.
    base = normalize_joints([0.5, 0, 0, 0, 0, 0])
    assert base != normalize_joints([0.5 + 2 * math.pi, 0, 0, 0, 0, 0])
    assert base != normalize_joints([0.5 - 2 * math.pi, 0, 0, 0, 0, 0])
    assert normalize_joints([math.pi] + [0] * 5) != normalize_joints([-math.pi] + [0] * 5)


def test_normalize_treats_negative_zero_as_zero():
    assert normalize_joints([-0.001, 0, 0, 0, 0, 0]) == normalize_joints([0.001, 0, 0, 0, 0, 0])
    assert repr(normalize_joints([-0.001] * 6)) == repr(normalize_joints(_SIX))


def test_normalize_buckets_near_identical_states():
    a = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    b = [0.004, -0.004, 0.0, 0.0, 0.0, 0.0]  # < 0.01 rad jitter
    assert normalize_joints(a) == normalize_joints(b)


def test_normalize_wrist_whole_turn_is_a_different_state():
    # -270 deg and +90 deg point the wrist the same way, but the joint must
    # unwind a full turn between them, so a cached plan cannot start from both.
    deg_neg = [0, 0, 0, 0, -270.0, 0]
    deg_pos = [0, 0, 0, 0, 90.0, 0]
    n1 = normalize_joints([math.radians(d) for d in deg_neg])
    n2 = normalize_joints([math.radians(d) for d in deg_pos])
    assert n1 != n2


def test_normalize_rejects_wrong_length_and_none():
    assert normalize_joints(None) is None
    assert normalize_joints([0.0, 0.0, 0.0]) is None  # not 6
    assert normalize_joints(_SIX) is not None


# ---- compute_key --------------------------------------------------------

def test_key_stable_across_whitespace_and_field_order():
    a = '{"start_gripper": "epick", "tasks": []}'
    b = '{"tasks": [],    "start_gripper":"epick"}'  # reordered + spaces
    assert TrajectoryCache.compute_key(a, _REVISION) == TrajectoryCache.compute_key(b, _REVISION)


def test_key_differs_by_model_revision():
    j = '{"tasks": []}'
    assert TrajectoryCache.compute_key(j, "one") != TrajectoryCache.compute_key(j, "two")


def test_key_falls_back_on_unparseable_json():
    assert TrajectoryCache.compute_key("not json", _REVISION) != TrajectoryCache.compute_key(
        "different invalid json", _REVISION
    )


def test_key_separates_different_requested_moves():
    first = '{"tasks": [{"task_type": "moveto", "target": "pick"}]}'
    second = '{"tasks": [{"task_type": "moveto", "target": "place"}]}'
    assert TrajectoryCache.compute_key(first, _REVISION, _SIX) != TrajectoryCache.compute_key(
        second, _REVISION, _SIX
    )


def test_saved_pose_edit_changes_orchestrator_cache_key(tmp_path):
    # Exercise the actual key input without importing ROS/MTC.
    path = Path(__file__).parents[1] / "beambot/action_servers/orchestrator.py"
    key_call = next(
        node for node in ast.walk(ast.parse(path.read_text()))
        if isinstance(node, ast.Call)
        and ast.unparse(node.func) == "TrajectoryCache.compute_key"
    )
    key_input = compile(ast.Expression(key_call.args[0]), str(path), "eval")
    full_json = json.dumps({
        "start_gripper": "epick",
        "tasks": [{"task_type": "moveto", "target": "home"}],
    })
    inputs = {
        "json": json,
        "goal_handle": SimpleNamespace(request=SimpleNamespace(full_json=full_json)),
    }
    poses_file = tmp_path / "poses.yaml"
    keys = []
    for angle in (0, 0, 90):
        poses_file.write_text(json.dumps({"home": [angle, 0, 0, 0, 0, 0]}))
        _, tasks, poses_json = parse_task_script(
            full_json, dry_run=False, grippers={"epick": {}},
            poses_file=str(poses_file), vision_targets={},
        )
        inputs.update(tasks=tasks, poses_json=poses_json)
        keys.append(TrajectoryCache.compute_key(
            eval(key_input, inputs), _REVISION, _SIX,
        ))

    assert keys[0] == keys[1]
    assert keys[1] != keys[2]


def test_key_differs_by_start_state():
    j = '{"tasks": []}'
    at_a = TrajectoryCache.compute_key(j, _REVISION, _SIX)
    at_b = TrajectoryCache.compute_key(j, _REVISION, [1.0, 0, 0, 0, 0, 0])
    assert at_a != at_b


def test_key_same_when_start_within_tolerance():
    j = '{"tasks": []}'
    k1 = TrajectoryCache.compute_key(j, _REVISION, [0.0, 0, 0, 0, 0, 0])
    k2 = TrajectoryCache.compute_key(j, _REVISION, [0.004, 0, 0, 0, 0, 0])
    assert k1 == k2


def test_key_degrades_when_start_missing():
    # Missing joints must not masquerade as a known all-zero start state.
    j = '{"tasks": []}'
    assert TrajectoryCache.compute_key(j, _REVISION, None) != TrajectoryCache.compute_key(
        j, _REVISION, _SIX
    )


# ---- store / get ------------------------------------------------------

def test_miss_then_hit():
    c = _cache()
    assert c.get("k1") is None
    sentinel = object()
    c.store("k1", sentinel)
    assert c.get("k1") is sentinel


def test_multi_entry_coexist():
    # A->B and B->A are distinct goals → both retained, both retrievable.
    c = _cache()
    ab, ba = object(), object()
    c.store("A->B", ab)
    c.store("B->A", ba)
    assert c.get("A->B") is ab
    assert c.get("B->A") is ba


def test_store_overwrites_same_key():
    c = _cache()
    first, second = object(), object()
    c.store("k", first)
    c.store("k", second)
    assert c.get("k") is second


# ---- LRU eviction -------------------------------------------------------

def test_lru_eviction_at_cap():
    c = _cache(max_entries=2)
    c.store("a", object())
    c.store("b", object())
    c.store("c", object())  # evicts "a" (oldest)
    assert c.get("a") is None
    assert c.get("b") is not None
    assert c.get("c") is not None


def test_lru_get_keeps_hot_entry():
    c = _cache(max_entries=2)
    c.store("a", object())
    c.store("b", object())
    c.get("a")                       # touch "a" → now most-recently-used
    c.store("c", object())  # evicts "b", not "a"
    assert c.get("a") is not None
    assert c.get("b") is None
    assert c.get("c") is not None
