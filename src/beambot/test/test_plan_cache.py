"""Tests for beambot.core.plan_cache (multi-entry trajectory cache, no ROS)."""

import math

from beambot.core.plan_cache import (
    MAX_ENTRIES,
    PlanCache,
    _wrap_to_pi,
    normalize_joints,
)


class _FakeLogger:
    """Stand-in for a ROS logger; PlanCache only calls .info()."""

    def info(self, *args, **kwargs):
        pass


def _cache(max_entries: int = MAX_ENTRIES) -> PlanCache:
    return PlanCache(_FakeLogger(), max_entries=max_entries)


_SIX = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# ---- normalize_joints / _wrap_to_pi -------------------------------------

def test_wrap_collapses_2pi_branches():
    # theta and theta +/- 2*pi map to the same canonical value.
    assert round(_wrap_to_pi(0.5), 6) == round(_wrap_to_pi(0.5 + 2 * math.pi), 6)
    assert round(_wrap_to_pi(0.5), 6) == round(_wrap_to_pi(0.5 - 2 * math.pi), 6)


def test_wrap_plus_and_minus_pi_unify():
    # +pi and -pi are the same physical angle → one canonical endpoint.
    assert _wrap_to_pi(math.pi) == _wrap_to_pi(-math.pi)


def test_normalize_buckets_near_identical_states():
    a = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    b = [0.004, -0.004, 0.0, 0.0, 0.0, 0.0]  # < 0.01 rad jitter
    assert normalize_joints(a) == normalize_joints(b)


def test_normalize_wrist_wrap_equivalent():
    # -270 deg (poses.yaml style) and +90 deg are the same physical wrist angle.
    deg_neg = [0, 0, 0, 0, -270.0, 0]
    deg_pos = [0, 0, 0, 0, 90.0, 0]
    n1 = normalize_joints([math.radians(d) for d in deg_neg])
    n2 = normalize_joints([math.radians(d) for d in deg_pos])
    assert n1 == n2


def test_normalize_rejects_wrong_length_and_none():
    assert normalize_joints(None) is None
    assert normalize_joints([0.0, 0.0, 0.0]) is None  # not 6
    assert normalize_joints(_SIX) is not None


# ---- compute_key --------------------------------------------------------

def test_key_stable_across_whitespace_and_field_order():
    a = '{"start_gripper": "epick", "tasks": []}'
    b = '{"tasks": [],    "start_gripper":"epick"}'  # reordered + spaces
    assert PlanCache.compute_key(a, "epick") == PlanCache.compute_key(b, "epick")


def test_key_differs_by_gripper():
    j = '{"tasks": []}'
    assert PlanCache.compute_key(j, "epick") != PlanCache.compute_key(j, "hande")


def test_key_falls_back_on_unparseable_json():
    assert PlanCache.compute_key("not json", "x") == PlanCache.compute_key("not json", "x")


def test_key_differs_by_start_state():
    j = '{"tasks": []}'
    at_a = PlanCache.compute_key(j, "epick", _SIX)
    at_b = PlanCache.compute_key(j, "epick", [1.0, 0, 0, 0, 0, 0])
    assert at_a != at_b


def test_key_same_when_start_within_tolerance():
    j = '{"tasks": []}'
    k1 = PlanCache.compute_key(j, "epick", [0.0, 0, 0, 0, 0, 0])
    k2 = PlanCache.compute_key(j, "epick", [0.004, 0, 0, 0, 0, 0])
    assert k1 == k2


def test_key_degrades_when_start_missing():
    # No start state (mock hardware) → stable key, just goal+gripper.
    j = '{"tasks": []}'
    assert PlanCache.compute_key(j, "epick", None) == PlanCache.compute_key(j, "epick", None)
    assert PlanCache.compute_key(j, "epick", None) != PlanCache.compute_key(j, "epick", _SIX)


# ---- store / get / has_entry / clear ------------------------------------

def test_miss_then_hit():
    c = _cache()
    assert c.get("k1") is None
    sentinel = object()
    c.store("k1", sentinel, "epick")
    assert c.get("k1")["sol_msg"] is sentinel


def test_multi_entry_coexist():
    # A->B and B->A are distinct goals → both retained, both retrievable.
    c = _cache()
    ab, ba = object(), object()
    c.store("A->B", ab, "epick")
    c.store("B->A", ba, "epick")
    assert c.get("A->B")["sol_msg"] is ab
    assert c.get("B->A")["sol_msg"] is ba


def test_abab_replays_after_intervening_store():
    # The whole point: A->B->A x N keeps replaying, no eviction of A->B by B->A.
    c = _cache()
    ab = object()
    c.store("A->B", ab, "epick")
    c.store("B->A", object(), "epick")  # intervening opposite move
    assert c.get("A->B")["sol_msg"] is ab  # still a hit


def test_store_overwrites_same_key():
    c = _cache()
    first, second = object(), object()
    c.store("k", first, "epick")
    c.store("k", second, "epick")
    assert c.get("k")["sol_msg"] is second
    assert c.size() == 1


def test_clear_flushes_all():
    c = _cache()
    c.store("k1", object(), "epick")
    c.store("k2", object(), "epick")
    c.clear("tool exchange")
    assert c.size() == 0
    assert c.get("k1") is None


# ---- LRU eviction -------------------------------------------------------

def test_lru_eviction_at_cap():
    c = _cache(max_entries=2)
    c.store("a", object(), "epick")
    c.store("b", object(), "epick")
    c.store("c", object(), "epick")  # evicts "a" (oldest)
    assert c.size() == 2
    assert c.get("a") is None
    assert c.get("b") is not None
    assert c.get("c") is not None


def test_lru_get_keeps_hot_entry():
    c = _cache(max_entries=2)
    c.store("a", object(), "epick")
    c.store("b", object(), "epick")
    c.get("a")                       # touch "a" → now most-recently-used
    c.store("c", object(), "epick")  # evicts "b", not "a"
    assert c.get("a") is not None
    assert c.get("b") is None
    assert c.get("c") is not None
