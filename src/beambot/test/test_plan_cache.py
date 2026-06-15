"""Tests for beambot.core.plan_cache.PlanCache (no ROS needed)."""

from beambot.core.plan_cache import PlanCache


class _FakeLogger:
    """Stand-in for a ROS logger; PlanCache only calls .info()."""

    def info(self, *args, **kwargs):
        pass


def _cache() -> PlanCache:
    return PlanCache(_FakeLogger())


# ---- compute_key --------------------------------------------------------

def test_key_stable_across_whitespace_and_field_order():
    a = '{"start_gripper": "epick", "tasks": []}'
    b = '{"tasks": [],    "start_gripper":"epick"}'  # reordered + spaces
    assert PlanCache.compute_key(a, "epick") == PlanCache.compute_key(b, "epick")


def test_key_differs_by_gripper():
    j = '{"tasks": []}'
    assert PlanCache.compute_key(j, "epick") != PlanCache.compute_key(j, "hande")


def test_key_falls_back_on_unparseable_json():
    # Bad JSON still yields a stable, repeatable key (raw-bytes path).
    assert PlanCache.compute_key("not json", "x") == PlanCache.compute_key("not json", "x")


# ---- validate / store / get / clear ------------------------------------

def test_miss_then_hit():
    c = _cache()
    ok, reason = c.validate("k1", "epick")
    assert not ok and reason.startswith("CACHE_MISS")

    sentinel = object()
    c.store("k1", sentinel, "epick")
    ok, reason = c.validate("k1", "epick")
    assert ok and reason == ""
    assert c.get()["task"] is sentinel


def test_key_mismatch_refused():
    c = _cache()
    c.store("k1", object(), "epick")
    ok, reason = c.validate("k2", "epick")
    assert not ok and reason.startswith("CACHE_KEY_MISMATCH")


def test_gripper_change_refused():
    c = _cache()
    c.store("k1", object(), "epick")
    ok, reason = c.validate("k1", "hande")
    assert not ok and reason.startswith("CACHE_GRIPPER_CHANGED")


def test_clear_resets_to_miss():
    c = _cache()
    c.store("k1", object(), "epick")
    assert c.has_entry()
    c.clear("test")
    assert not c.has_entry()
    ok, reason = c.validate("k1", "epick")
    assert not ok and reason.startswith("CACHE_MISS")
