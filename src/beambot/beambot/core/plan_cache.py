"""Trajectory cache: keyed on (start joints, goal, gripper, model revision).

Stores a planned MTC solution as a serialized
``moveit_task_constructor_msgs/Solution`` msg (NOT a live ``core.Task`` — that
holds references into the per-process RobotModel and dangles when MoveIt
relaunches on tool-exchange). On an exact (normalized start joints, goal JSON,
gripper) key hit, the orchestrator replays the stored msg via
``/execute_task_solution`` instead of re-planning.

One store serves two jobs:
  - **consistency**: a dry-run preview, then Execute replays that exact
    trajectory rather than a freshly re-planned (different) OMPL path.
  - **performance**: repeated identical moves (A->B->A x15) replay after the
    first plan instead of paying ~4 s of planning each time.

Start-state safety is downstream and fails *safe*: MoveIt's
``trajectory_execution.allowed_start_tolerance`` (~0.01 rad, on by default)
rejects a stale-start replay BEFORE motion, so a jogged robot fails loudly
instead of jumping. The start key is quantized to that same ~0.01 rad grid, so
any key hit is within the tolerance MoveIt would itself accept for replay.

The cache does NOT re-check collisions: a replayed trajectory is only valid
against the scene it was planned in. For a static cell this is fine; if the
scene can change between identical moves, gate replay on a scene re-check
upstream (see the orchestrator) or assert scene-invariance.
"""

import hashlib
import json
import math
import threading
from collections import OrderedDict

# ponytail: FIFO/LRU cap so a long session of one-off moves can't grow the
# cache without bound. Bump if a workflow legitimately has more distinct
# (start, goal, gripper, model revision) tuples in flight than this.
MAX_ENTRIES = 64

_TWO_PI = 2.0 * math.pi

# Start-joint rounding grid: 2 decimal radians (~0.01 rad) matches MoveIt's
# default trajectory_execution.allowed_start_tolerance, so any key hit is a
# replay MoveIt would itself accept. If that tolerance is ever tuned per
# beamline, read this from the YAML alongside it rather than editing here.
_BUCKET_NDIGITS = 2


def _wrap_to_pi(theta: float) -> float:
    """Wrap a radian angle to (-pi, pi].

    Collapses theta and theta +/- 2*pi*k to one canonical value. The UR5e arm
    joints have +/-2*pi limits, so the same physical pose can report theta or
    theta+2*pi (poses.yaml e.g. stores wrist_2 near -270 deg == +90 deg); an
    un-wrapped joint-value key would miss on those equivalent branches.
    """
    w = (theta + math.pi) % _TWO_PI - math.pi
    if w == -math.pi:  # half-open: +pi and -pi are the same physical angle
        w = math.pi
    return w


def normalize_joints(positions_rad):
    """Canonical, hashable start-state key component, or None.

    ``positions_rad``: 6 arm joint angles in RADIANS, already in canonical group
    order (caller drops any gripper joint and remaps name->position order — the
    live /joint_states order differs from the kinematic order). Wrap to (-pi,pi]
    FIRST (collapse 2*pi branches), THEN round to ~0.01 rad buckets (matches
    allowed_start_tolerance). Returns None when fewer/more than 6 values are
    available (startup, mock hardware) so the caller omits only the start-state
    component; goal, gripper, and model revision remain in the key.
    """
    if positions_rad is None:
        return None
    vals = list(positions_rad)
    if len(vals) != 6:
        return None
    return tuple(round(_wrap_to_pi(float(p)), _BUCKET_NDIGITS) for p in vals)


class PlanCache:
    """Thread-safe cache keyed on (start, goal, gripper, model revision).

    Values are opaque to this class: it stores whatever ``sol_msg`` object the
    caller hands it (a ``moveit_task_constructor_msgs/Solution`` msg in
    production, a sentinel in tests) and only keys, stores, fetches, and
    evicts. Every component is folded into the key, so a key hit IS a valid
    match — there is no separate validate step.
    """

    def __init__(self, logger, max_entries: int = MAX_ENTRIES):
        self._logger = logger
        self._lock = threading.Lock()
        self._max_entries = max_entries
        # key -> {"sol_msg", "gripper"}; ordered so we can evict the LRU entry.
        self._entries: "OrderedDict[str, dict]" = OrderedDict()

    @staticmethod
    def compute_key(
        full_json: str,
        gripper: str,
        model_revision: str,
        start_joints_rad=None,
    ) -> str:
        """Stable key from start joints, goal, gripper, and model revision.

        The GOAL is keyed on ``full_json`` text — the task script already
        uniquely describes named / relative / cartesian goals — so no
        joint-value normalization is needed on the goal side, sidestepping the
        wrist-wrap problem there. Re-serializing parsed JSON with
        ``sort_keys=True`` normalizes whitespace/field order so byte-identical
        re-sends hash the same; unparseable JSON falls back to raw bytes (the
        cache just misses more often, which is the safe direction).

        The START component is the current arm joints, wrapped+quantized via
        ``normalize_joints``. When unavailable (mock hardware, not-yet-received)
        it is ``None`` and the key omits only the start state.
        """
        try:
            normalized = json.dumps(json.loads(full_json), sort_keys=True)
        except json.JSONDecodeError:
            normalized = full_json
        start = normalize_joints(start_joints_rad)
        h = hashlib.sha256()
        h.update(normalized.encode("utf-8"))
        h.update(b"|")
        h.update(gripper.encode("utf-8"))
        h.update(b"|")
        h.update(model_revision.encode("utf-8"))
        h.update(b"|")
        h.update(repr(start).encode("utf-8"))
        return h.hexdigest()

    def store(self, key: str, sol_msg, gripper: str) -> None:
        """Cache a planned Solution msg under its key (overwrites the same key).

        Evicts the least-recently-used entry once the cap is exceeded.
        """
        with self._lock:
            self._entries[key] = {"sol_msg": sol_msg, "gripper": gripper}
            self._entries.move_to_end(key)
            while len(self._entries) > self._max_entries:
                evicted, _ = self._entries.popitem(last=False)
                self._logger.info(f"Plan cache evicted LRU entry {evicted[:8]}…")

    def get(self, key: str) -> dict | None:
        """Return the cached entry for ``key`` (caller reads ['sol_msg']), or
        None. A hit marks the entry most-recently-used."""
        with self._lock:
            entry = self._entries.get(key)
            if entry is not None:
                self._entries.move_to_end(key)
            return entry

    def clear(self, reason: str = "") -> None:
        """Flush ALL entries.

        Not wired to any caller — tool/cup changes invalidate via
        ``model_revision`` in the key. Escape hatch for a scene change that
        invalidates cached paths without bumping the revision.
        """
        with self._lock:
            n = len(self._entries)
            if n and reason:
                self._logger.info(f"Plan cache cleared ({n} entries): {reason}")
            self._entries.clear()

    def size(self) -> int:
        """Number of cached entries (for tests / introspection)."""
        with self._lock:
            return len(self._entries)
