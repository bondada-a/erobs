"""Plan cache for the dry-run -> execute replay path.

A dry-run plans a trajectory and stashes it here; a later Execute with a
matching goal replays that exact plan instead of re-planning (OMPL would
otherwise pick a different path under the operator's feet). Validation
refuses a replay when the goal JSON or the gripper changed since the
preview, so the operator never silently executes something different from
what they previewed.

The cached ``task`` is an opaque MTC core.Task with solutions populated;
this class does not interpret it — it only keys, stores, validates, and
clears.
"""

import hashlib
import json
import threading


class PlanCache:
    """Thread-safe single-entry cache keyed on (goal JSON, gripper)."""

    def __init__(self, logger):
        self._logger = logger
        self._lock = threading.Lock()
        self._entry: dict | None = None  # {"goal_key", "task", "gripper"}

    @staticmethod
    def compute_key(full_json: str, gripper: str) -> str:
        """Stable key from goal JSON + gripper.

        Re-serializing parsed JSON with sort_keys=True normalizes whitespace
        and field ordering so byte-identical re-sends always hash the same.
        Falls back to raw bytes if the JSON can't be parsed (the cache will
        miss more often, which is fine — invalidation is the safe direction).
        """
        try:
            normalized = json.dumps(json.loads(full_json), sort_keys=True)
        except json.JSONDecodeError:
            normalized = full_json
        h = hashlib.sha256()
        h.update(normalized.encode("utf-8"))
        h.update(b"|")
        h.update(gripper.encode("utf-8"))
        return h.hexdigest()

    def has_entry(self) -> bool:
        """True if a plan is currently cached."""
        with self._lock:
            return self._entry is not None

    def store(self, goal_key: str, task, gripper: str) -> None:
        """Cache a freshly-planned task (called on a successful dry-run)."""
        with self._lock:
            self._entry = {"goal_key": goal_key, "task": task, "gripper": gripper}

    def get(self) -> dict | None:
        """Return the cached entry dict (caller reads ['task']), or None."""
        with self._lock:
            return self._entry

    def validate(self, goal_key: str, current_gripper: str) -> tuple[bool, str]:
        """Is the cached plan still safe to replay?

        Returns:
            (valid, reason). On valid=False, reason is a structured string
            starting with CACHE_<REASON>: which the GUI parses to show a
            friendly message.
        """
        with self._lock:
            entry = self._entry
            if entry is None:
                return False, "CACHE_MISS: No cached plan; planning fresh"
            if entry["goal_key"] != goal_key:
                return False, (
                    "CACHE_KEY_MISMATCH: Cached plan was for a different task. "
                    "Run Dry Run again to preview this task, then Execute."
                )
            if entry["gripper"] != current_gripper:
                return False, (
                    "CACHE_GRIPPER_CHANGED: Tool exchange happened since dry-run. "
                    "Run Dry Run again to preview with the current gripper."
                )

            # No robot-moved check here: MoveIt's trajectory_execution.
            # allowed_start_tolerance (~0.01 rad, on by default — verified in
            # libmoveit_trajectory_execution_manager.so) rejects a stale-start
            # replay BEFORE motion, so a jogged robot fails loudly instead of
            # jumping. ponytail: a friendly "robot moved, re-run Dry Run" message
            # needs the real MoveItErrorCode observed on a jog-then-Execute
            # hardware test — add the ~2-line translation then, not on a guess.
            return True, ""

    def clear(self, reason: str = "") -> None:
        """Drop the cached plan. Call after execute, on tool_exchange, or
        whenever cached MTC state should not be reused."""
        with self._lock:
            if self._entry is not None:
                if reason:
                    self._logger.info(f"Plan cache cleared: {reason}")
                self._entry = None
