"""Tests for beambot.batch_planner.group_into_batches()."""

import pytest
from beambot.batch_planner import group_into_batches, BATCHABLE_TYPES, BATCH_BREAKERS


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _task(task_type: str, **kwargs) -> dict:
    """Create a minimal task dict."""
    return {"task_type": task_type, **kwargs}


def _types(batches):
    """Extract (batch_type, [task_types]) for easier assertion."""
    return [
        (bt, [t["task_type"] for t in tasks])
        for bt, tasks in batches
    ]


# ---------------------------------------------------------------------------
# Basic behavior
# ---------------------------------------------------------------------------

class TestGroupIntoBatchesBasic:

    def test_empty_list(self):
        assert group_into_batches([]) == []

    def test_single_batchable(self):
        result = group_into_batches([_task("moveto")])
        assert _types(result) == [("batched", ["moveto"])]

    def test_single_non_batchable(self):
        result = group_into_batches([_task("tool_exchange")])
        assert _types(result) == [("single", ["tool_exchange"])]

    def test_single_pick_sample(self):
        """pick_sample is a batch breaker — executes as single."""
        result = group_into_batches([_task("pick_sample")])
        assert _types(result) == [("single", ["pick_sample"])]


# ---------------------------------------------------------------------------
# Consecutive batchable tasks get grouped
# ---------------------------------------------------------------------------

class TestBatching:

    def test_consecutive_moveto(self):
        tasks = [_task("moveto"), _task("moveto"), _task("moveto")]
        result = group_into_batches(tasks)
        assert _types(result) == [("batched", ["moveto", "moveto", "moveto"])]

    def test_mixed_batchable(self):
        """moveto + end_effector should batch together."""
        tasks = [_task("moveto"), _task("end_effector"), _task("moveto")]
        result = group_into_batches(tasks)
        assert _types(result) == [
            ("batched", ["moveto", "end_effector", "moveto"])
        ]

    def test_batch_breaker_splits(self):
        """A non-batchable task splits consecutive batchables."""
        tasks = [
            _task("moveto"),
            _task("moveto"),
            _task("vision_moveto"),
            _task("moveto"),
        ]
        result = group_into_batches(tasks)
        assert _types(result) == [
            ("batched", ["moveto", "moveto"]),
            ("single", ["vision_moveto"]),
            ("batched", ["moveto"]),
        ]

    def test_consecutive_breakers(self):
        """Each non-batchable becomes its own single batch."""
        tasks = [_task("tool_exchange"), _task("vision_moveto")]
        result = group_into_batches(tasks)
        assert _types(result) == [
            ("single", ["tool_exchange"]),
            ("single", ["vision_moveto"]),
        ]


# ---------------------------------------------------------------------------
# Real-world task sequences
# ---------------------------------------------------------------------------

class TestRealWorldSequences:

    def test_spincoat_to_hotplate(self):
        """Simulates the cms/tasks/spincoat_to_hotplate.json sequence."""
        tasks = [
            _task("moveto", target="pre_spincoat"),
            _task("moveto", target="spincoat"),
            _task("end_effector", end_effector_action="vacuum_on"),
            _task("moveto", target="pre_spincoat"),
            _task("moveto", target="post_spincoat"),
            _task("moveto", target="pre_hotplate"),
            _task("moveto", target="hotplate"),
            _task("end_effector", end_effector_action="vacuum_off"),
            _task("moveto", target="pre_hotplate"),
        ]
        result = group_into_batches(tasks)
        # All 9 tasks are batchable → single batch
        assert len(result) == 1
        assert result[0][0] == "batched"
        assert len(result[0][1]) == 9

    def test_vision_pick_sample_workflow(self):
        """Move → scan → vision pick → move home."""
        tasks = [
            _task("moveto", target="scan_position"),
            _task("vision_moveto", tag_id=5),
            _task("end_effector", end_effector_action="vacuum_on"),
            _task("moveto", target="place"),
            _task("end_effector", end_effector_action="vacuum_off"),
        ]
        result = group_into_batches(tasks)
        assert _types(result) == [
            ("batched", ["moveto"]),
            ("single", ["vision_moveto"]),
            ("batched", ["end_effector", "moveto", "end_effector"]),
        ]

    def test_tool_exchange_sequence(self):
        """Dock current → load new → move home."""
        tasks = [
            _task("moveto", target="dock_approach"),
            _task("tool_exchange", operation="dock"),
            _task("tool_exchange", operation="load"),
            _task("moveto", target="home"),
        ]
        result = group_into_batches(tasks)
        assert _types(result) == [
            ("batched", ["moveto"]),
            ("single", ["tool_exchange"]),
            ("single", ["tool_exchange"]),
            ("batched", ["moveto"]),
        ]


# ---------------------------------------------------------------------------
# Disabled batching
# ---------------------------------------------------------------------------

class TestDisabledBatching:

    def test_disabled_makes_all_single(self):
        tasks = [_task("moveto"), _task("end_effector"), _task("moveto")]
        result = group_into_batches(tasks, enabled=False)
        assert _types(result) == [
            ("single", ["moveto"]),
            ("single", ["end_effector"]),
            ("single", ["moveto"]),
        ]

    def test_disabled_preserves_order(self):
        tasks = [_task("moveto", target="a"), _task("moveto", target="b")]
        result = group_into_batches(tasks, enabled=False)
        assert result[0][1][0]["target"] == "a"
        assert result[1][1][0]["target"] == "b"


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:

    def test_missing_task_type(self):
        """Task with no task_type should not batch (unknown type)."""
        tasks = [{"target": "home"}]
        result = group_into_batches(tasks)
        # Empty string is not in BATCHABLE_TYPES → single
        assert len(result) == 1
        assert result[0][0] == "single"
        assert result[0][1] == [{"target": "home"}]

    def test_unknown_task_type(self):
        tasks = [_task("unknown_type")]
        result = group_into_batches(tasks)
        assert _types(result) == [("single", ["unknown_type"])]

    def test_all_batch_breaker_types(self):
        """Every BATCH_BREAKER type should be single."""
        for breaker in BATCH_BREAKERS:
            result = group_into_batches([_task(breaker)])
            assert result == [("single", [_task(breaker)])], f"{breaker} should be single"

    def test_all_batchable_types(self):
        """Every BATCHABLE_TYPES should batch together."""
        tasks = [_task(t) for t in sorted(BATCHABLE_TYPES)]
        result = group_into_batches(tasks)
        assert len(result) == 1
        assert result[0][0] == "batched"

    def test_task_data_preserved(self):
        """Batching should not modify or lose task data."""
        task = _task("moveto", target="home", planning_type="joint", distance=0.1)
        result = group_into_batches([task])
        assert result[0][1][0] is task  # Same object reference
