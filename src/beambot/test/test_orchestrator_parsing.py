"""Tests for orchestrator JSON parsing logic.

Since MTCOrchestratorServer requires rclpy.init() and a full node,
these tests validate the JSON contract independently — ensuring that
the task script format is correctly understood and edge cases handled.

The batch_planner tests cover task grouping. These tests cover the
JSON structure that feeds into the orchestrator.
"""

import json
import math

import pytest

from beambot_interfaces.action import MTCExecution


# ---------------------------------------------------------------------------
# Task JSON construction helpers
# ---------------------------------------------------------------------------

def _make_script(start_gripper="epick", tasks=None, poses=None) -> str:
    """Build a valid task script JSON string."""
    return json.dumps({
        "start_gripper": start_gripper,
        "tasks": tasks or [],
        "poses": poses or {},
    })


# ---------------------------------------------------------------------------
# Valid JSON structure tests
# ---------------------------------------------------------------------------

class TestValidTaskScripts:

    def test_minimal_valid(self):
        """Minimal valid script: gripper + empty tasks."""
        script = json.loads(_make_script())
        assert script["start_gripper"] == "epick"
        assert script["tasks"] == []
        assert script["poses"] == {}

    def test_moveto_with_pose(self):
        script = json.loads(_make_script(
            tasks=[{"task_type": "moveto", "target": "home"}],
            poses={"home": [0, -90, 90, -90, -90, 0]},
        ))
        assert len(script["tasks"]) == 1
        assert script["tasks"][0]["task_type"] == "moveto"
        assert len(script["poses"]["home"]) == 6

    def test_pick_and_place(self):
        script = json.loads(_make_script(
            tasks=[{
                "task_type": "pick_and_place",
                "pick_approach": "pa",
                "pick_target": "pt",
                "place_approach": "pla",
                "place_target": "plt",
            }],
            poses={
                "pa": [10, 20, 30, 40, 50, 60],
                "pt": [11, 21, 31, 41, 51, 61],
                "pla": [12, 22, 32, 42, 52, 62],
                "plt": [13, 23, 33, 43, 53, 63],
            },
        ))
        task = script["tasks"][0]
        assert task["pick_approach"] == "pa"
        assert "pa" in script["poses"]

    def test_vision_moveto(self):
        script = json.loads(_make_script(
            tasks=[{
                "task_type": "vision_moveto",
                "tag_id": 5,
                "detection_type": "marker",
                "z_offset": 0.02,
            }],
        ))
        task = script["tasks"][0]
        assert task["tag_id"] == 5
        assert task["detection_type"] == "marker"

    def test_tool_exchange(self):
        script = json.loads(_make_script(
            start_gripper="epick",
            tasks=[
                {"task_type": "tool_exchange", "operation": "dock", "gripper": "epick", "dock_number": 1, "approach_pose": "dock_approach"},
                {"task_type": "tool_exchange", "operation": "load", "gripper": "hande", "dock_number": 2, "approach_pose": "load_approach"},
            ],
            poses={
                "dock_approach": [10, 20, 30, 40, 50, 60],
                "load_approach": [11, 21, 31, 41, 51, 61],
            },
        ))
        assert script["tasks"][0]["operation"] == "dock"
        assert script["tasks"][1]["operation"] == "load"

    def test_relative_move(self):
        script = json.loads(_make_script(
            tasks=[{
                "task_type": "moveto",
                "target": "",
                "planning_type": "cartesian",
                "direction": "backward",
                "distance": 0.1,
            }],
        ))
        task = script["tasks"][0]
        assert task["direction"] == "backward"
        assert task["distance"] == 0.1

    def test_cartesian_target_3dof(self):
        script = json.loads(_make_script(
            tasks=[{
                "task_type": "moveto",
                "target": "",
                "cartesian_target": [0.3, -0.2, 0.15],
            }],
        ))
        assert len(script["tasks"][0]["cartesian_target"]) == 3

    def test_cartesian_target_6dof(self):
        script = json.loads(_make_script(
            tasks=[{
                "task_type": "moveto",
                "target": "",
                "cartesian_target": [0.3, -0.2, 0.15, 180, 0, 0],
                "frame_id": "base_link",
            }],
        ))
        assert len(script["tasks"][0]["cartesian_target"]) == 6

    def test_constraints_in_moveto(self):
        script = json.loads(_make_script(
            tasks=[{
                "task_type": "moveto",
                "target": "scan",
                "constraints": {
                    "joint_constraints": [{
                        "joint_name": "wrist_3_joint",
                        "position": 0.0,
                        "tolerance_above": 5.0,
                        "tolerance_below": 5.0,
                    }]
                }
            }],
            poses={"scan": [0, -90, 90, -90, -90, 0]},
        ))
        assert "constraints" in script["tasks"][0]

    def test_all_gripper_types(self):
        for gripper in ["hande", "epick", "pipettor", "none"]:
            script = json.loads(_make_script(start_gripper=gripper))
            assert script["start_gripper"] == gripper

    def test_pipettor_task(self):
        script = json.loads(_make_script(
            start_gripper="pipettor",
            tasks=[{
                "task_type": "pipettor",
                "operation": "SUCK",
                "volume_pct": 0.55,
            }],
        ))
        task = script["tasks"][0]
        assert task["operation"] == "SUCK"
        assert task["volume_pct"] == 0.55

    def test_multi_step_sequence(self):
        """Full realistic sequence with multiple task types."""
        script = json.loads(_make_script(
            start_gripper="epick",
            tasks=[
                {"task_type": "moveto", "target": "scan_position"},
                {"task_type": "vision_moveto", "tag_id": 5, "detection_type": "marker"},
                {"task_type": "end_effector", "end_effector_action": "vacuum_on"},
                {"task_type": "moveto", "target": "place_approach"},
                {"task_type": "moveto", "target": "place"},
                {"task_type": "end_effector", "end_effector_action": "vacuum_off"},
                {"task_type": "moveto", "target": "home"},
            ],
            poses={
                "scan_position": [13.38, -112.45, -65.22, -90.98, -267.33, -166.94],
                "place_approach": [65.63, -106.91, -69.8, -93.6, -270.12, -206.3],
                "place": [65.63, -106.82, -73.45, -90.04, -270.12, -206.28],
                "home": [0, -90, 90, -90, -90, 0],
            },
        ))
        assert len(script["tasks"]) == 7
        assert len(script["poses"]) == 4


# ---------------------------------------------------------------------------
# Invalid JSON tests (documents the contract)
# ---------------------------------------------------------------------------

class TestInvalidTaskScripts:

    def test_not_json(self):
        with pytest.raises(json.JSONDecodeError):
            json.loads("not json")

    def test_missing_start_gripper(self):
        script = json.loads('{"tasks": []}')
        assert "start_gripper" not in script

    def test_missing_tasks(self):
        script = json.loads('{"start_gripper": "epick"}')
        assert "tasks" not in script

    def test_poses_joint_count(self):
        """Poses should have exactly 6 elements."""
        script = json.loads(_make_script(
            poses={"bad": [0, 0, 0]}  # Only 3 joints
        ))
        assert len(script["poses"]["bad"]) != 6


# ---------------------------------------------------------------------------
# Goal message construction
# ---------------------------------------------------------------------------

class TestGoalMessageConstruction:

    def test_goal_accepts_json_string(self):
        """MTCExecution.Goal.full_json should accept a string."""
        goal = MTCExecution.Goal()
        goal.full_json = _make_script()
        parsed = json.loads(goal.full_json)
        assert parsed["start_gripper"] == "epick"

    def test_poses_degrees_convention(self):
        """Verify poses are in degrees (matching CLAUDE.md convention)."""
        poses = {"home": [0, -90, 90, -90, -90, 0]}
        # These values in radians would be nonsensical for a UR5e
        # (e.g., -90 radians = ~14.3 full rotations)
        for angle in poses["home"]:
            assert -360 <= angle <= 360, f"Angle {angle} looks like radians, not degrees"
