"""PickPlace stages - Python equivalent of pick_place_stages.hpp/cpp.

Pick and place sequence: approach -> grip -> retreat -> approach -> release -> retreat.
Uses constrained motion during pick to maintain tool orientation.
"""

import json
from typing import Any, Dict, Optional
from moveit.task_constructor import core, stages
from beambot.stages.base_stages import (
    BaseStages,
    create_wrist3_level_constraint,
    joints_from_degrees,
)


class PickPlaceStages(BaseStages):
    """Handles pick and place sequences with gripper operations."""

    def run(self, goal) -> bool:
        """Execute PickPlace action.

        Sequence (10 stages):
        1. Open gripper
        2. Move to pick approach (constrained)
        3. Move to pick target (constrained)
        4. Close gripper
        5. Move to pick approach (retreat, constrained)
        6. Move to place approach
        7. Move to place target
        8. Open gripper (release)
        9. Move to place approach (retreat)
        10. Return home

        Args:
            goal: PickPlaceAction.Goal with fields:
                - gripper_group: MoveIt group name (from config)
                - gripper_states_json: JSON dict of semantic states
                - pick_approach: Approach pose name
                - pick_target: Grasp pose name
                - place_approach: Place approach pose name
                - place_target: Place target pose name
                - poses_json: JSON string with pose definitions

        Returns:
            True if successful, False otherwise
        """
        self.logger.info(
            f"Pick/place: gripper_group={goal.gripper_group}, "
            f"pick={goal.pick_target}, place={goal.place_target}"
        )

        # Parse poses
        poses = self.parse_poses(goal.poses_json)
        if poses is None:
            return False

        # Parse gripper states
        try:
            gripper_states = json.loads(goal.gripper_states_json) if goal.gripper_states_json else {}
        except json.JSONDecodeError as e:
            self.logger.error(f"Invalid gripper_states_json: {e}")
            return False

        # Create task and planners
        task = self.create_task_template("Pick and Place")
        pipeline_planner = self.make_pipeline_planner()
        gripper_planner = self.make_joint_interpolation_planner()

        # === PICK SEQUENCE ===
        # 1. Open gripper
        open_stage = self.make_gripper_stage(
            "open gripper", gripper_planner, goal.gripper_group, gripper_states.get("release", "")
        )
        if open_stage:
            task.add(open_stage)

        # 2. Move to pick approach (constrained)
        stage = self._make_constrained_move_stage(
            "pickup approach", goal.pick_approach, poses, pipeline_planner
        )
        if not stage:
            return False
        task.add(stage)

        # 3. Move to pick target (constrained)
        stage = self._make_constrained_move_stage(
            "pickup", goal.pick_target, poses, pipeline_planner
        )
        if not stage:
            return False
        task.add(stage)

        # 4. Close gripper
        close_stage = self.make_gripper_stage(
            "close gripper", gripper_planner, goal.gripper_group, gripper_states.get("grasp", "")
        )
        if close_stage:
            task.add(close_stage)

        # 5. Retreat from pick (constrained)
        stage = self._make_constrained_move_stage(
            "pickup retreat", goal.pick_approach, poses, pipeline_planner
        )
        if not stage:
            return False
        task.add(stage)

        # === PLACE SEQUENCE (no wrist constraint - more flexibility needed) ===
        # 6. Move to place approach
        stage = self._make_move_to_named_stage(
            "place approach", goal.place_approach, poses, pipeline_planner
        )
        if not stage:
            return False
        task.add(stage)

        # 7. Move to place target
        stage = self._make_move_to_named_stage(
            "place", goal.place_target, poses, pipeline_planner
        )
        if not stage:
            return False
        task.add(stage)

        # 8. Release (open gripper)
        release_stage = self.make_gripper_stage(
            "release", gripper_planner, goal.gripper_group, gripper_states.get("release", "")
        )
        if release_stage:
            task.add(release_stage)

        # 9. Retreat from place
        stage = self._make_move_to_named_stage(
            "place retreat", goal.place_approach, poses, pipeline_planner
        )
        if not stage:
            return False
        task.add(stage)

        # 10. Return home
        home = stages.MoveTo("return home", pipeline_planner)
        home.group = self.arm_group
        self._set_ik_frame(home)
        home.setGoal("moveit_home")
        task.add(home)

        return self.load_plan_execute(task)

    def _make_move_to_named_stage(
        self,
        label: str,
        pose_key: str,
        poses: Dict[str, Any],
        planner
    ) -> Optional[stages.MoveTo]:
        """Create a MoveTo stage for a named joint pose."""
        joint_pose = self.get_joint_pose(poses, pose_key)
        if joint_pose is None:
            return None

        stage = stages.MoveTo(label, planner)
        stage.group = self.arm_group
        self._set_ik_frame(stage)
        stage.setGoal(joints_from_degrees(joint_pose))
        return stage

    def _make_constrained_move_stage(
        self,
        label: str,
        pose_key: str,
        poses: Dict[str, Any],
        planner
    ) -> Optional[stages.MoveTo]:
        """Create a MoveTo stage with wrist3 constraint."""
        stage = self._make_move_to_named_stage(label, pose_key, poses, planner)
        if stage:
            stage.setPathConstraints(create_wrist3_level_constraint())
        return stage
