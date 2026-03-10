"""PickPlace stages - dedicated pick and place action.

Provides a convenient single-action interface for the most common robot operation:
picking an object from one location and placing it at another.

Two execution modes are available:

1. run() - DEFAULT: All 9 stages in one MTC task
   Single continuous trajectory - fastest execution, smoothest motion.

2. run_with_gripper_settle() - Split into 3 MTC tasks
   Task 1 (Pick):      open gripper → approach → grasp → close gripper
   Task 2 (Transport): retreat → approach place → place → open gripper
   Task 3 (Retreat):   retreat from place

   The planning time between tasks provides natural delays for gripper settling.
   Use this if gripper needs more time to close/open before the next motion.

Uses joint-space planning (OMPL) for all movements.
"""

import json
from typing import Any, Dict, Optional
from moveit.task_constructor import stages
from beambot.stages.base_stages import (
    BaseStages,
    joints_from_degrees,
    parse_constraints,
)


class PickPlaceStages(BaseStages):
    """Handles pick and place sequences with gripper operations."""

    def run(self, goal) -> 'Optional[str]':
        """Execute PickPlace as a single MTC task (all 9 stages).

        This is the default implementation where all stages are planned and
        executed together as one continuous trajectory.

        Sequence (9 stages):
          1. Open gripper
          2. Move to pick approach
          3. Move to pick target (grasp)
          4. Close gripper
          5. Retreat to pick approach
          6. Move to place approach
          7. Move to place target
          8. Open gripper (release)
          9. Retreat to place approach

        Args:
            goal: PickPlaceAction.Goal with fields:
                - gripper_group: MoveIt group name (e.g., "hande_gripper")
                - gripper_states_json: JSON dict {"grasp": "hande_closed", "release": "hande_open"}
                - pick_approach: Approach pose name (key in poses_json)
                - pick_target: Grasp pose name (key in poses_json)
                - place_approach: Place approach pose name (key in poses_json)
                - place_target: Place target pose name (key in poses_json)
                - poses_json: JSON string with pose definitions (joint angles in degrees)

        Returns:
            None if successful, error string describing failure otherwise
        """
        self.logger.info(
            f"Pick/place: gripper_group={goal.gripper_group}, "
            f"pick={goal.pick_target}, place={goal.place_target}"
        )

        # Parse poses
        poses = self.parse_poses(goal.poses_json)
        if poses is None:
            return "Failed to parse poses_json for pick_and_place"

        # Parse gripper states
        try:
            gripper_states = json.loads(goal.gripper_states_json) if goal.gripper_states_json else {}
        except json.JSONDecodeError as e:
            return f"Invalid gripper_states_json: {e}"

        # Parse optional path constraints
        constraints = parse_constraints(
            json.loads(goal.constraints_json) if goal.constraints_json else None
        )

        # Create single task with all stages
        task = self.create_task_template("Pick and Place")
        gripper_planner = self.make_joint_interpolation_planner()

        # === PICK SEQUENCE ===
        # 1. Open gripper
        open_stage = self.make_gripper_stage(
            "open gripper", gripper_planner, goal.gripper_group, gripper_states.get("release", "")
        )
        if open_stage:
            task.add(open_stage)

        # 2. Move to pick approach
        stage = self.make_move_to_named_stage(
            "pick approach", goal.pick_approach, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.pick_approach}' not found or invalid in poses_json (pick approach)"
        task.add(stage)

        # 3. Move to pick target (grasp position)
        stage = self.make_move_to_named_stage(
            "pick", goal.pick_target, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.pick_target}' not found or invalid in poses_json (pick target)"
        task.add(stage)

        # 4. Close gripper
        close_stage = self.make_gripper_stage(
            "close gripper", gripper_planner, goal.gripper_group, gripper_states.get("grasp", "")
        )
        if close_stage:
            task.add(close_stage)

        # 5. Retreat from pick (back to approach)
        stage = self.make_move_to_named_stage(
            "pick retreat", goal.pick_approach, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.pick_approach}' not found or invalid in poses_json (pick retreat)"
        task.add(stage)

        # === PLACE SEQUENCE ===
        # 6. Move to place approach
        stage = self.make_move_to_named_stage(
            "place approach", goal.place_approach, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.place_approach}' not found or invalid in poses_json (place approach)"
        task.add(stage)

        # 7. Move to place target
        stage = self.make_move_to_named_stage(
            "place", goal.place_target, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.place_target}' not found or invalid in poses_json (place target)"
        task.add(stage)

        # 8. Release (open gripper)
        release_stage = self.make_gripper_stage(
            "release", gripper_planner, goal.gripper_group, gripper_states.get("release", "")
        )
        if release_stage:
            task.add(release_stage)

        # 9. Retreat from place
        stage = self.make_move_to_named_stage(
            "place retreat", goal.place_approach, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.place_approach}' not found or invalid in poses_json (place retreat)"
        task.add(stage)

        return self.load_plan_execute(task)

    def run_with_gripper_settle(self, goal) -> 'Optional[str]':
        """Execute PickPlace split into 3 MTC tasks for gripper settling.

        Use this when the gripper needs more time to fully close/open before
        the next motion begins. The planning time between tasks provides
        natural delays.

        Task 1 - PICK (4 stages):
          1. Open gripper
          2. Move to pick approach
          3. Move to pick target (grasp position)
          4. Close gripper

        Task 2 - TRANSPORT & PLACE (4 stages):
          5. Retreat to pick approach
          6. Move to place approach
          7. Move to place target
          8. Open gripper (release)

        Task 3 - RETREAT (1 stage):
          9. Retreat to place approach

        Args:
            goal: PickPlaceAction.Goal (same as run())

        Returns:
            None if successful, error string describing failure otherwise
        """
        self.logger.info(
            f"Pick/place (with settle): gripper_group={goal.gripper_group}, "
            f"pick={goal.pick_target}, place={goal.place_target}"
        )

        # Parse poses
        poses = self.parse_poses(goal.poses_json)
        if poses is None:
            return "Failed to parse poses_json for pick_and_place (settle mode)"

        # Parse gripper states
        try:
            gripper_states = json.loads(goal.gripper_states_json) if goal.gripper_states_json else {}
        except json.JSONDecodeError as e:
            return f"Invalid gripper_states_json: {e}"

        # Parse optional path constraints
        constraints = parse_constraints(
            json.loads(goal.constraints_json) if goal.constraints_json else None
        )

        # === TASK 1: PICK ===
        self.logger.info("Task 1/3: Executing pick sequence...")
        error = self._execute_pick(goal, poses, gripper_states, constraints)
        if error is not None:
            return f"Pick sequence failed: {error}"

        # === TASK 2: TRANSPORT & PLACE ===
        self.logger.info("Task 2/3: Executing transport and place sequence...")
        error = self._execute_transport_and_place(goal, poses, gripper_states, constraints)
        if error is not None:
            return f"Transport and place failed: {error}"

        # === TASK 3: RETREAT ===
        self.logger.info("Task 3/3: Executing retreat sequence...")
        error = self._execute_retreat(goal, poses, constraints)
        if error is not None:
            return f"Retreat failed: {error}"

        self.logger.info("Pick and place completed successfully")
        return None

    def _execute_pick(self, goal, poses: Dict[str, Any], gripper_states: Dict[str, str], constraints=None) -> 'Optional[str]':
        """Execute the pick sequence (open → approach → grasp → close).

        Returns:
            None if successful, error string on failure
        """
        task = self.create_task_template("Pick")
        gripper_planner = self.make_joint_interpolation_planner()

        # 1. Open gripper
        open_stage = self.make_gripper_stage(
            "open gripper", gripper_planner, goal.gripper_group, gripper_states.get("release", "")
        )
        if open_stage:
            task.add(open_stage)

        # 2. Move to pick approach
        stage = self.make_move_to_named_stage(
            "pick approach", goal.pick_approach, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.pick_approach}' not found or invalid (pick approach)"
        task.add(stage)

        # 3. Move to pick target (grasp position)
        stage = self.make_move_to_named_stage(
            "pick", goal.pick_target, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.pick_target}' not found or invalid (pick target)"
        task.add(stage)

        # 4. Close gripper
        close_stage = self.make_gripper_stage(
            "close gripper", gripper_planner, goal.gripper_group, gripper_states.get("grasp", "")
        )
        if close_stage:
            task.add(close_stage)

        return self.load_plan_execute(task)

    def _execute_transport_and_place(self, goal, poses: Dict[str, Any], gripper_states: Dict[str, str], constraints=None) -> 'Optional[str]':
        """Execute transport and place (retreat → approach → place → release).

        Returns:
            None if successful, error string on failure
        """
        task = self.create_task_template("Transport and Place")
        gripper_planner = self.make_joint_interpolation_planner()

        # 5. Retreat from pick (back to approach)
        stage = self.make_move_to_named_stage(
            "pick retreat", goal.pick_approach, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.pick_approach}' not found or invalid (pick retreat)"
        task.add(stage)

        # 6. Move to place approach
        stage = self.make_move_to_named_stage(
            "place approach", goal.place_approach, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.place_approach}' not found or invalid (place approach)"
        task.add(stage)

        # 7. Move to place target
        stage = self.make_move_to_named_stage(
            "place", goal.place_target, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.place_target}' not found or invalid (place target)"
        task.add(stage)

        # 8. Release (open gripper)
        release_stage = self.make_gripper_stage(
            "release", gripper_planner, goal.gripper_group, gripper_states.get("release", "")
        )
        if release_stage:
            task.add(release_stage)

        return self.load_plan_execute(task)

    def _execute_retreat(self, goal, poses: Dict[str, Any], constraints=None) -> 'Optional[str]':
        """Execute retreat from place.

        Returns:
            None if successful, error string on failure
        """
        task = self.create_task_template("Retreat")

        # 9. Retreat from place
        stage = self.make_move_to_named_stage(
            "place retreat", goal.place_approach, poses, constraints=constraints
        )
        if not stage:
            return f"Pose '{goal.place_approach}' not found or invalid (place retreat)"
        task.add(stage)

        return self.load_plan_execute(task)

