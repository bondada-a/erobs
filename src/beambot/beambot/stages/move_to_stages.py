"""MoveTo stages - Python equivalent of move_to_stages.hpp/cpp.

Handles MoveTo operations:
- Relative moves (direction + distance)
- Target-based moves (joint poses from JSON or named SRDF states)
"""

from moveit.task_constructor import stages
from beambot.stages.base_stages import BaseStages, joints_from_degrees


class MoveToStages(BaseStages):
    """Handles MoveTo action: relative moves, joint poses, named states."""

    def run(self, goal) -> bool:
        """Execute MoveTo action.

        Args:
            goal: MoveToAction.Goal with fields:
                - target: Pose name, SRDF state, or empty for relative moves
                - planning_type: "joint" or "cartesian"
                - direction: Direction for relative moves
                - distance: Distance in meters for relative moves
                - poses_json: JSON string with pose definitions

        Returns:
            True if successful, False otherwise
        """
        task = self.create_task_template("MoveTo Task")

        # Select planner based on planning_type
        planning_type = goal.planning_type if goal.planning_type else "joint"
        if planning_type == "cartesian":
            planner = self.make_cartesian_planner()
            self.logger.info("Using Cartesian planner")
        else:
            planner = self.make_pipeline_planner()
            self.logger.info("Using pipeline planner")

        # Case 1: Relative move (direction + distance)
        if goal.direction and goal.distance != 0.0:
            stage = self.create_relative_move_stage(
                f"move_{goal.direction}_{goal.distance:.3f}m",
                goal.direction,
                goal.distance,
                planner
            )
            task.add(stage)
            self.logger.info(
                f"Planning relative move: {goal.direction} {goal.distance}m"
            )

        # Case 2: Target-based move
        elif goal.target:
            # Poses are optional for MoveTo (might use SRDF named state)
            poses = self.parse_poses(goal.poses_json)
            if poses is None:
                return False

            move_stage = stages.MoveTo(f"move_to_{goal.target}", planner)
            move_stage.group = self.arm_group
            self._set_ik_frame(move_stage)

            # Check if target is a defined joint pose in the JSON
            if goal.target in poses:
                joint_values = poses[goal.target]
                if isinstance(joint_values, list):
                    move_stage.setGoal(joints_from_degrees(joint_values))
                    self.logger.info(f"Planning move to joint pose: {goal.target}")
                else:
                    self.logger.error(
                        f"Invalid pose format for '{goal.target}': "
                        f"expected list, got {type(joint_values)}"
                    )
                    return False
            else:
                # Assume it's a named SRDF state
                move_stage.setGoal(goal.target)
                self.logger.info(f"Planning move to named state: {goal.target}")

            task.add(move_stage)

        else:
            self.logger.error(
                "No valid move target specified. "
                "Provide either (direction + distance) or target."
            )
            return False

        return self.load_plan_execute(task)
