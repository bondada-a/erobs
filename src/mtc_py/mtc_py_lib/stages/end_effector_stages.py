"""EndEffector stages - Python equivalent of end_effector_stages.hpp/cpp.

Handles gripper open/close operations using SRDF-defined group states.
"""

from moveit.task_constructor import stages
from mtc_py_lib.stages.base_stages import BaseStages


class EndEffectorStages(BaseStages):
    """Handles gripper open/close operations."""

    def run(self, goal) -> bool:
        """Execute EndEffector action.

        Args:
            goal: EndEffectorAction.Goal with fields:
                - gripper_group: MoveIt group name (from config)
                - end_effector_action: SRDF state name (e.g., "hande_open")

        Returns:
            True if successful, False otherwise
        """
        if not goal.gripper_group:
            self.logger.info("No gripper group - treating as no-op success")
            return True  # No-op success for "none" or "pipettor" gripper

        # Validate we have an action to perform
        if not goal.end_effector_action:
            self.logger.error("No end_effector_action specified")
            return False

        task = self.create_task_template("EndEffector Task")
        planner = self.make_joint_interpolation_planner()

        # Create MoveTo stage for gripper
        stage = stages.MoveTo(f"gripper_{goal.end_effector_action}", planner)
        stage.group = goal.gripper_group
        stage.setGoal(goal.end_effector_action)

        task.add(stage)

        self.logger.info(
            f"Planning gripper action: {goal.end_effector_action} "
            f"(group: {goal.gripper_group})"
        )
        return self.load_plan_execute(task)
