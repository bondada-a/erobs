"""EndEffector stages - Python equivalent of end_effector_stages.hpp/cpp.

Handles gripper open/close operations using SRDF-defined group states.
"""

from moveit.task_constructor import core, stages
from beambot.stages.base_stages import BaseStages


class EndEffectorStages(BaseStages):
    """Handles gripper open/close operations."""

    def add_to_task(self, task: core.Task, goal, planner=None) -> bool:
        """Add EndEffector stages to an existing MTC task.

        This method adds stages without creating or executing the task,
        enabling batch execution of multiple tasks.

        Args:
            task: Existing MTC Task to add stages to
            goal: EndEffectorAction.Goal with fields:
                - gripper_group: MoveIt group name (from config)
                - end_effector_action: SRDF state name (e.g., "hande_open")
            planner: Optional planner instance (creates JointInterpolation if None)

        Returns:
            True if stages were added successfully, False on error
        """
        if not goal.gripper_group:
            self.logger.info("No gripper group - treating as no-op success")
            return True  # No-op success for "none" or "pipettor" gripper

        # Validate we have an action to perform
        if not goal.end_effector_action:
            self.logger.error("No end_effector_action specified")
            return False

        # Select planner if not provided
        if planner is None:
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
        return True

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

        task = self.create_task_template("EndEffector Task")

        if not self.add_to_task(task, goal):
            return False

        return self.load_plan_execute(task)
