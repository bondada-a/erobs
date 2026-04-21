"""EndEffector stages - Python equivalent of end_effector_stages.hpp/cpp.

Handles gripper open/close operations using SRDF-defined group states.
"""

from moveit.task_constructor import core, stages
from beambot.stages.base_stages import BaseStages


class EndEffectorStages(BaseStages):
    """Handles gripper open/close operations."""

    def add_to_task(self, task: core.Task, goal, planner=None) -> str | None:
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
            None if stages were added successfully, error string on failure
        """
        if not goal.gripper_group:
            self.logger.info("No gripper group - treating as no-op success")
            return None  # No-op success for "none" or "pipettor" gripper

        # Validate we have an action to perform
        if not goal.end_effector_action:
            error = "No end_effector_action specified"
            self.logger.error(error)
            return error

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
        return None

    def run(self, goal) -> str | None:
        """Execute EndEffector action.

        Args:
            goal: EndEffectorAction.Goal with fields:
                - gripper_group: MoveIt group name (from config)
                - end_effector_action: SRDF state name (e.g., "hande_open")

        Returns:
            None if successful, error string describing failure otherwise
        """
        if not goal.gripper_group:
            self.logger.info("No gripper group - treating as no-op success")
            return None  # No-op success for "none" or "pipettor" gripper

        task = self.create_task_template("EndEffector Task")

        error = self.add_to_task(task, goal)
        if error is not None:
            return error

        return self.load_plan_execute(task)
