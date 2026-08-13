"""EndEffector stages - Python equivalent of end_effector_stages.hpp/cpp.

Handles gripper open/close operations using SRDF-defined group states.
"""

from moveit.task_constructor import core
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
        try:
            stage = self.make_gripper_stage(
                f"gripper_{goal.end_effector_action}",
                planner,
                goal.gripper_group,
                goal.end_effector_action,
            )
        except ValueError as error:
            self.logger.error(str(error))
            return str(error)

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
        task = self.create_task_template("EndEffector Task")

        error = self.add_to_task(task, goal)
        if error is not None:
            return error

        return self.load_plan_execute(task)
