"""EndEffector stages - Python equivalent of end_effector_stages.hpp/cpp.

Handles gripper open/close operations using SRDF-defined group states.
"""

from moveit.task_constructor import core, stages
from mtc_py_lib.stages.base_stages import BaseStages
from mtc_py_lib.utils.gripper_utils import get_group_name


class EndEffectorStages(BaseStages):
    """Handles gripper open/close operations."""

    def run(self, goal) -> bool:
        """Execute EndEffector action.

        Args:
            goal: EndEffectorAction.Goal with fields:
                - end_effector_type: "hande", "epick", or "none"
                - end_effector_action: SRDF state name (e.g., "hande_open")
                - poses_json: Unused for end effector operations

        Returns:
            True if successful, False otherwise
        """
        # Get the gripper group name from the type
        gripper_group = get_group_name(goal.end_effector_type)

        if not gripper_group:
            self.logger.info(
                f"No gripper group for type: '{goal.end_effector_type}' - "
                "treating as no-op success"
            )
            return True  # No-op success for "none" or "pipettor" gripper

        # Validate we have an action to perform
        if not goal.end_effector_action:
            self.logger.error("No end_effector_action specified")
            return False

        task = self.create_task_template("EndEffector Task")
        planner = self.make_joint_interpolation_planner()

        # Create MoveTo stage for gripper
        stage = stages.MoveTo(f"gripper_{goal.end_effector_action}", planner)
        stage.group = gripper_group
        stage.setGoal(goal.end_effector_action)

        task.add(stage)

        self.logger.info(
            f"Planning gripper action: {goal.end_effector_action} "
            f"(group: {gripper_group})"
        )
        return self.load_plan_execute(task)
