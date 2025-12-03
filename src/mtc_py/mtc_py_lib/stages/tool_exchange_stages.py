"""ToolExchange stages - Python equivalent of tool_exchange_stages.hpp/cpp.

Tool exchange: load/dock grippers at magnetic holder stations.
Uses Cartesian moves for precise tool attachment/detachment.
"""

from geometry_msgs.msg import PoseStamped
from moveit.task_constructor import core, stages
from mtc_py_lib.stages.base_stages import BaseStages, DOCK_SPACING_METERS


class ToolExchangeStages(BaseStages):
    """Handles tool loading and docking operations at magnetic holders."""

    def run(self, goal) -> bool:
        """Execute ToolExchange action.

        Load sequence (attaching a tool):
        1. Move to approach pose
        2. Shift laterally to align with dock
        3. Move forward to attach tool
        4. Move up to detach from holder
        5. Retreat backward

        Dock sequence (storing a tool):
        1. Move to approach pose
        2. Shift laterally to align with dock
        3. Move forward to align with holder
        4. Move down to detach tool
        5. Retreat backward

        Args:
            goal: ToolExchangeAction.Goal with fields:
                - operation: "load" or "dock"
                - gripper: "hande", "epick", or "none" - gripper being loaded/docked
                - current_attached_gripper: What's currently attached
                - dock_number: Dock position (1-5)
                - approach_pose: Approach pose name
                - poses_json: Pose definitions from task

        Returns:
            True if successful, False otherwise
        """
        # Validate state transitions
        if goal.operation == "load" and goal.current_attached_gripper != "none":
            self.logger.error(
                f"Cannot load {goal.gripper}: "
                f"{goal.current_attached_gripper} already attached"
            )
            return False

        if goal.operation == "dock" and goal.current_attached_gripper != goal.gripper:
            self.logger.error(
                f"Cannot dock {goal.gripper}: "
                f"{goal.current_attached_gripper} is attached"
            )
            return False

        # Parse poses (required for tool exchange operations)
        poses = self.parse_poses(goal.poses_json, required=True)
        if poses is None:
            return False

        task_name = "Load Tool" if goal.operation == "load" else "Dock Tool"
        task = self.create_task_template(task_name)
        sampling = self.make_pipeline_planner()
        cartesian = self.make_cartesian_planner()

        # 1. Move to approach pose
        if goal.approach_pose not in poses:
            self.logger.error(f"Approach pose '{goal.approach_pose}' not found")
            return False

        joint_pose = poses[goal.approach_pose]
        if not isinstance(joint_pose, list) or len(joint_pose) != 6:
            self.logger.error(
                f"'{goal.approach_pose}' must be array of 6 joint angles"
            )
            return False

        approach = stages.MoveTo("approach", sampling)
        approach.group = self.arm_group

        # Set ik_frame for Cartesian planning (matches C++ configureInitFrom)
        ik_frame_pose = PoseStamped()
        ik_frame_pose.header.frame_id = self.ik_frame
        approach.ik_frame = ik_frame_pose

        approach.setGoal(self.joints_from_degrees(joint_pose))
        task.add(approach)

        # 2. Lateral shift to align with dock (reference is dock 3)
        offset_y = DOCK_SPACING_METERS * (3 - goal.dock_number)
        if abs(offset_y) >= 1e-4:
            direction = "right" if offset_y >= 0 else "left"
            shift = self.create_relative_move_stage(
                "shift to dock", direction, abs(offset_y), cartesian
            )
            task.add(shift)

        # 3-5. Perform load or dock sequence
        if goal.operation == "load":
            # Move forward to attach tool
            attach = self.create_relative_move_stage(
                "attach tool", "forward", 0.2, cartesian
            )
            task.add(attach)

            # Move up to detach from holder
            detach = self.create_relative_move_stage(
                "detach holder", "up", 0.15, cartesian
            )
            task.add(detach)

            # Retreat backward
            retreat = self.create_relative_move_stage(
                "retreat", "backward", 0.2, cartesian
            )
            task.add(retreat)

        elif goal.operation == "dock":
            # Move forward to align with holder
            align = self.create_relative_move_stage(
                "align holder", "forward", 0.2, cartesian
            )
            task.add(align)

            # Move down to detach tool
            detach = self.create_relative_move_stage(
                "detach tool", "down", 0.15, cartesian
            )
            task.add(detach)

            # Retreat backward
            retreat = self.create_relative_move_stage(
                "retreat", "backward", 0.2, cartesian
            )
            task.add(retreat)

        else:
            self.logger.error(f"Unknown operation: {goal.operation}")
            return False

        return self.load_plan_execute(task)
