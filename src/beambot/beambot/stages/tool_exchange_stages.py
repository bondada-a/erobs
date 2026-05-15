"""ToolExchange stages - Python equivalent of tool_exchange_stages.hpp/cpp.

Tool exchange: load/dock grippers at magnetic holder stations.
Uses Cartesian moves for precise tool attachment/detachment.

Dock geometry (spacing, reference dock, load/dock motion sequences) is
sourced from the active beamline YAML's `tool_exchange:` block — see
config_loader for the schema. Required keys (no silent fallbacks):
  tool_exchange.dock_spacing_m, .reference_dock, .load_sequence, .dock_sequence
"""

from moveit.task_constructor import stages
from beambot.stages.base_stages import BaseStages, joints_from_degrees


def _load_tool_exchange_config() -> dict:
    """Read the `tool_exchange:` block from the active beamline YAML.

    Raises ValueError if the YAML doesn't declare `tool_exchange:` or any
    of its required keys. Silent CMS-geometry fallback would let a new
    beamline drift into wrong-physical-spacing territory without warning.
    """
    from beambot.config_loader import load_beamline_config
    cfg, _ = load_beamline_config()
    te = cfg.get("tool_exchange") or {}
    required = ("dock_spacing_m", "reference_dock", "load_sequence", "dock_sequence")
    missing = [k for k in required if te.get(k) in (None, "")]
    if missing:
        raise ValueError(
            "Active beamline YAML is missing required tool_exchange keys: "
            f"{missing}. Add a `tool_exchange:` block to "
            "$BEAMBOT_BEAMLINE_CONFIG declaring all of: "
            f"{required}."
        )
    return te


class ToolExchangeStages(BaseStages):
    """Handles tool loading and docking operations at magnetic holders."""

    def run(self, goal) -> str | None:
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
            None if successful, error string describing failure otherwise
        """
        self.logger.info(
            f"Executing tool exchange: operation={goal.operation}, "
            f"gripper={goal.gripper}, dock={goal.dock_number}"
        )

        # Resolve dock geometry from beamline YAML — required, no fallback.
        try:
            te_cfg = _load_tool_exchange_config()
        except ValueError as e:
            self.logger.error(str(e))
            return str(e)
        dock_spacing = float(te_cfg["dock_spacing_m"])
        reference_dock = int(te_cfg["reference_dock"])
        load_sequence = te_cfg["load_sequence"]
        dock_sequence = te_cfg["dock_sequence"]

        # Validate state transitions
        if goal.operation == "load" and goal.current_attached_gripper != "none":
            error = (
                f"Cannot load {goal.gripper}: "
                f"{goal.current_attached_gripper} already attached"
            )
            self.logger.error(error)
            return error

        if goal.operation == "dock" and goal.current_attached_gripper != goal.gripper:
            error = (
                f"Cannot dock {goal.gripper}: "
                f"{goal.current_attached_gripper} is attached"
            )
            self.logger.error(error)
            return error

        # Parse poses
        poses = self.parse_poses(goal.poses_json)
        if poses is None:
            return "Failed to parse poses_json for tool_exchange"

        task_name = "Load Tool" if goal.operation == "load" else "Dock Tool"
        task = self.create_task_template(task_name)
        sampling = self.make_pipeline_planner()
        cartesian = self.make_cartesian_planner()

        # 1. Move to approach pose
        joint_pose = self.get_joint_pose(poses, goal.approach_pose)
        if joint_pose is None:
            return f"Pose '{goal.approach_pose}' not found or invalid (tool exchange approach)"

        approach = stages.MoveTo("approach", sampling)
        approach.group = self.arm_group
        self._set_ik_frame(approach)
        approach.setGoal(joints_from_degrees(joint_pose))
        task.add(approach)

        # 2. Lateral shift to align with the requested dock
        offset_y = dock_spacing * (reference_dock - goal.dock_number)
        if abs(offset_y) >= 1e-4:
            direction = "right" if offset_y >= 0 else "left"
            shift = self.create_relative_move_stage(
                "shift to dock", direction, abs(offset_y), cartesian
            )
            task.add(shift)

        # 3-5. Perform the configured load or dock sequence
        if goal.operation == "load":
            sequence = load_sequence
            stage_prefix = "load"
        elif goal.operation == "dock":
            sequence = dock_sequence
            stage_prefix = "dock"
        else:
            return f"Unknown tool exchange operation: '{goal.operation}' (expected 'load' or 'dock')"

        for idx, move in enumerate(sequence):
            direction = move.get("direction", "")
            distance = float(move.get("distance", 0.0))
            if not direction or distance <= 0:
                return f"Invalid {goal.operation}_sequence step {idx}: {move}"
            stage = self.create_relative_move_stage(
                f"{stage_prefix} step {idx + 1}: {direction} {distance:.3f}m",
                direction, distance, cartesian,
            )
            task.add(stage)

        return self.load_plan_execute(task)
