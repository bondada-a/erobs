#include "mtc_pipeline/move_to_stages.hpp"
#include <moveit/task_constructor/stages/move_to.h>

MoveToStages::MoveToStages(const rclcpp::Node::SharedPtr& node)
  : BaseStages(node) {}


bool MoveToStages::run(const mtc_pipeline::action::MoveToAction::Goal& goal) {
  const std::string& planning_type = goal.planning_type;

  auto task = create_task_template("MoveTo Task");
  auto planner = (planning_type == "cartesian") ? make_cartesian_planner() : make_pipeline_planner();

  // Auto-detect target type based on fields present

  // 1. RELATIVE: Move relative to current position (e.g., "forward", "up")
  if (!goal.direction.empty() && goal.distance != 0.0) {
    const std::string& direction = goal.direction;
    const double distance = goal.distance;
    const std::string label = "move_" + direction + "_" + std::to_string(distance) + "m";
    auto stage = create_relative_move_stage(label, direction, distance, planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(stage));
  }

  // 2. POSE or NAMED STATE: Check if target exists
  else if (!goal.target.empty()) {
    const std::string& target = goal.target;

    // Parse poses JSON to check if target is a pose definition
    nlohmann::json poses;
    try {
      poses = nlohmann::json::parse(goal.poses_json);
    } catch (const nlohmann::json::exception& e) {
      RCLCPP_ERROR(node()->get_logger(), "Failed to parse poses_json: %s", e.what());
      return false;
    }

    // Check if target exists in poses JSON
    if (poses.contains(target)) {
      // 2a. POSE: Move to joint configuration from JSON config
      const auto& joint_pose_json = poses.at(target);

      if (!joint_pose_json.is_array() || joint_pose_json.size() != 6) {
        RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 joint angles", target.c_str());
        return false;
      }

      const auto joint_angles_deg = joint_pose_json.get<std::vector<double>>();
      const std::string label = planning_type == "cartesian" ? "move_to_cartesian_" + target : "move_to_" + target;

      auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
      stage->setGroup(default_arm_group_name());
      stage->setGoal(joints_from_degrees(joint_angles_deg));
      task.add(std::move(stage));
    }
    else {
      // 2b. NAMED STATE: Move to predefined SRDF state (e.g., "moveit_home")
      auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_" + target, planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
      stage->setGroup(default_arm_group_name());
      stage->setGoal(target);
      task.add(std::move(stage));
    }
  }

  else {
    RCLCPP_ERROR(node()->get_logger(), "MoveTo step missing 'target' or 'direction'/'distance' fields");
    return false;
  }

  return load_plan_execute(task);
}
