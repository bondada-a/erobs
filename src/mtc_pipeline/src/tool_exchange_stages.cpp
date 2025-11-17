#include "mtc_pipeline/tool_exchange_stages.hpp"

#include <cmath>
#include <moveit/task_constructor/stages/move_to.h>

namespace {
constexpr double DOCK_SPACING_METERS = 0.1524;
}

ToolExchangeStages::ToolExchangeStages(const rclcpp::Node::SharedPtr& node)
  : BaseStages(node) {}

bool ToolExchangeStages::run(const mtc_pipeline::action::ToolExchangeAction::Goal& goal){
  // Parse poses JSON
  nlohmann::json poses;
  try {
    poses = nlohmann::json::parse(goal.poses_json);
  } catch (const nlohmann::json::exception& e) {
    RCLCPP_ERROR(node()->get_logger(), "Failed to parse poses_json: %s", e.what());
    return false;
  }

  const std::string& operation = goal.operation;
  const std::string& gripper = goal.gripper;
  const std::string& current_attached = goal.current_attached_gripper;
  const int dock_number = goal.dock_number;
  const std::string& approach_pose = goal.approach_pose;

  // Validate state transitions
  if (operation == "load" && current_attached != "none") {
    RCLCPP_ERROR(node()->get_logger(),
      "Cannot load %s: %s is already attached. Dock it first.",
      gripper.c_str(), current_attached.c_str());
    return false;
  }
  if (operation == "dock" && current_attached != gripper) {
    RCLCPP_ERROR(node()->get_logger(),
      "Cannot dock %s: %s is currently attached",
      gripper.c_str(), current_attached.c_str());
    return false;
  }

  // Offset from reference dock 3: positive = right, negative = left
  const double dock_offset_y = DOCK_SPACING_METERS * static_cast<double>(3 - dock_number);
  const std::string task_name = (operation == "load") ? "Load Tool Task" :
                                 (operation == "dock") ? "Dock Tool Task" :
                                 "Tool Exchange Task";

  auto task = create_task_template(task_name);
  auto sampling_planner = make_pipeline_planner();
  auto cartesian_planner = make_cartesian_planner();

  // Validate and move to approach pose
  const auto& joint_pose_json = poses.at(approach_pose);
  if (!joint_pose_json.is_array() || joint_pose_json.size() != 6) {
    RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 joint angles", approach_pose.c_str());
    return false;
  }

  const auto joint_angles_deg = joint_pose_json.get<std::vector<double>>();
  const std::string approach_label = (operation == "load") ? "move to load approach" : "move to dock approach";
  auto approach_stage = std::make_unique<mtc::stages::MoveTo>(approach_label, sampling_planner);
  approach_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
  approach_stage->setGroup(default_arm_group_name());
  approach_stage->setGoal(joints_from_degrees(joint_angles_deg));
  task.add(std::move(approach_stage));

  // Shift laterally to align with specific dock
  if (std::abs(dock_offset_y) >= 1e-4) {
    const std::string direction = (dock_offset_y >= 0.0) ? "right" : "left";
    auto shift_stage = create_relative_move_stage("shift to dock", direction, std::abs(dock_offset_y), cartesian_planner);
    shift_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(shift_stage));
  }

  if (operation == "load") {
    auto attach_stage = create_relative_move_stage("attach tool", "forward", 0.2, cartesian_planner);
    attach_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(attach_stage));

    auto detach_stage = create_relative_move_stage("detach holder", "up", 0.15, cartesian_planner);
    detach_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(detach_stage));

    auto moveup_stage = create_relative_move_stage("move up", "backward", 0.2, cartesian_planner);
    moveup_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(moveup_stage));
  }
  else if (operation == "dock") {
    auto align_stage = create_relative_move_stage("align holder", "forward", 0.2, cartesian_planner);
    align_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(align_stage));

    auto detach_stage = create_relative_move_stage("detach tool", "down", 0.15, cartesian_planner);
    detach_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(detach_stage));

    auto dock_stage = create_relative_move_stage("dock connect", "backward", 0.2, cartesian_planner);
    dock_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(dock_stage));
  }
  else {
    RCLCPP_ERROR(node()->get_logger(), "Unknown tool exchange operation '%s'", operation.c_str());
    return false;
  }

  return load_plan_execute(task);
}
