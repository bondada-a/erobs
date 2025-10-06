#include "mtc_pipeline/tool_exchange_stages.hpp"

#include <cmath>
#include <moveit/task_constructor/stages/move_to.h>

namespace mtc = moveit::task_constructor;

namespace {
constexpr double DOCK_SPACING_METERS = 0.1524;
}

ToolExchangeStages::ToolExchangeStages(const rclcpp::Node::SharedPtr& node)
  : BaseStages(node) {}

bool ToolExchangeStages::run(const nlohmann::json& step, const nlohmann::json& poses){
  const std::string operation = step.at("operation");
  const int dock_number = step.value("dock_number", 3);
  const std::string approach_pose = step.at("approach_pose");

  const double dock_offset_y = DOCK_SPACING_METERS * static_cast<double>(3 - dock_number);
  const std::string task_name = (operation == "load") ? "Load Tool Task" :
                                 (operation == "dock") ? "Dock Tool Task" :
                                 "Tool Exchange Task";

  auto task = create_task_template(task_name);
  auto sampling_planner = make_pipeline_planner();
  auto cartesian_planner = make_cartesian_planner();

  // ============================================================================
  // LOAD OPERATION: Attach tool from dock
  // ============================================================================
  if (operation == "load") {
    // Move to approach pose
    const auto& joint_pose_json = poses.at(approach_pose);
    if (!joint_pose_json.is_array() || joint_pose_json.size() != 6) {
      RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 joint angles", approach_pose.c_str());
      return false;
    }

    const auto joint_angles_deg = joint_pose_json.get<std::vector<double>>();
    auto approach_stage = std::make_unique<mtc::stages::MoveTo>("move to load approach", sampling_planner);
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

    // Execute tool loading sequence
    auto attach_stage = create_relative_move_stage("attach_tool", "forward", 0.1, cartesian_planner);
    attach_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(attach_stage));

    auto detach_stage = create_relative_move_stage("detach_holder", "up", 0.15, cartesian_planner);
    detach_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(detach_stage));

    auto moveup_stage = create_relative_move_stage("move_up", "backward", 0.2, cartesian_planner);
    moveup_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(moveup_stage));
  }

  // ============================================================================
  // DOCK OPERATION: Return tool to dock
  // ============================================================================
  else if (operation == "dock") {
    // Move to approach pose
    const auto& joint_pose_json = poses.at(approach_pose);
    if (!joint_pose_json.is_array() || joint_pose_json.size() != 6) {
      RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 joint angles", approach_pose.c_str());
      return false;
    }

    const auto joint_angles_deg = joint_pose_json.get<std::vector<double>>();
    auto approach_stage = std::make_unique<mtc::stages::MoveTo>("move to dock approach", sampling_planner);
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

    // Execute tool docking sequence
    auto align_stage = create_relative_move_stage("align_holder", "forward", 0.2, cartesian_planner);
    align_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(align_stage));

    auto detach_stage = create_relative_move_stage("detach_tool", "down", 0.15, cartesian_planner);
    detach_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(detach_stage));

    auto dock_stage = create_relative_move_stage("dock connect", "backward", 0.1, cartesian_planner);
    dock_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    task.add(std::move(dock_stage));
  }

  // ============================================================================
  // UNSUPPORTED OPERATION
  // ============================================================================
  else {
    RCLCPP_ERROR(node()->get_logger(), "Unknown tool exchange operation '%s'", operation.c_str());
    return false;
  }

  return load_plan_execute(task);
}
