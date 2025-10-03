#include "mtc_pipeline/tool_exchange_stages.hpp"

#include <cmath>

namespace mtc = moveit::task_constructor;

namespace {
constexpr double DOCK_SPACING_METERS = 0.1524;
}

ToolExchangeStages::ToolExchangeStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config) {}

bool ToolExchangeStages::run(const nlohmann::json& step, const nlohmann::json& poses){
  const std::string operation = step.at("operation");
  const int dock_number = step.value("dock_number", 3);
  const std::string approach_pose = step.at("approach_pose");

  const double dock_offset_y = DOCK_SPACING_METERS * static_cast<double>(3 - dock_number);
  const std::string task_name = (operation == "load") ? "Load Tool Task" :
                                 (operation == "dock") ? "Dock Tool Task" :
                                 "Tool Exchange Task";

  auto task = createTaskTemplate(task_name);
  auto sampling_planner = makePipelinePlanner();
  auto cartesian_planner = makeCartesianPlanner();

  // Lambda: Add joint move to approach pose
  const auto addNamedMoveStage = [&](const std::string& label, const std::string& pose_key) -> bool {
    const auto& joint_pose_json = poses.at(pose_key);
    if (!joint_pose_json.is_array() || joint_pose_json.size() != 6) {
      RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 joint angles", pose_key.c_str());
      return false;
    }

    const auto joint_angles_deg = joint_pose_json.get<std::vector<double>>();
    auto stage = createJointMoveStage(label, joint_angles_deg, sampling_planner);
    if (!stage) return false;
    task.add(std::move(stage));
    return true;
  };

  // Lambda: Add relative move with custom MTC visualization properties
  const auto addRelativeMoveStage = [&](const std::string& name, const std::string& direction, double distance, const std::string& marker_ns) {
    auto stage = createRelativeMoveStage(name, direction, std::abs(distance), cartesian_planner);
    if (!stage) return;

    // Set custom MTC properties for visualization
    stage->properties().set("marker_ns", marker_ns);
    stage->properties().set("link", defaultIkFrame());
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});

    task.add(std::move(stage));
  };

  // Lambda: Shift laterally to align with specific dock
  const auto addDockShiftStage = [&](double offset) {
    if (std::abs(offset) < 1e-4) return;  // Skip if offset is negligible

    const std::string direction = (offset >= 0.0) ? "right" : "left";
    addRelativeMoveStage("shift to dock", direction, offset, "dock_shift");
  };

  // ============================================================================
  // LOAD OPERATION: Attach tool from dock
  // ============================================================================
  if (operation == "load") {
    if (!addNamedMoveStage("move to load approach", approach_pose)) return false;

    addDockShiftStage(dock_offset_y);                              // Align with specific dock
    addRelativeMoveStage("attach_tool", "forward", 0.1, "approach_object");      // Move forward into tool
    addRelativeMoveStage("detach_holder", "up", 0.15, "approach_object");        // Move up to release holder
    addRelativeMoveStage("move_up", "backward", 0.2, "approach_object");         // Move back with tool
  }

  // ============================================================================
  // DOCK OPERATION: Return tool to dock
  // ============================================================================
  else if (operation == "dock") {
    if (!addNamedMoveStage("move to dock approach", approach_pose)) return false;

    addDockShiftStage(dock_offset_y);                              // Align with specific dock
    addRelativeMoveStage("align_holder", "forward", 0.2, "approach_object");     // Move forward to holder
    addRelativeMoveStage("detach_tool", "down", 0.15, "approach_object");        // Move down to release tool
    addRelativeMoveStage("dock connect", "backward", 0.1, "approach_object");    // Move back from dock
  }

  // ============================================================================
  // UNSUPPORTED OPERATION
  // ============================================================================
  else {
    RCLCPP_ERROR(node()->get_logger(), "Unknown tool exchange operation '%s'", operation.c_str());
    return false;
  }

  return loadPlanExecute(task);
}
