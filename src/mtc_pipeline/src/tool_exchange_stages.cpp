#include "mtc_pipeline/tool_exchange_stages.hpp"

#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace mtc = moveit::task_constructor;

namespace {
constexpr double DOCK_SPACING_METERS = 0.1524;
}

ToolExchangeStages::ToolExchangeStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config) {}

bool ToolExchangeStages::run(const nlohmann::json& step, const nlohmann::json& poses){
  
  const std::string operation = step.value("operation", "load");
  const int dock_number = step.value("dock_number", 3);
  const std::string approach_pose = step.at("approach_pose");

  refreshPoses(poses);

  const double dock_offset_y = DOCK_SPACING_METERS * static_cast<double>(3 - dock_number);
  const std::string& arm_group = defaultArmGroupName();
  const std::string& ik_frame = defaultIkFrame();

  std::string task_name;
  if (operation == "load") {
    task_name = "Load Tool Task";
  } else if (operation == "dock") {
    task_name = "Dock Tool Task";
  } else {
    task_name = "Tool Exchange Task";
  }

  auto task = createTaskTemplate(task_name, arm_group, ik_frame);

  // Create planners for this task
  auto sampling_planner = makePipelinePlanner();
  auto cartesian_planner = makeCartesianPlanner();

  const auto addNamedMoveStage = [&](const std::string& label, const std::string& pose_key) -> bool {
    const auto& joint_pose_json = config().at("poses").at(pose_key);
    if (!joint_pose_json.is_array() || joint_pose_json.size() != 6) {
      RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 joint angles", pose_key.c_str());
      return false;
    }

    const auto joint_angles_deg = joint_pose_json.get<std::vector<double>>();

    auto stage = std::make_unique<mtc::stages::MoveTo>(label, sampling_planner);
    stage->setGroup(arm_group);
    stage->setGoal(jointsFromDegrees(joint_angles_deg));
    task.add(std::move(stage));
    return true;
  };

  const auto addRelativeMoveStage = [&](const std::string& name, double distance, double x, double y, double z, const std::string& marker_ns) {
    auto stage = std::make_unique<mtc::stages::MoveRelative>(name, cartesian_planner);
    stage->properties().set("marker_ns", marker_ns);
    stage->properties().set("link", ik_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    stage->setMinMaxDistance(std::abs(distance), std::abs(distance));

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = ik_frame;
    vec.vector.x = x;
    vec.vector.y = y;
    vec.vector.z = z;
    stage->setDirection(vec);
    task.add(std::move(stage));
  };

  const auto addDockShiftStage = [&](double offset) {
    if (std::abs(offset) < 1e-4) {
      return;
    }
    double direction = offset >= 0.0 ? 1.0 : -1.0;
    addRelativeMoveStage("shift to dock", offset, 0.0, direction, 0.0, "dock_shift");
  };

  if (operation == "load") {
    if (!addNamedMoveStage("move to load approach", approach_pose)) {
      return false;
    }
    addDockShiftStage(dock_offset_y);
    addRelativeMoveStage("attach_tool", 0.1, 1.0, 0.0, 0.0, "approach_object");
    addRelativeMoveStage("detach_holder", 0.15, 0.0, 0.0, -1.0, "approach_object");
    addRelativeMoveStage("move_up", 0.2, -1.0, 0.0, 0.0, "approach_object");
  } else if (operation == "dock") {
    if (!addNamedMoveStage("move to dock approach", approach_pose)) {
      return false;
    }
    addDockShiftStage(dock_offset_y);
    addRelativeMoveStage("align_holder", 0.2, 1.0, 0.0, 0.0, "approach_object");
    addRelativeMoveStage("detach_tool", 0.15, 0.0, 0.0, 1.0, "approach_object");
    addRelativeMoveStage("dock connect", 0.1, -1.0, 0.0, 0.0, "approach_object");
  } else {
    RCLCPP_ERROR(node()->get_logger(), "Unknown tool exchange operation '%s'", operation.c_str());
    return false;
  }

  return loadPlanExecute(task);
}
