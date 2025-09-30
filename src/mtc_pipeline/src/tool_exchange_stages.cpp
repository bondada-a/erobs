#include "mtc_pipeline/tool_exchange_stages.hpp"

#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace mtc = moveit::task_constructor;

namespace {
constexpr double DOCK_SPACING_METERS = 0.1524;
}

ToolExchangeStages::ToolExchangeStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config) {}

bool ToolExchangeStages::run(const nlohmann::json& step,
                             const nlohmann::json& poses,
                             rclcpp::Node::SharedPtr /*node_ptr*/)
{
  const std::string operation = step.value("operation", "load");
  const int dock_number = step.value("dock_number", 3);
  const auto& approach_entries = step.value("poses", std::vector<std::string>{});
  if (approach_entries.empty()) {
    RCLCPP_ERROR(node()->get_logger(), "Tool exchange step requires at least one approach pose");
    return false;
  }

  refreshPoses(poses);

  const double dock_offset_y = DOCK_SPACING_METERS * static_cast<double>(3 - dock_number);
  const std::string& arm_group = defaultArmGroupName();
  constexpr const char* ik_frame = "flange";

  std::string task_name;
  if (operation == "load") {
    task_name = "Load Tool Task";
  } else if (operation == "dock") {
    task_name = "Dock Tool Task";
  } else {
    task_name = "Tool Exchange Task";
  }

  auto task = createTaskTemplate(task_name, arm_group, ik_frame);

  auto sampling_planner = makePipelinePlanner();
  auto cartesian_planner = makeCartesianPlanner();

  const auto addNamedMoveStage = [&](const std::string& label, const std::string& pose_key) {
    const auto& pose = config().at("poses").at(pose_key);
    if (!pose.is_array() || pose.size() != BaseStages::defaultJointNames().size()) {
      throw std::runtime_error(pose_key + " must be an array of 6 numbers");
    }

    std::vector<double> joint_angles_deg;
    joint_angles_deg.reserve(pose.size());
    for (const auto& v : pose) {
      joint_angles_deg.push_back(v.get<double>());
    }

    auto stage = std::make_unique<mtc::stages::MoveTo>(label, sampling_planner);
    stage->setGroup(arm_group);
    stage->setGoal(jointsFromDegrees(joint_angles_deg));
    task.add(std::move(stage));
  };

  const auto addRelativeMoveStage = [&](const std::string& name,
                                        double distance,
                                        double x, double y, double z,
                                        const std::string& marker_ns) {
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

  const std::string& approach_pose = approach_entries.front();

  if (operation == "load") {
    addNamedMoveStage("move to load approach", approach_pose);
    addDockShiftStage(dock_offset_y);
    addRelativeMoveStage("attach_tool", 0.1, 1.0, 0.0, 0.0, "approach_object");
    addRelativeMoveStage("detach_holder", 0.15, 0.0, 0.0, -1.0, "approach_object");
    addRelativeMoveStage("move_up", 0.2, -1.0, 0.0, 0.0, "approach_object");
  } else if (operation == "dock") {
    addNamedMoveStage("move to dock approach", approach_pose);
    addDockShiftStage(dock_offset_y);
    addRelativeMoveStage("align_holder", 0.2, 1.0, 0.0, 0.0, "approach_object");
    addRelativeMoveStage("detach_tool", 0.15, 0.0, 0.0, 1.0, "approach_object");
    addRelativeMoveStage("dock connect", 0.1, -1.0, 0.0, 0.0, "approach_object");
  } else {
    RCLCPP_ERROR(node()->get_logger(), "Unknown tool exchange operation '%s'", operation.c_str());
    return false;
  }

  const bool success = loadPlanExecute(task);
  if (success) {
    RCLCPP_INFO(node()->get_logger(), "Tool exchange %s completed successfully", operation.c_str());
  } else {
    RCLCPP_ERROR(node()->get_logger(), "Tool exchange %s failed", operation.c_str());
  }
  return success;
}
