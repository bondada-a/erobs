#include "mtc_pipeline/moveto_stages.hpp"
#include <moveit/task_constructor/stages/move_to.h>

namespace mtc = moveit::task_constructor;

MoveToStages::MoveToStages(const rclcpp::Node::SharedPtr& node)
  : BaseStages(node) {}


bool MoveToStages::run(const nlohmann::json& step, const nlohmann::json& poses) {
  const std::string target_type = step.at("target_type");
  const std::string planning_type = step.value("planning_type", "joint");

  auto task = createTaskTemplate("MoveTo Task");
  auto planner = (planning_type == "cartesian") ? makeCartesianPlanner() : makePipelinePlanner();

  // 1. NAMED STATE or POSE: Move to predefined SRDF state or joint configuration
  if (target_type == "named_state" || target_type == "pose") {
    const std::string target_name = step.at("target");
    const std::string label = planning_type == "cartesian"
      ? "move_to_cartesian_" + target_name
      : "move_to_" + target_name;

    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(defaultArmGroupName());

    if (target_type == "pose") {
      const auto& joint_pose_json = poses.at(target_name);
      if (!joint_pose_json.is_array() || joint_pose_json.size() != 6) {
        RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 joint angles", target_name.c_str());
        return false;
      }
      stage->setGoal(jointsFromDegrees(joint_pose_json.get<std::vector<double>>()));
    } else {
      stage->setGoal(target_name);  // named_state
    }

    task.add(std::move(stage));
  }

  // 3. RELATIVE: Move relative to current position (e.g., "forward", "up")
  else if (target_type == "relative") {
    const std::string direction = step.at("direction");
    const double distance = step.at("distance").get<double>();
    const std::string label = "move_" + direction + "_" + std::to_string(distance) + "m";
    task.add(createRelativeMoveStage(label, direction, distance, planner));
  }

  else {
    RCLCPP_ERROR(node()->get_logger(), "Unsupported target_type '%s'", target_type.c_str());
    return false;
  }

  return loadPlanExecute(task);
}
