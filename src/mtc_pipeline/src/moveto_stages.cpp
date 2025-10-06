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

  // 1. NAMED STATE: Move to predefined SRDF state (e.g., "moveit_home")
  if (target_type == "named_state") {
    const std::string named_state = step.at("target");
    auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_" + named_state, planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(defaultArmGroupName());
    stage->setGoal(named_state);
    task.add(std::move(stage));
  }

  // 2. POSE: Move to joint configuration from JSON config
  else if (target_type == "pose") {
    const std::string pose_key = step.at("target");
    const auto& joint_pose_json = poses.at(pose_key);

    if (!joint_pose_json.is_array() || joint_pose_json.size() != 6) {
      RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 joint angles", pose_key.c_str());
      return false;
    }

    const auto joint_angles_deg = joint_pose_json.get<std::vector<double>>();
    const std::string label = planning_type == "cartesian" ? "move_to_cartesian_" + pose_key : "move_to_" + pose_key;

    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(defaultArmGroupName());
    stage->setGoal(jointsFromDegrees(joint_angles_deg));
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
