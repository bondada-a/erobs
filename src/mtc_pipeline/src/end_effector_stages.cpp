#include "mtc_pipeline/end_effector_stages.hpp"

#include <moveit/task_constructor/stages/move_to.h>

namespace mtc = moveit::task_constructor;

EndEffectorStages::EndEffectorStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config) {}

bool EndEffectorStages::run(const nlohmann::json& step, const nlohmann::json& poses)
{
  const std::string group_name = step.at("end_effector_type");
  const std::string goal_state = step.at("end_effector_action");

  // Create planner for this task
  auto interpolation_planner = makeJointInterpolationPlanner();

  // Create MTC task
  const std::string task_name = group_name + " " + goal_state;
  auto task = createTaskTemplate(task_name, group_name);

  auto stage = std::make_unique<mtc::stages::MoveTo>(task_name, interpolation_planner);
  stage->setGroup(group_name);
  stage->setGoal(goal_state);
  task.add(std::move(stage));

  return loadPlanExecute(task);
}
