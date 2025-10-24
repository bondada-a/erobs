#include "mtc_pipeline/end_effector_stages.hpp"

#include <moveit/task_constructor/stages/move_to.h>

EndEffectorStages::EndEffectorStages(const rclcpp::Node::SharedPtr& node)
  : BaseStages(node) {}

bool EndEffectorStages::run(const nlohmann::json& step, const nlohmann::json& poses)
{
  const std::string group_name = step.at("end_effector_type");
  const std::string goal_state = step.at("end_effector_action");

  // Create planner for this task
  auto interpolation_planner = make_joint_interpolation_planner();

  // Create MTC task
  const std::string task_name = goal_state;
  auto task = create_task_template(task_name, group_name);

  auto stage = std::make_unique<mtc::stages::MoveTo>(task_name, interpolation_planner);
  stage->setGroup(group_name);
  stage->setGoal(goal_state);
  task.add(std::move(stage));

  return load_plan_execute(task);
}
