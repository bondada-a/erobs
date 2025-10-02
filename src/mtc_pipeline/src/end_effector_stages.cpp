#include "mtc_pipeline/end_effector_stages.hpp"

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>

namespace mtc = moveit::task_constructor;

EndEffectorStages::EndEffectorStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config)
{}

bool EndEffectorStages::run(const nlohmann::json& step,
                            const nlohmann::json& poses,
                            rclcpp::Node::SharedPtr node_ptr)
{
  return run(step, poses, node_ptr, nullptr);
}

bool EndEffectorStages::run(const nlohmann::json& step,
                            const nlohmann::json& poses,
                            rclcpp::Node::SharedPtr /*node_ptr*/,
                            std::function<bool()> should_cancel)
{
  refreshPoses(poses);

  const std::string end_effector_type = step.value("end_effector_type", "hande");
  const std::string action = step.value("end_effector_action", "");

  RCLCPP_INFO(node()->get_logger(), "End effector control: type=%s, action=%s",
              end_effector_type.c_str(), action.c_str());

  // Map (type, action) -> (group_name, goal_state)
  std::string group_name;
  std::string goal_state;

  if (end_effector_type == "hande" || end_effector_type == "gripper") {
    group_name = "hande_gripper";
    if (action == "open") {
      goal_state = "hande_open";
    } else if (action == "close") {
      goal_state = "hande_closed";
    } else {
      RCLCPP_ERROR(node()->get_logger(), "Unknown gripper action: %s", action.c_str());
      return false;
    }
  } else if (end_effector_type == "epick" || end_effector_type == "vacuum") {
    group_name = "epick_gripper";
    if (action == "vacuum_on" || action == "on") {
      goal_state = "vacuum_on";
    } else if (action == "vacuum_off" || action == "off") {
      goal_state = "vacuum_off";
    } else {
      RCLCPP_ERROR(node()->get_logger(), "Unknown EPick action: %s", action.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(node()->get_logger(), "Unknown end effector type: %s", end_effector_type.c_str());
    return false;
  }

  // Execute MTC task
  auto interpolation_planner = makeJointInterpolationPlanner();

  const std::string task_name = end_effector_type + " " + action;
  const std::string stage_name = goal_state;

  mtc::Task task;
  task.stages()->setName(task_name);
  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  auto stage = std::make_unique<mtc::stages::MoveTo>(stage_name, interpolation_planner);
  stage->setGroup(group_name);
  stage->setGoal(goal_state);
  task.add(std::move(stage));

  const bool success = loadPlanExecute(task, 5, should_cancel);
  if (success) {
    RCLCPP_INFO(node()->get_logger(), "End effector control successful: %s", task_name.c_str());
  }
  return success;
}
