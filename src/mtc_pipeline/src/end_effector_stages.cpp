#include "mtc_pipeline/end_effector_stages.hpp"
#include "../../end_effectors/gripper_config.hpp"

#include <moveit/task_constructor/stages/move_to.h>

EndEffectorStages::EndEffectorStages(const rclcpp::Node::SharedPtr& node)
  : BaseStages(node) {}

std::string EndEffectorStages::get_gripper_group_name(const std::string& end_effector_type)
{
  // Use shared gripper configuration
  auto config = gripper_config::get_gripper_config(end_effector_type);

  if (config.group.empty()) {
    RCLCPP_ERROR(node()->get_logger(),
                 "Pipettor is a static end effector with no movable joints. "
                 "End effector state changes are not supported for pipettor.");
  }

  return config.group;
}

std::string EndEffectorStages::get_goal_state_name(
    const std::string& end_effector_type,
    const std::string& action)
{
  // Action is expected to be the SRDF state name directly
  // Examples: "hande_open", "hande_closed", "vacuum_on", "vacuum_off"

  // Validate that we have a known end effector type
  auto config = gripper_config::get_gripper_config(end_effector_type);

  if (config.group.empty()) {
    RCLCPP_ERROR(node()->get_logger(),
                 "Unknown end effector type: %s",
                 end_effector_type.c_str());
    return "";
  }

  // Return the action as the SRDF state name directly
  return action;
}

bool EndEffectorStages::run(const mtc_pipeline::action::EndEffectorAction::Goal& goal)
{
  const std::string& end_effector_type = goal.end_effector_type;
  const std::string& action = goal.end_effector_action;

  // Map end effector type to MoveIt group name
  const std::string group_name = get_gripper_group_name(end_effector_type);

  if (group_name.empty()) {
    RCLCPP_ERROR(node()->get_logger(),
                 "Failed to get group name for end effector type: %s",
                 end_effector_type.c_str());
    return false;
  }

  // Get SRDF goal state name (action should already be SRDF state name)
  const std::string goal_state = get_goal_state_name(end_effector_type, action);

  if (goal_state.empty()) {
    RCLCPP_ERROR(node()->get_logger(),
                 "Failed to get goal state for end effector '%s' with action '%s'",
                 end_effector_type.c_str(), action.c_str());
    return false;
  }

  RCLCPP_INFO(node()->get_logger(),
              "End effector action: type='%s', action='%s', group='%s', goal_state='%s'",
              end_effector_type.c_str(), action.c_str(), group_name.c_str(), goal_state.c_str());

  // Create planner for this task
  auto interpolation_planner = make_joint_interpolation_planner();

  // Create MTC task
  const std::string task_name = action;
  auto task = create_task_template(task_name, group_name);

  auto stage = std::make_unique<mtc::stages::MoveTo>(task_name, interpolation_planner);
  stage->setGroup(group_name);
  stage->setGoal(goal_state);
  task.add(std::move(stage));

  return load_plan_execute(task);
}
