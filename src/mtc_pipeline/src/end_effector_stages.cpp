#include "mtc_pipeline/end_effector_stages.hpp"

#include <moveit/task_constructor/stages/move_to.h>

EndEffectorStages::EndEffectorStages(const rclcpp::Node::SharedPtr& node)
  : BaseStages(node) {}

std::string EndEffectorStages::get_gripper_group_name(const std::string& end_effector_type)
{
  // Map end effector types to their MoveIt group names
  // epick and hande have movable joints, pipettor is static
  if (end_effector_type == "epick") {
    return "epick_gripper";
  } else if (end_effector_type == "hande") {
    return "hande_gripper";
  } else if (end_effector_type == "pipettor") {
    // Pipettor has no movable joints, no group defined
    RCLCPP_ERROR(node()->get_logger(),
                 "Pipettor is a static end effector with no movable joints. "
                 "End effector actions (open/close) are not supported for pipettor.");
    return "";
  } else {
    RCLCPP_ERROR(node()->get_logger(),
                 "Unknown end effector type: %s", end_effector_type.c_str());
    return "";
  }
}

std::string EndEffectorStages::get_goal_state_name(
    const std::string& end_effector_type,
    const std::string& action)
{
  // Map generic action names to SRDF group state names
  // Different grippers use different naming conventions in SRDF

  if (end_effector_type == "epick") {
    // EPick uses vacuum_on/vacuum_off directly in SRDF
    if (action == "vacuum_on" || action == "vacuum_off") {
      return action;
    } else {
      RCLCPP_ERROR(node()->get_logger(),
                   "Invalid action '%s' for epick. Valid actions: vacuum_on, vacuum_off",
                   action.c_str());
      return "";
    }
  } else if (end_effector_type == "hande") {
    // Hande uses hande_open/hande_closed in SRDF (not just open/close)
    if (action == "open") {
      return "hande_open";
    } else if (action == "close") {
      return "hande_closed";
    } else {
      RCLCPP_ERROR(node()->get_logger(),
                   "Invalid action '%s' for hande. Valid actions: open, close",
                   action.c_str());
      return "";
    }
  } else {
    RCLCPP_ERROR(node()->get_logger(),
                 "Cannot map goal state for unknown end effector type: %s",
                 end_effector_type.c_str());
    return "";
  }
}

bool EndEffectorStages::run(const nlohmann::json& step, const nlohmann::json& poses)
{
  const std::string end_effector_type = step.at("end_effector_type");
  const std::string action = step.at("end_effector_action");

  // Map end effector type to MoveIt group name
  const std::string group_name = get_gripper_group_name(end_effector_type);

  if (group_name.empty()) {
    RCLCPP_ERROR(node()->get_logger(),
                 "Failed to get group name for end effector type: %s",
                 end_effector_type.c_str());
    return false;
  }

  // Map generic action to SRDF goal state name
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
