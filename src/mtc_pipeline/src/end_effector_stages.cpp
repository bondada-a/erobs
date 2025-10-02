#include "mtc_pipeline/end_effector_stages.hpp"

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>

namespace mtc = moveit::task_constructor;

EndEffectorStages::EndEffectorStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config)
{
  initializeGripperConfigs();
}

void EndEffectorStages::initializeGripperConfigs()
{
  // Initialize gripper configurations based on SRDF definitions
  // This matches the actual states defined in the SRDF files

  // Hande gripper configuration
  gripper_configs_["hande"] = {
    "hande_gripper",
    {
      {"open", "hande_open"},
      {"close", "hande_closed"}
    }
  };

  // Epick vacuum gripper configuration
  gripper_configs_["epick"] = {
    "epick_gripper",
    {
      {"on", "vacuum_on"},
      {"off", "vacuum_off"}
    }
  };

  // TODO: To add a new end effector:
  // 1. Define group states in your SRDF file
  // 2. Add configuration here:
  // gripper_configs_["your_gripper"] = {
  //   "your_gripper_group",
  //   {
  //     {"action1", "state_name1"},
  //     {"action2", "state_name2"}
  //   }
  // };
}

bool EndEffectorStages::run(const nlohmann::json& step, const nlohmann::json& poses)
{
  refreshPoses(poses);

  // Validate required fields
  if (!step.contains("end_effector_action") || step["end_effector_action"].empty()) {
    RCLCPP_ERROR(node()->get_logger(), "Missing or empty required field: end_effector_action");
    return false;
  }

  const std::string end_effector_type = step.value("end_effector_type", "hande");
  const std::string action = step["end_effector_action"];

  RCLCPP_DEBUG(node()->get_logger(), "End effector control: type=%s, action=%s",
               end_effector_type.c_str(), action.c_str());

  // Look up gripper configuration
  auto gripper_it = gripper_configs_.find(end_effector_type);
  if (gripper_it == gripper_configs_.end()) {
    RCLCPP_ERROR(node()->get_logger(),
                 "Unknown end effector type: '%s'. Available types: hande, epick",
                 end_effector_type.c_str());
    return false;
  }

  const GripperConfig& config = gripper_it->second;

  // Look up action in the configuration
  auto action_it = config.action_to_state.find(action);
  if (action_it == config.action_to_state.end()) {
    // Build list of valid actions for error message
    std::string valid_actions;
    for (const auto& [act, _] : config.action_to_state) {
      if (!valid_actions.empty()) valid_actions += ", ";
      valid_actions += act;
    }

    RCLCPP_ERROR(node()->get_logger(),
                 "Unknown action '%s' for end effector '%s'. Valid actions: [%s]",
                 action.c_str(), end_effector_type.c_str(), valid_actions.c_str());
    return false;
  }

  // Create planner for this task
  auto interpolation_planner = makeJointInterpolationPlanner();

  // Create MTC task
  const std::string task_name = end_effector_type + " " + action;
  const std::string& goal_state = action_it->second;

  mtc::Task task;
  task.stages()->setName(task_name);
  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  auto stage = std::make_unique<mtc::stages::MoveTo>(goal_state, interpolation_planner);
  stage->setGroup(config.group_name);
  stage->setGoal(goal_state);
  task.add(std::move(stage));

  const bool success = loadPlanExecute(task);
  if (success) {
    RCLCPP_DEBUG(node()->get_logger(), "End effector control successful: %s", task_name.c_str());
  } else {
    RCLCPP_ERROR(node()->get_logger(), "End effector control failed: %s", task_name.c_str());
  }

  return success;
}
