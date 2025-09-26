#include "mtc_pipeline/end_effector_stages.hpp"

#include <control_msgs/action/gripper_command.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/task_constructor/stages/current_state.h>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>
#include <string>

namespace mtc = moveit::task_constructor;

EndEffectorStages::EndEffectorStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config)
{
  loadEndEffectorConfig();
}

void EndEffectorStages::loadEndEffectorConfig()
{
  end_effector_config_["type"] = "hande";
  end_effector_config_["gripper_topic"] = "/gripper_action_controller/gripper_cmd";
  end_effector_config_["gripper_open_position"] = "0.025";
  end_effector_config_["gripper_close_position"] = "0.0";
  end_effector_config_["gripper_force"] = "100.0";
  end_effector_config_["vacuum_topic"] = "/vacuum_control";
  end_effector_config_["vacuum_pressure"] = "0.8";
}

std::string EndEffectorStages::getEndEffectorType()
{
  return end_effector_config_["type"];
}

std::string EndEffectorStages::getGripperActionTopic()
{
  return end_effector_config_["gripper_topic"];
}

std::string EndEffectorStages::getVacuumActionTopic()
{
  return end_effector_config_["vacuum_topic"];
}

bool EndEffectorStages::controlGripper(const std::string& action, double position, double force)
{
  const std::string end_effector_type = getEndEffectorType();

  if (end_effector_type != "hande") {
    RCLCPP_ERROR(node()->get_logger(), "Unknown end effector type: %s", end_effector_type.c_str());
    return false;
  }

  auto gripper_action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
    node(), getGripperActionTopic());

  if (!gripper_action_client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node()->get_logger(), "Gripper action server not available");
    return false;
  }

  double target_position = position;
  if (action == "open") {
    target_position = std::stod(end_effector_config_["gripper_open_position"]);
  } else if (action == "close") {
    target_position = std::stod(end_effector_config_["gripper_close_position"]);
  }

  const double target_force = (force > 0.0) ? force : std::stod(end_effector_config_["gripper_force"]);

  control_msgs::action::GripperCommand::Goal goal;
  goal.command.position = target_position;
  goal.command.max_effort = target_force;

  auto goal_handle_future = gripper_action_client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node()->get_logger(), "Failed to send gripper goal");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node()->get_logger(), "Gripper goal was rejected");
    return false;
  }

  auto result_future = gripper_action_client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node(), result_future, std::chrono::seconds(10))
      != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node()->get_logger(), "Gripper action timed out");
    return false;
  }

  auto wrapped_result = result_future.get();
  return wrapped_result.result ? wrapped_result.result->reached_goal : false;
}

bool EndEffectorStages::controlVacuum(const std::string& action,
                                      const std::string& end_effector_type,
                                      double /*pressure*/)
{
  if (end_effector_type != "epick") {
    RCLCPP_ERROR(node()->get_logger(), "Vacuum control not supported for end effector type: %s",
                 end_effector_type.c_str());
    return false;
  }

  auto gripper_action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
    node(), "/epick_gripper_action_controller/gripper_cmd");

  if (!gripper_action_client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node()->get_logger(), "EPick gripper action server not available");
    return false;
  }

  control_msgs::action::GripperCommand::Goal goal;
  if (action == "on" || action == "vacuum_on") {
    goal.command.position = 1.0;
  } else if (action == "off" || action == "vacuum_off") {
    goal.command.position = 0.0;
  } else {
    RCLCPP_ERROR(node()->get_logger(), "Unknown vacuum action: %s", action.c_str());
    return false;
  }
  goal.command.max_effort = 100.0;

  auto goal_handle_future = gripper_action_client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node()->get_logger(), "Failed to send EPick gripper goal");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node()->get_logger(), "EPick gripper goal was rejected");
    return false;
  }

  auto result_future = gripper_action_client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node(), result_future, std::chrono::seconds(10))
      != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node()->get_logger(), "EPick gripper action timed out");
    return false;
  }

  auto wrapped_result = result_future.get();
  return wrapped_result.result ? wrapped_result.result->reached_goal : false;
}

bool EndEffectorStages::controlCustom(const std::string& end_effector_type,
                                      const std::string& action,
                                      const nlohmann::json& params)
{
  (void)action;
  (void)params;
  RCLCPP_WARN(node()->get_logger(), "Custom end effector control not implemented for type: %s",
              end_effector_type.c_str());
  return false;
}

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

  const std::string end_effector_type = step.value("end_effector_type", getEndEffectorType());
  const std::string action = step.value("end_effector_action", "");
  RCLCPP_INFO(node()->get_logger(), "End effector control: type=%s, action=%s",
              end_effector_type.c_str(), action.c_str());

  auto interpolation_planner = makeJointInterpolationPlanner();

  auto execute_simple_stage = [&](const std::string& task_name,
                                  const std::string& group_name,
                                  const std::string& goal_state) -> bool {
    mtc::Task task;
    task.stages()->setName(task_name);
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));

    auto stage = std::make_unique<mtc::stages::MoveTo>("end_effector_control", interpolation_planner);
    stage->setGroup(group_name);
    stage->setGoal(goal_state);
    task.add(std::move(stage));

    return loadPlanExecute(task, 5, should_cancel);
  };

  if (end_effector_type == "hande" || end_effector_type == "gripper") {
    std::string goal_state;
    if (action == "open") {
      goal_state = "hande_open";
    } else if (action == "close") {
      goal_state = "hande_closed";
    } else {
      RCLCPP_ERROR(node()->get_logger(), "Unknown gripper action: %s", action.c_str());
      return false;
    }

    const bool success = execute_simple_stage("Gripper Control Task", "hande_gripper", goal_state);
    if (success) {
      RCLCPP_INFO(node()->get_logger(), "End effector control successful: %s %s",
                  end_effector_type.c_str(), action.c_str());
    }
    return success;
  }

  if (end_effector_type == "epick" || end_effector_type == "vacuum") {
    std::string goal_state;
    if (action == "vacuum_on" || action == "on") {
      goal_state = "vacuum_on";
    } else if (action == "vacuum_off" || action == "off") {
      goal_state = "vacuum_off";
    } else {
      RCLCPP_ERROR(node()->get_logger(), "Unknown EPick action: %s", action.c_str());
      return false;
    }

    const bool success = execute_simple_stage("EPick Gripper Control Task", "epick_gripper", goal_state);
    if (success) {
      RCLCPP_INFO(node()->get_logger(), "EPick gripper control successful: %s", action.c_str());
    }
    return success;
  }

  nlohmann::json params = step.value("params", nlohmann::json::object());
  return controlCustom(end_effector_type, action, params);
}
