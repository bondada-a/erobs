/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <cms_beamtime/inner_state_machine.hpp>

using namespace std::chrono_literals;

InnerStateMachine::InnerStateMachine(
  const rclcpp::Node::SharedPtr node, const rclcpp::Node::SharedPtr gripper_node)
: node_(node), gripper_node_(gripper_node)
{
  internal_state_enum_ = Internal_State::RESTING;

  // // Create gripper client
  // gripper_client_ =
  //   gripper_node_->create_client<cms_beamtime_interfaces::srv::GripperControlMsg>(
  //   "gripper_service");

    // Create Hand-E GripperCommand action client
  gripper_action_client_ =
    rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      gripper_node_, "/gripper_action_controller/gripper_cmd");
}

moveit::core::MoveItErrorCode InnerStateMachine::move_robot(
  moveit::planning_interface::MoveGroupInterface & mgi, std::vector<double> joint_goal)
{
  moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;
  joint_goal_ = joint_goal;

  switch (internal_state_enum_) {
    case Internal_State::RESTING:
    case Internal_State::CLEANUP: {
        mgi.setJointValueTarget(joint_goal_);
        // Create a plan to that target pose
        auto const [planing_success, plan] = [&mgi] {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto const ok = static_cast<bool>(mgi.plan(plan));
            return std::make_pair(ok, plan);
          }();
        if (planing_success) {
          // Change inner state to Moving if the robot is ready to move and not on clean up
          if (internal_state_enum_ == Internal_State::RESTING) {
            set_internal_state(Internal_State::MOVING);
          }
          auto exec_results = mgi.execute(plan);
          return_error_code = exec_results;
          if (exec_results == moveit::core::MoveItErrorCode::SUCCESS) {
            set_internal_state(Internal_State::RESTING);
          }
        } else {
          return_error_code = moveit::core::MoveItErrorCode::FAILURE;
        }
      }
      break;

    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Robot's current internal state is %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return_error_code = moveit::core::MoveItErrorCode::FAILURE;
      break;
  }

  return return_error_code;
}

moveit::core::MoveItErrorCode InnerStateMachine::move_robot_cartesian(
  moveit::planning_interface::MoveGroupInterface & mgi,
  std::vector<geometry_msgs::msg::Pose> target_pose)
{
  moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;

  switch (internal_state_enum_) {
    case Internal_State::RESTING:
    case Internal_State::CLEANUP: {
        mgi.setStartStateToCurrentState();
        // Create a plan to that target pose
        auto const [planing_success, plan] = [&mgi, target_pose] {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            // path_achieved_fraction, between 0.0 and 1.0, indicating the fraction of the path
            // achieved as described by the waypoints. Return -1.0 in case of error.
            double path_achieved_fraction = mgi.computeCartesianPath(
              target_pose, 0.01, 0.0,
              plan.trajectory_);
            return std::make_pair(path_achieved_fraction, plan);
          }();
        if (1.0 - planing_success < 0.000001) {
          // Change inner state to Moving if the robot is ready to move and not on clean up
          if (internal_state_enum_ == Internal_State::RESTING) {
            set_internal_state(Internal_State::MOVING);
          }
          auto exec_results = mgi.execute(plan);
          return_error_code = exec_results;
          if (exec_results == moveit::core::MoveItErrorCode::SUCCESS) {
            set_internal_state(Internal_State::RESTING);
          }
        } else {
          return_error_code = moveit::core::MoveItErrorCode::FAILURE;
        }
      }
      break;

    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Robot's current internal state is %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return_error_code = moveit::core::MoveItErrorCode::FAILURE;
      break;
  }

  return return_error_code;
}

// moveit::core::MoveItErrorCode InnerStateMachine::close_gripper()
// {
//   moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;
//   switch (internal_state_enum_) {
//     case Internal_State::RESTING:
//     case Internal_State::CLEANUP: {
//         auto request = std::make_shared<cms_beamtime_interfaces::srv::GripperControlMsg::Request>();
//         request->command = "CLOSE";
//         request->grip = 100;

//         if (!gripper_client_->wait_for_service(10s)) {
//           return_error_code = moveit::core::MoveItErrorCode::FAILURE;
//           break;
//         } else {
//           set_internal_state(Internal_State::MOVING);
//           auto result = gripper_client_->async_send_request(request);
//           // if (rclcpp::spin_until_future_complete(gripper_node_, result) ==
//           //   rclcpp::FutureReturnCode::SUCCESS)
//           //   set_internal_state(Internal_State::RESTING);

//           // {
//           //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper open: %d", result.get()->results);
//           //   std::this_thread::sleep_for(3s);
//           //   return_error_code = moveit::core::MoveItErrorCode::SUCCESS;
//           //   set_internal_state(Internal_State::RESTING);
//           // } else {
//           //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
//           //   return_error_code = moveit::core::MoveItErrorCode::FAILURE;
//           // }
//           if (rclcpp::spin_until_future_complete(gripper_node_, result) ==
//               rclcpp::FutureReturnCode::SUCCESS)
//           {
//             set_internal_state(Internal_State::RESTING);
//             RCLCPP_INFO(
//               rclcpp::get_logger("rclcpp"),
//               "Gripper result: %d", result.get()->results);
//             std::this_thread::sleep_for(3s);
//             return_error_code = moveit::core::MoveItErrorCode::SUCCESS;
//           }
//           else
//           {
//             RCLCPP_ERROR(
//               rclcpp::get_logger("rclcpp"),
//               "Failed to call gripper service");
//             return_error_code = moveit::core::MoveItErrorCode::FAILURE;
//           }

//         }
//       }
//       break;

//     default:
//       RCLCPP_ERROR(
//         node_->get_logger(), "Robot's current internal state is %s ",
//         internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
//       return_error_code = moveit::core::MoveItErrorCode::FAILURE;
//       break;
//   }
//   return return_error_code;
// }
moveit::core::MoveItErrorCode InnerStateMachine::close_gripper()
{
  using GripperCmd = control_msgs::action::GripperCommand;

  // only valid from RESTING or CLEANUP
  if (internal_state_enum_ != Internal_State::RESTING &&
      internal_state_enum_ != Internal_State::CLEANUP)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Invalid internal state for close_gripper()");
    return moveit::core::MoveItErrorCode::FAILURE;
  }

  // Build the goal: fully closed = position 0.0
  GripperCmd::Goal goal_msg;
  goal_msg.command.position   = 0.0;
  goal_msg.command.max_effort = 100.0;

  // Wait for the Hand-E action server
  if (!gripper_action_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node_->get_logger(),
      "GripperCommand action server not available");
    return moveit::core::MoveItErrorCode::FAILURE;
  }

  // Send the goal
  set_internal_state(Internal_State::MOVING);
  auto send_opts = typename rclcpp_action::Client<GripperCmd>::SendGoalOptions();
  auto gh_future = gripper_action_client_->async_send_goal(goal_msg, send_opts);

  if (rclcpp::spin_until_future_complete(gripper_node_, gh_future)
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Failed to send gripper close goal");
    return moveit::core::MoveItErrorCode::FAILURE;
  }

  auto goal_handle = gh_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(),
      "Gripper close goal was rejected");
    return moveit::core::MoveItErrorCode::FAILURE;
  }

  // Wait for result
  auto result_future = gripper_action_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(gripper_node_, result_future)
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Failed to get gripper close result");
    return moveit::core::MoveItErrorCode::FAILURE;
  }

  // Completed
  set_internal_state(Internal_State::RESTING);
  return moveit::core::MoveItErrorCode::SUCCESS;
}

// moveit::core::MoveItErrorCode InnerStateMachine::open_gripper()
// {
//   moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;

//   switch (internal_state_enum_) {
//     case Internal_State::RESTING:
//     case Internal_State::CLEANUP: {
//         auto request = std::make_shared<cms_beamtime_interfaces::srv::GripperControlMsg::Request>();
//         request->command = "OPEN";
//         request->grip = 100;

//         if (!gripper_client_->wait_for_service(10s)) {
//           return_error_code = moveit::core::MoveItErrorCode::FAILURE;
//           break;
//         } else {
//           set_internal_state(Internal_State::MOVING);
//           auto result = gripper_client_->async_send_request(request);
//           if (rclcpp::spin_until_future_complete(gripper_node_, result) ==
//             rclcpp::FutureReturnCode::SUCCESS)
//           {
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper open: %d", result.get()->results);
//             std::this_thread::sleep_for(3s);
//             return_error_code = moveit::core::MoveItErrorCode::SUCCESS;
//           } else {
//             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
//             return_error_code = moveit::core::MoveItErrorCode::FAILURE;
//           }
//         }
//       }
//       break;

//     default:
//       RCLCPP_ERROR(
//         node_->get_logger(), "Robot's current internal state is %s ",
//         internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
//       return_error_code = moveit::core::MoveItErrorCode::FAILURE;
//       break;
//   }
//   return return_error_code;
// }
moveit::core::MoveItErrorCode InnerStateMachine::open_gripper()
{
  using GripperCmd = control_msgs::action::GripperCommand;

  if (internal_state_enum_ != Internal_State::RESTING &&
      internal_state_enum_ != Internal_State::CLEANUP)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Invalid internal state for open_gripper()");
    return moveit::core::MoveItErrorCode::FAILURE;
  }

  // Build the goal: fully open = max width (e.g. 0.085 m)
  GripperCmd::Goal goal_msg;
  goal_msg.command.position   = 0.025;  
  goal_msg.command.max_effort = 100.0;

  if (!gripper_action_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node_->get_logger(),
      "GripperCommand action server not available");
    return moveit::core::MoveItErrorCode::FAILURE;
  }

  set_internal_state(Internal_State::MOVING);
  auto send_opts = typename rclcpp_action::Client<GripperCmd>::SendGoalOptions();
  auto gh_future = gripper_action_client_->async_send_goal(goal_msg, send_opts);

  if (rclcpp::spin_until_future_complete(gripper_node_, gh_future)
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Failed to send gripper open goal");
    return moveit::core::MoveItErrorCode::FAILURE;
  }

  auto goal_handle = gh_future.get();
  auto result_future = gripper_action_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(gripper_node_, result_future)
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Failed to get gripper open result");
    return moveit::core::MoveItErrorCode::FAILURE;
  }

  set_internal_state(Internal_State::RESTING);
  return moveit::core::MoveItErrorCode::SUCCESS;
}

void InnerStateMachine::pause(moveit::planning_interface::MoveGroupInterface & mgi)
{
  switch (internal_state_enum_) {
    case Internal_State::RESTING:
      RCLCPP_INFO(
        node_->get_logger(), "Paused while at internal state %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      set_internal_state(Internal_State::PAUSED);
      break;

    case Internal_State::MOVING:
      mgi.stop();
      RCLCPP_INFO(
        node_->get_logger(), "Paused while at internal state %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      set_internal_state(Internal_State::PAUSED);
      break;
    default:
      break;
  }
}

void InnerStateMachine::abort(moveit::planning_interface::MoveGroupInterface & mgi)
{
  mgi.stop();
  RCLCPP_INFO(
    node_->get_logger(), "Stopped while at internal state %s ",
    internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
  set_internal_state(Internal_State::ABORT);
}

void InnerStateMachine::halt(moveit::planning_interface::MoveGroupInterface & mgi)
{
  mgi.stop();
  RCLCPP_INFO(
    node_->get_logger(), "Halted while at internal state %s ",
    internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
  set_internal_state(Internal_State::HALT);
}

void InnerStateMachine::rewind()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    set_internal_state(Internal_State::RESTING);
  }
}

void InnerStateMachine::set_internal_state(Internal_State state)
{
  RCLCPP_INFO(
    node_->get_logger(), "Internal state changed from %s to %s ",
    internal_state_names[static_cast<int>(internal_state_enum_)].c_str(),
    internal_state_names[static_cast<int>(state)].c_str());
  internal_state_enum_ = state;
}

Internal_State InnerStateMachine::get_internal_state()
{
  return internal_state_enum_;
}