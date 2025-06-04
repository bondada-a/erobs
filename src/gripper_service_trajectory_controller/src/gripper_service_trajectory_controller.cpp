#include "gripper_service_trajectory_controller/gripper_service_trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace gripper_service_trajectory_controller
{
GripperServiceTrajectoryController::GripperServiceTrajectoryController() : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration GripperServiceTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back("joint_finger/position");
  return config;
}

controller_interface::InterfaceConfiguration GripperServiceTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back("joint_finger/position");
  return config;
}

controller_interface::CallbackReturn GripperServiceTrajectoryController::on_init()
{
  auto node = get_node();
  gripper_client_ = node->create_client<cms_beamtime_interfaces::srv::GripperControlMsg>("gripper_service");
  joint_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
  traj_sub_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", rclcpp::QoS(1),
    std::bind(&GripperServiceTrajectoryController::traj_callback, this, std::placeholders::_1));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperServiceTrajectoryController::on_activate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperServiceTrajectoryController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void GripperServiceTrajectoryController::traj_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  traj_buffer_.writeFromNonRT(*msg);
}

controller_interface::return_type GripperServiceTrajectoryController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  auto traj = traj_buffer_.readFromRT();
  if (!traj)
  {
    return controller_interface::return_type::OK;
  }

  if (traj->points.empty())
  {
    return controller_interface::return_type::OK;
  }

  double target = 0.0;
  const auto & point = traj->points.back();
  if (!point.positions.empty())
  {
    target = point.positions[0];
  }

  std::string command;
  int32_t grip = 0;
  if (target <= 0.005)
  {
    command = "OPEN";
    grip = 0;
  }
  else if (target >= 0.02)
  {
    command = "CLOSE";
    grip = 100;
  }
  else
  {
    command = "PARTIAL";
    grip = static_cast<int32_t>(std::round(target / 0.025 * 100.0));
  }

  auto request = std::make_shared<cms_beamtime_interfaces::srv::GripperControlMsg::Request>();
  request->command = command;
  request->grip = grip;
  if (gripper_client_->service_is_ready())
  {
    gripper_client_->async_send_request(request);
  }

  sensor_msgs::msg::JointState js;
  js.header.stamp = get_node()->now();
  js.name = {"joint_finger"};
  js.position = {target};
  joint_state_pub_->publish(js);

  traj_buffer_.reset();
  return controller_interface::return_type::OK;
}

}  // namespace gripper_service_trajectory_controller

PLUGINLIB_EXPORT_CLASS(gripper_service_trajectory_controller::GripperServiceTrajectoryController, controller_interface::ControllerInterface)
