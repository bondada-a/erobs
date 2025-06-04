#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <cms_beamtime_interfaces/srv/gripper_control_msg.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace gripper_service_trajectory_controller
{
class GripperServiceTrajectoryController : public controller_interface::ControllerInterface
{
public:
  GripperServiceTrajectoryController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
  void traj_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory> traj_buffer_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Client<cms_beamtime_interfaces::srv::GripperControlMsg>::SharedPtr gripper_client_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};
}  // namespace gripper_service_trajectory_controller
