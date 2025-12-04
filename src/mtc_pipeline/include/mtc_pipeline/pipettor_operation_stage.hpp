// Custom MTC stage for pipettor hardware operations (SUCK, EXPEL, SET_LED).
// Propagates state unchanged since pipettor ops don't modify robot pose.

#pragma once

#include <moveit/task_constructor/stage.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <pipette_driver/action/pipettor_operation.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace mtc = moveit::task_constructor;

class PipettorOperationStage : public mtc::PropagatingEitherWay {
public:
    /// @brief Construct PipettorOperation stage with name and ROS 2 node
    PipettorOperationStage(const std::string& name, const rclcpp::Node::SharedPtr& node);

    /// @brief Set pipettor operation type
    void setOperation(const std::string& operation);

    /// @brief Set volume percentage for operation
    void setVolumePct(double volume_pct);

    /// @brief Set LED color for operation
    void setLedColor(const std_msgs::msg::ColorRGBA& color);

protected:
    /// @brief Compute forward propagation for MTC
    void computeForward(const mtc::InterfaceState& from) override;

    /// @brief Compute backward propagation for MTC
    void computeBackward(const mtc::InterfaceState& to) override;

private:
    /// @brief Execute pipettor action via action client
    bool execute_pipettor_action();

    /// @brief Propagate state unchanged through this stage
    void propagate_state(const mtc::InterfaceState& state, bool forward);

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<pipette_driver::action::PipettorOperation>::SharedPtr action_client_;
    std::string operation_;
    double volume_pct_ = 0.0;
    std_msgs::msg::ColorRGBA led_color_;
};
