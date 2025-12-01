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
    PipettorOperationStage(const std::string& name, const rclcpp::Node::SharedPtr& node);

    // Configuration setters
    void setOperation(const std::string& operation);
    void setVolumePct(double volume_pct);
    void setLedColor(const std_msgs::msg::ColorRGBA& color);

protected:
    // MTC interface overrides
    void computeForward(const mtc::InterfaceState& from) override;
    void computeBackward(const mtc::InterfaceState& to) override;

private:
    bool execute_pipettor_action();
    void propagate_state(const mtc::InterfaceState& state, bool forward);

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<pipette_driver::action::PipettorOperation>::SharedPtr action_client_;
    std::string operation_;
    double volume_pct_ = 0.0;
    std_msgs::msg::ColorRGBA led_color_;
};
