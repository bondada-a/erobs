#pragma once

#include <moveit/task_constructor/stage.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <pipette_driver/action/pipettor_operation.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace mtc = moveit::task_constructor;

/**
 * Custom MTC Stage for pipettor hardware operations (SUCK, EXPEL, EJECT_TIP, SET_LED)
 *
 * This stage appears in RViz's Motion Planning Tasks panel with descriptive names
 * like "SUCK 50%" or "EXPEL 80%". It doesn't perform motion planning—it just
 * calls the pipettor hardware action when executed.
 *
 * The stage propagates the interface state unchanged (both forward and backward)
 * since pipettor operations don't modify the robot's pose or planning scene.
 */
class PipettorOperationStage : public mtc::PropagatingEitherWay {
public:
    PipettorOperationStage(const std::string& stage_name,
                           const rclcpp::Node::SharedPtr& node);

    // Configure the pipettor operation to execute
    void setOperation(const std::string& operation);
    void setVolumePct(double volume_pct);
    void setLedColor(const std_msgs::msg::ColorRGBA& led_color);

protected:
    // MTC stage interface: compute forward solutions
    void computeForward(const mtc::InterfaceState& from) override;

    // MTC stage interface: compute backward solutions
    void computeBackward(const mtc::InterfaceState& to) override;

private:
    // Execute the pipettor action and return success/failure
    bool execute_pipettor_action();

    // Propagate state without modification (helper for both directions)
    void propagate_state(const mtc::InterfaceState& state, bool forward);

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<pipette_driver::action::PipettorOperation>::SharedPtr action_client_;

    // Operation parameters
    std::string operation_;
    double volume_pct_;
    std_msgs::msg::ColorRGBA led_color_;
};
