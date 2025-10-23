#include "mtc_pipeline/pipettor_operation_stage.hpp"
#include <moveit/task_constructor/storage.h>
#include <chrono>

using namespace std::chrono_literals;

PipettorOperationStage::PipettorOperationStage(const std::string& stage_name,
                                               const rclcpp::Node::SharedPtr& node)
  : PropagatingEitherWay(stage_name)
  , node_(node)
  , volume_pct_(0.0)
{
    // Initialize action client
    action_client_ = rclcpp_action::create_client<pipette_driver::action::PipettorOperation>(
        node_, "pipettor_operation");

    // Default LED color (off)
    led_color_.r = led_color_.g = led_color_.b = 0.0;
    led_color_.a = 1.0;
}

void PipettorOperationStage::setOperation(const std::string& operation) {
    operation_ = operation;
}

void PipettorOperationStage::setVolumePct(double volume_pct) {
    volume_pct_ = volume_pct;
}

void PipettorOperationStage::setLedColor(const std_msgs::msg::ColorRGBA& led_color) {
    led_color_ = led_color;
}

void PipettorOperationStage::computeForward(const mtc::InterfaceState& from) {
    propagate_state(from, true);
}

void PipettorOperationStage::computeBackward(const mtc::InterfaceState& to) {
    propagate_state(to, false);
}

void PipettorOperationStage::propagate_state(const mtc::InterfaceState& state, bool forward) {
    // Execute the pipettor action
    RCLCPP_INFO(node_->get_logger(), "Executing stage: %s", name().c_str());

    bool success = execute_pipettor_action();

    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Pipettor stage '%s' failed", name().c_str());
        return;  // Don't send a solution on failure
    }

    // Create a new SubTrajectory (required by MTC interface)
    mtc::SubTrajectory trajectory;
    trajectory.setCost(0.0);  // No cost for hardware operations
    trajectory.setComment(name());

    // Send the solution
    // For PropagatingEitherWay, we pass the input state unchanged
    // (pipettor operations don't modify robot pose or planning scene)
    if (forward) {
        mtc::InterfaceState state_copy(state);
        sendForward(state, std::move(state_copy), std::move(trajectory));
    } else {
        mtc::InterfaceState state_copy(state);
        sendBackward(std::move(state_copy), state, std::move(trajectory));
    }

    RCLCPP_INFO(node_->get_logger(), "Pipettor stage '%s' succeeded", name().c_str());
}

bool PipettorOperationStage::execute_pipettor_action() {
    // Wait for action server with timeout
    if (!action_client_->wait_for_action_server(5s)) {
        RCLCPP_ERROR(node_->get_logger(), "Pipettor action server not available");
        return false;
    }

    // Create goal
    auto goal = pipette_driver::action::PipettorOperation::Goal();
    goal.operation = operation_;
    goal.volume_pct = volume_pct_;
    goal.led_color = led_color_;

    // Send goal synchronously
    RCLCPP_INFO(node_->get_logger(), "Sending pipettor operation: %s (%.0f%%)",
                operation_.c_str(), volume_pct_ * 100.0);
    auto future = action_client_->async_send_goal(goal);

    // Wait for goal acceptance
    if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Pipettor goal send timeout");
        return false;
    }

    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Pipettor goal rejected");
        return false;
    }

    // Wait for result
    auto result_future = action_client_->async_get_result(goal_handle);
    if (result_future.wait_for(60s) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Pipettor operation timeout");
        action_client_->async_cancel_goal(goal_handle);
        return false;
    }

    // Check result
    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(node_->get_logger(), "Pipettor action failed with code: %d",
                     static_cast<int>(result.code));
        return false;
    }

    if (!result.result->success) {
        RCLCPP_ERROR(node_->get_logger(), "Pipettor operation failed: %s",
                     result.result->message.c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Pipettor operation succeeded: %s",
                result.result->message.c_str());
    return true;
}
