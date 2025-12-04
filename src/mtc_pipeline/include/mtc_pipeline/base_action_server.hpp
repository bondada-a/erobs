// Template base class for MTC action servers.
// Handles goal lifecycle, threading, and concurrent execution prevention.
// Usage:
//   class MyServer : public BaseActionServer<MyAction, MyStages> { ... };
//   auto node = std::make_shared<MyServer>();
//   node->initialize_stages();  // Required: shared_from_this() not available in ctor
//   rclcpp::spin(node);

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <string>
#include <thread>

template<typename ActionType, typename StagesType>
class BaseActionServer : public rclcpp::Node
{
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;

    /// @brief Construct action server with node and action names
    BaseActionServer(const std::string& node_name, const std::string& action_name)
        : Node(node_name)
    {
        action_server_ = rclcpp_action::create_server<ActionType>(
            this,
            action_name,
            [this](const auto&, const auto&) {
                RCLCPP_INFO(this->get_logger(), "Received goal");
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            nullptr,  // Cancel not supported (can't safely abort mid-motion)
            [this](const auto& gh) { handle_accepted(gh); }
        );
        RCLCPP_INFO(this->get_logger(), "%s started", node_name.c_str());
    }

    /// @brief Initialize stages object after construction
    void initialize_stages()
    {
        stages_ = std::make_unique<StagesType>(this->shared_from_this());
    }

private:
    typename rclcpp_action::Server<ActionType>::SharedPtr action_server_;
    std::unique_ptr<StagesType> stages_;
    bool executing_{false};

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        if (executing_) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: server busy");
            auto result = std::make_shared<typename ActionType::Result>();
            result->success = false;
            result->error_message = "Server busy";
            goal_handle->abort(result);
            return;
        }
        executing_ = true;

        // Worker thread keeps main executor responsive for callbacks
        std::thread{[this, node_lifetime = shared_from_this(), goal_handle]() {
            execute(goal_handle);
            executing_ = false;
        }}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        auto result = std::make_shared<typename ActionType::Result>();

        if (!stages_) {
            RCLCPP_ERROR(this->get_logger(), "Stages not initialized");
            result->success = false;
            result->error_message = "Stages not initialized";
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Executing goal");

        try {
            result->success = stages_->run(*goal_handle->get_goal());
            if (!result->success) {
                result->error_message = "Execution failed";
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
            result->success = false;
            result->error_message = e.what();
        }

        if (!rclcpp::ok()) return;

        if (result->success) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } else {
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Goal failed: %s", result->error_message.c_str());
        }
    }
};
