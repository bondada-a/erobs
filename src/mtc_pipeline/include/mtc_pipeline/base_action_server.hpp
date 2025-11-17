#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <thread>

template<typename ActionType, typename StagesType>
class BaseActionServer : public rclcpp::Node
{
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;

    BaseActionServer(const std::string& node_name, const std::string& action_name)
        : Node(node_name)
    {
        this->action_server_ = rclcpp_action::create_server<ActionType>(
            this,
            action_name,
            [this](const auto&, const auto&) {
                RCLCPP_INFO(this->get_logger(), "Received goal");
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            // Cancellation disabled - actions cannot be safely aborted mid-execution yet (TODO)
            nullptr,
            [this](const auto& goal_handle) {
                handle_accepted(goal_handle);
            }
        );

        RCLCPP_INFO(this->get_logger(), "%s Action Server started", node_name.c_str());
    }

    void initialize_stages()
    {
        stages_ = std::make_unique<StagesType>(this->shared_from_this());
    }

private:
    typename rclcpp_action::Server<ActionType>::SharedPtr action_server_;
    std::unique_ptr<StagesType> stages_;

    // Callbacks
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{
            [this, node_lifetime = shared_from_this(), goal_handle]() {
                this->execute(goal_handle);
            }
        }.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<typename ActionType::Result>();

        try {
            nlohmann::json poses = nlohmann::json::parse(goal->poses_json);

            bool success = stages_->run(*goal, poses);

            result->success = success;
            if (!success) {
                result->error_message = "Stage execution failed";
            }

        } catch (const nlohmann::json::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON error: %s", e.what());
            result->success = false;
            result->error_message = std::string("JSON error: ") + e.what();
            goal_handle->abort(result);
            return;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Execution exception: %s", e.what());
            result->success = false;
            result->error_message = std::string("Execution exception: ") + e.what();
            goal_handle->abort(result);
            return;
        }

        if (rclcpp::ok()) {
            if (result->success) {
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal completed successfully");
            } else {
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "Goal aborted: %s", result->error_message.c_str());
            }
        }
    }
};
