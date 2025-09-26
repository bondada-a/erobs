#ifndef BASE_ACTION_SERVER_HPP
#define BASE_ACTION_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <thread>
#include <functional>

template<typename ActionType, typename StagesType>
class BaseActionServer : public rclcpp::Node
{
public:

    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;

    BaseActionServer(const std::string& node_name, const std::string& action_name)
        : Node(node_name)
    {
        // Create action server
        this->action_server_ = rclcpp_action::create_server<ActionType>(
            this,
            action_name,
            std::bind(&BaseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BaseActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&BaseActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "%s Action Server started", node_name.c_str());
    }

    // Initialize stages after object is fully constructed and managed by shared_ptr
    void initialize_stages() {
        stages_ = std::make_unique<StagesType>(this->shared_from_this(), nlohmann::json{});
    }

protected:
    // Pure virtual function that each derived class must implement
    // This is the only part that differs between action servers
    virtual nlohmann::json goal_to_step(const typename ActionType::Goal& goal) = 0;

private:
    typename rclcpp_action::Server<ActionType>::SharedPtr action_server_;
    std::unique_ptr<StagesType> stages_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const typename ActionType::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal cancellation requested");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        auto self = this->shared_from_this();
        std::thread{[self, goal_handle]() { self->execute(goal_handle); }}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<typename ActionType::Result>();

        try {
            // Convert goal to step JSON using derived class implementation
            nlohmann::json step = goal_to_step(*goal);

            // Parse poses JSON
            nlohmann::json poses = nlohmann::json::parse(goal->poses_json);

            // Execute using stages
            bool success = stages_->run(step, poses, this->shared_from_this());

            result->success = success;
            if (!success) {
                result->error_message = "Execution failed";
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Execution exception: %s", e.what());
            result->success = false;
            result->error_message = std::string("Exception: ") + e.what();
        }

        // Send result
        if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal completed");
        }
    }
};

#endif // BASE_ACTION_SERVER_HPP