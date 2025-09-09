#include "mtc_pipeline/tool_exchange_stages.hpp"
#include "mtc_pipeline/action/tool_exchange_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>

class ToolExchangeActionServer : public rclcpp::Node
{
public:
    using ToolExchangeAction = mtc_pipeline::action::ToolExchangeAction;
    using GoalHandleToolExchange = rclcpp_action::ServerGoalHandle<ToolExchangeAction>;

    ToolExchangeActionServer() : Node("toolexchange_action_server")
    {
        // Create action server
        this->action_server_ = rclcpp_action::create_server<ToolExchangeAction>(
            this,
            "toolexchange_action",
            std::bind(&ToolExchangeActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ToolExchangeActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ToolExchangeActionServer::handle_accepted, this, std::placeholders::_1));

        // Initialize tool exchange stages with empty config (will be updated with poses)
        nlohmann::json config;
        // Note: We'll initialize tool_exchange_stages_ in a separate method after construction

        RCLCPP_INFO(this->get_logger(), "ToolExchange Action Server started");
    }

    void initialize_stages() {
        nlohmann::json config;
        tool_exchange_stages_ = std::make_unique<ToolExchangeStages>(this->shared_from_this(), config);
    }

private:
    rclcpp_action::Server<ToolExchangeAction>::SharedPtr action_server_;
    std::unique_ptr<ToolExchangeStages> tool_exchange_stages_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ToolExchangeAction::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received ToolExchange goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleToolExchange> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "ToolExchange goal cancellation requested");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleToolExchange> goal_handle)
    {
        std::thread{std::bind(&ToolExchangeActionServer::execute_toolexchange, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute_toolexchange(const std::shared_ptr<GoalHandleToolExchange> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing ToolExchange goal");
        
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ToolExchangeAction::Result>();

        try {
            // Parse JSON from goal
            nlohmann::json step;
            step["operation"] = goal->operation;
            step["gripper"] = goal->gripper;
            step["dock_number"] = goal->dock_number;
            
            // Parse poses JSON
            nlohmann::json poses = nlohmann::json::parse(goal->poses_json);
            
            // Add approach pose to step (required by ToolExchangeStages)
            if (poses.contains("approach_pose")) {
                step["poses"] = {poses["approach_pose"]};
            } else {
                // Default approach pose if not specified
                step["poses"] = {"approach_pose"};
            }

            // Execute using existing ToolExchangeStages
            bool success = tool_exchange_stages_->run(step, poses, this->shared_from_this());
            
            result->success = success;
            if (!success) {
                result->error_message = "ToolExchange execution failed";
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "ToolExchange execution exception: %s", e.what());
            result->success = false;
            result->error_message = std::string("Exception: ") + e.what();
        }

        // Send result
        if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "ToolExchange goal completed");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ToolExchangeActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
