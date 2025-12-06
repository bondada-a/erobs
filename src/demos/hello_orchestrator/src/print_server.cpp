// Simple action server that prints messages to console

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <hello_orchestrator/action/print_message.hpp>

using PrintMessage = hello_orchestrator::action::PrintMessage;
using GoalHandlePrint = rclcpp_action::ServerGoalHandle<PrintMessage>;

class PrintServer : public rclcpp::Node
{
public:
    PrintServer() : Node("print_server")
    {
        action_server_ = rclcpp_action::create_server<PrintMessage>(
            this,
            "print_message",
            std::bind(&PrintServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PrintServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&PrintServer::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Print action server started");
    }

private:
    rclcpp_action::Server<PrintMessage>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const PrintMessage::Goal> /*goal*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received print goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePrint> /*goal_handle*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePrint> goal_handle)
    {
        std::thread{[this, goal_handle]() {
            execute(goal_handle);
        }}.detach();
    }

    void execute(const std::shared_ptr<GoalHandlePrint> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing print goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<PrintMessage::Feedback>();
        auto result = std::make_shared<PrintMessage::Result>();

        // Publish feedback
        feedback->current_message = goal->message;
        goal_handle->publish_feedback(feedback);

        // Print the message
        RCLCPP_INFO(this->get_logger(), "📝 MESSAGE: %s", goal->message.c_str());

        // Set result
        result->success = true;
        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(), "Print goal completed");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PrintServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
