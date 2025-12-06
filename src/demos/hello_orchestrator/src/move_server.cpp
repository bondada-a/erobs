// Simple action server that moves robot to named poses using MoveIt

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <hello_orchestrator/action/move_to_named.hpp>

using MoveToNamed = hello_orchestrator::action::MoveToNamed;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveToNamed>;

class MoveServer : public rclcpp::Node
{
public:
    MoveServer() : Node("move_server")
    {
        // Initialization happens in initialize() method after node is in shared_ptr
    }

    void initialize()
    {
        // Create MoveGroupInterface (ur_arm is standard for UR robots)
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur_arm");

        action_server_ = rclcpp_action::create_server<MoveToNamed>(
            this,
            "move_to_named",
            std::bind(&MoveServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveServer::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Move action server started");
    }

private:
    rclcpp_action::Server<MoveToNamed>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const MoveToNamed::Goal> /*goal*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received move goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMove> /*goal_handle*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        std::thread{[this, goal_handle]() {
            execute(goal_handle);
        }}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing move goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveToNamed::Feedback>();
        auto result = std::make_shared<MoveToNamed::Result>();

        // Update feedback
        feedback->status = "Planning to " + goal->target_pose;
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "🤖 MOVING to: %s", goal->target_pose.c_str());

        // Set named target
        move_group_->setNamedTarget(goal->target_pose);

        // Plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            result->success = false;
            result->error_message = "Planning failed for target: " + goal->target_pose;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->error_message.c_str());
            return;
        }

        // Update feedback
        feedback->status = "Executing motion to " + goal->target_pose;
        goal_handle->publish_feedback(feedback);

        // Execute
        success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            result->success = false;
            result->error_message = "Execution failed for target: " + goal->target_pose;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->error_message.c_str());
            return;
        }

        // Success
        result->success = true;
        result->error_message = "";
        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(), "Move goal completed successfully");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveServer>();
    node->initialize();  // Initialize after node is in shared_ptr
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
