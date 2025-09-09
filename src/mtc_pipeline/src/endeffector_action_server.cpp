#include "mtc_pipeline/end_effector_stages.hpp"
#include "mtc_pipeline/action/end_effector_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>

class EndEffectorActionServer : public rclcpp::Node
{
public:
    using EndEffectorAction = mtc_pipeline::action::EndEffectorAction;
    using GoalHandleEndEffector = rclcpp_action::ServerGoalHandle<EndEffectorAction>;

    EndEffectorActionServer() : Node("endeffector_action_server")
    {
        // Create action server
        this->action_server_ = rclcpp_action::create_server<EndEffectorAction>(
            this,
            "endeffector_action",
            std::bind(&EndEffectorActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&EndEffectorActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&EndEffectorActionServer::handle_accepted, this, std::placeholders::_1));

        // Initialize end effector stages with empty config (will be updated with poses)
        nlohmann::json config;
        // Note: We'll initialize end_effector_stages_ in a separate method after construction

        RCLCPP_INFO(this->get_logger(), "EndEffector Action Server started");
    }

    void initialize_stages() {
        nlohmann::json config;
        end_effector_stages_ = std::make_unique<EndEffectorStages>(this->shared_from_this(), config);
    }

private:
    rclcpp_action::Server<EndEffectorAction>::SharedPtr action_server_;
    std::unique_ptr<EndEffectorStages> end_effector_stages_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const EndEffectorAction::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received EndEffector goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleEndEffector> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "EndEffector goal cancellation requested");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleEndEffector> goal_handle)
    {
        std::thread{std::bind(&EndEffectorActionServer::execute_endeffector, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute_endeffector(const std::shared_ptr<GoalHandleEndEffector> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing EndEffector goal");
        
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<EndEffectorAction::Result>();

        try {
            // Parse JSON from goal
            nlohmann::json step;
            step["end_effector_type"] = goal->end_effector_type;
            step["end_effector_action"] = goal->end_effector_action;
            step["position"] = goal->position;
            step["force"] = goal->force;
            step["pressure"] = goal->pressure;
            
            // Parse poses JSON
            nlohmann::json poses = nlohmann::json::parse(goal->poses_json);

            // Execute using existing EndEffectorStages
            bool success = end_effector_stages_->run(step, poses, this->shared_from_this());
            
            result->success = success;
            if (!success) {
                result->error_message = "EndEffector execution failed";
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "EndEffector execution exception: %s", e.what());
            result->success = false;
            result->error_message = std::string("Exception: ") + e.what();
        }

        // Send result
        if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "EndEffector goal completed");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EndEffectorActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
