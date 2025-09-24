#include "mtc_pipeline/moveto_stages.hpp"
#include "mtc_pipeline/action/move_to_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>

class MoveToActionServer : public rclcpp::Node
{
public:
    using MoveToAction = mtc_pipeline::action::MoveToAction;
    using GoalHandleMoveTo = rclcpp_action::ServerGoalHandle<MoveToAction>;

    MoveToActionServer() : Node("moveto_action_server")
    {
        // Create action server
        this->action_server_ = rclcpp_action::create_server<MoveToAction>(
            this,
            "moveto_action",
            std::bind(&MoveToActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveToActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveToActionServer::handle_accepted, this, std::placeholders::_1));

        // Initialize move to stages with empty config (will be updated with poses)
        nlohmann::json config;
        // Note: We'll initialize moveto_stages_ in a separate method after construction

        RCLCPP_INFO(this->get_logger(), "MoveTo Action Server started");
    }

    void initialize_stages() {
        nlohmann::json config;
        moveto_stages_ = std::make_unique<MoveToStages>(this->shared_from_this(), config);
    }

private:
    rclcpp_action::Server<MoveToAction>::SharedPtr action_server_;
    std::unique_ptr<MoveToStages> moveto_stages_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveToAction::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received MoveTo goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveTo> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "MoveTo goal cancellation requested");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
    {
        std::thread{std::bind(&MoveToActionServer::execute_moveto, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute_moveto(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing MoveTo goal");
        
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MoveToAction::Result>();

        try {
            // Parse JSON from goal
            nlohmann::json step;
            step["target_type"] = goal->target_type;
            step["target"] = goal->target;
            step["planning_type"] = goal->planning_type;
            // arm_group removed - hardcoded as "ur_arm" in stages
            step["direction"] = goal->direction;
            step["distance"] = goal->distance;
            
            // Parse poses JSON
            nlohmann::json poses = nlohmann::json::parse(goal->poses_json);

            // Execute using existing MoveToStages
            bool success = moveto_stages_->run(step, poses, this->shared_from_this());
            
            result->success = success;
            if (!success) {
                result->error_message = "MoveTo execution failed";
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "MoveTo execution exception: %s", e.what());
            result->success = false;
            result->error_message = std::string("Exception: ") + e.what();
        }

        // Send result
        if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "MoveTo goal completed");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
