#include "mtc_pipeline/pick_place_stages.hpp"
#include "mtc_pipeline/action/pick_place_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>

class PickPlaceActionServer : public rclcpp::Node
{
public:
    using PickPlaceAction = mtc_pipeline::action::PickPlaceAction;
    using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlaceAction>;

    PickPlaceActionServer() : Node("pickplace_action_server")
    {
        // Create action server
        this->action_server_ = rclcpp_action::create_server<PickPlaceAction>(
            this,
            "pickplace_action",
            std::bind(&PickPlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PickPlaceActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&PickPlaceActionServer::handle_accepted, this, std::placeholders::_1));

        // Initialize pick place stages with empty config (will be updated with poses)
        nlohmann::json config;
        // Note: We'll initialize pick_place_stages_ in a separate method after construction

        RCLCPP_INFO(this->get_logger(), "PickPlace Action Server started");
    }

    void initialize_stages() {
        nlohmann::json config;
        pick_place_stages_ = std::make_unique<PickPlaceStages>(this->shared_from_this(), config);
    }

private:
    rclcpp_action::Server<PickPlaceAction>::SharedPtr action_server_;
    std::unique_ptr<PickPlaceStages> pick_place_stages_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PickPlaceAction::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received PickPlace goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePickPlace> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "PickPlace goal cancellation requested");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
    {
        std::thread{std::bind(&PickPlaceActionServer::execute_pickplace, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute_pickplace(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing PickPlace goal");
        
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<PickPlaceAction::Result>();

        try {
            // Parse JSON from goal
            nlohmann::json step;
            step["gripper"] = goal->gripper;
            step["pick_pose"] = goal->pick_pose;
            step["place_pose"] = goal->place_pose;
            // approach_distance removed - not implemented in stages
            step["planning_type"] = goal->planning_type;
            // arm_group removed - hardcoded as "ur_arm" in stages
            
            // Parse poses JSON
            nlohmann::json poses = nlohmann::json::parse(goal->poses_json);
            
            // Create pick and place poses arrays for the stage
            step["pick_poses"] = {goal->pick_pose + "_approach", goal->pick_pose};
            step["place_poses"] = {goal->place_pose + "_approach", goal->place_pose};

            // Execute using existing PickPlaceStages
            bool success = pick_place_stages_->run(step, poses, this->shared_from_this());
            
            result->success = success;
            if (!success) {
                result->error_message = "PickPlace execution failed";
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "PickPlace execution exception: %s", e.what());
            result->success = false;
            result->error_message = std::string("Exception: ") + e.what();
        }

        // Send result
        if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "PickPlace goal completed");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlaceActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
