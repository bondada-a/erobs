#include "mtc_pipeline/vision_pick_place_stages.hpp"
#include "mtc_pipeline/action/vision_pick_place_action.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <thread>

class VisionPickPlaceActionServer : public rclcpp::Node
{
public:
    VisionPickPlaceActionServer() : Node("vision_pick_place_action_server")
    {
        using namespace std::placeholders;

        action_server_ = rclcpp_action::create_server<mtc_pipeline::action::VisionPickPlaceAction>(
            this,
            "vision_pick_place_action",
            std::bind(&VisionPickPlaceActionServer::handle_goal, this, _1, _2),
            std::bind(&VisionPickPlaceActionServer::handle_cancel, this, _1),
            std::bind(&VisionPickPlaceActionServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Vision Pick Place Action Server started");
    }

    void initialize_stages()
    {
        stages_ = std::make_shared<VisionPickPlaceStages>(shared_from_this());
    }

private:
    std::shared_ptr<VisionPickPlaceStages> stages_;
    rclcpp_action::Server<mtc_pipeline::action::VisionPickPlaceAction>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const mtc_pipeline::action::VisionPickPlaceAction::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request: pick_tag=%d, place_tag=%d",
                    goal->pick_tag_id, goal->place_tag_id);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<mtc_pipeline::action::VisionPickPlaceAction>> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<mtc_pipeline::action::VisionPickPlaceAction>> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&VisionPickPlaceActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<mtc_pipeline::action::VisionPickPlaceAction>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<mtc_pipeline::action::VisionPickPlaceAction::Result>();

        try {
            nlohmann::json step;

            // Vision detection parameters
            step["pick_tag_id"] = goal->pick_tag_id;
            step["place_tag_id"] = goal->place_tag_id;

            // Gripper configuration
            step["gripper"] = goal->gripper;

            // Grasp offset configuration (JSON string)
            step["grasp_offset_json"] = goal->grasp_offset_json;

            // Place poses for fallback (when place_tag_id == -1)
            if (!goal->place_poses_json.empty()) {
                step["place_poses_json"] = goal->place_poses_json;
            }

            // Approach and retreat offsets
            step["approach_offset"] = goal->approach_offset;
            step["retreat_offset"] = goal->retreat_offset;

            // Empty poses object since vision doesn't use predefined poses
            nlohmann::json poses = nlohmann::json::object();

            // Execute using stages
            bool success = stages_->run(step, poses);

            result->success = success;
            if (!success) {
                result->error_message = "Vision pick place execution failed";
            }

            if (result->success) {
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            } else {
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "Goal failed: %s", result->error_message.c_str());
            }

        } catch (const std::exception& e) {
            result->success = false;
            result->error_message = std::string("Exception: ") + e.what();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Goal failed with exception: %s", e.what());
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionPickPlaceActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}