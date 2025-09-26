#ifndef MTC_ORCHESTRATOR_ACTION_SERVER_HPP
#define MTC_ORCHESTRATOR_ACTION_SERVER_HPP

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

// Third-party includes
#include <nlohmann/json.hpp>

// Standard library includes
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unordered_map>
#include <vector>

#include "mtc_pipeline/action/mtc_execution.hpp"
#include "mtc_pipeline/action/move_to_action.hpp"
#include "mtc_pipeline/action/end_effector_action.hpp"
#include "mtc_pipeline/action/tool_exchange_action.hpp"
#include "mtc_pipeline/action/pick_place_action.hpp"

using namespace std::chrono_literals;
using MTCExecution = mtc_pipeline::action::MTCExecution;
using GoalHandleMTCExecution = rclcpp_action::ServerGoalHandle<MTCExecution>;
using MoveToAction = mtc_pipeline::action::MoveToAction;
using EndEffectorAction = mtc_pipeline::action::EndEffectorAction;
using ToolExchangeAction = mtc_pipeline::action::ToolExchangeAction;
using PickPlaceAction = mtc_pipeline::action::PickPlaceAction;


class MTCOrchestratorActionServer : public rclcpp::Node
{
public:
    using ActionServer = rclcpp_action::Server<MTCExecution>;

    // Constants for common timeout values
    static constexpr auto ACTION_SERVER_TIMEOUT = std::chrono::seconds(5);

    MTCOrchestratorActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    ActionServer::SharedPtr action_server_;
    std::atomic<bool> is_executing_;
    std::string current_gripper_ = "none";
    
    
    // Action clients to call embedded actions
    rclcpp_action::Client<MoveToAction>::SharedPtr moveto_action_client_;
    rclcpp_action::Client<EndEffectorAction>::SharedPtr endeffector_action_client_;
    rclcpp_action::Client<ToolExchangeAction>::SharedPtr toolexchange_action_client_;
    rclcpp_action::Client<PickPlaceAction>::SharedPtr pickplace_action_client_;
    
    

    // Main MTC execution action server handlers
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MTCExecution::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMTCExecution> goal_handle);
    
    void handle_accepted(const std::shared_ptr<GoalHandleMTCExecution> goal_handle);
    
    void execute(const std::shared_ptr<GoalHandleMTCExecution> goal_handle);


    // Main execution logic
    bool execute_step(const std::string& action, const nlohmann::json& step, 
                     const nlohmann::json& poses, const std::string& robot_ip);
    
    // Gripper switching logic
    bool switch_gripper(const std::string& new_gripper, const std::string& robot_ip);

    // Execute function helpers
    bool initialize_moveit_stack(const std::string& start_gripper, const std::string& robot_ip);
    
    // Generic template for action client calls
    template<typename ActionType>
    bool call_action_generic(
        typename rclcpp_action::Client<ActionType>::SharedPtr client,
        const std::string& action_name,
        const nlohmann::json& step,
        const nlohmann::json& poses,
        std::function<void(typename ActionType::Goal&, const nlohmann::json&, const nlohmann::json&)> populate_goal
    ) {
        if (!client->wait_for_action_server(ACTION_SERVER_TIMEOUT)) {
            RCLCPP_ERROR(this->get_logger(), "%s action server unavailable", action_name.c_str());
            return false;
        }

        auto goal = typename ActionType::Goal();
        populate_goal(goal, step, poses);

        auto future = client->async_send_goal(goal);
        auto goal_handle = future.get();

        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send %s goal", action_name.c_str());
            return false;
        }

        auto result_future = client->async_get_result(goal_handle);
        auto result = result_future.get();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            return result.result->success;
        }
        return false;
    }

    // Action client methods to call modular action servers via ROS2 actions
    bool call_moveto_action(const nlohmann::json& step, const nlohmann::json& poses);
    bool call_endeffector_action(const nlohmann::json& step, const nlohmann::json& poses);
    bool call_toolexchange_action(const nlohmann::json& step, const nlohmann::json& poses);
    bool call_pickplace_action(const nlohmann::json& step, const nlohmann::json& poses);

    // Helper functions for execute_step
    bool handle_tool_exchange(const nlohmann::json& step, const nlohmann::json& poses, const std::string& robot_ip);

    // Helper function for feedback updates
    void update_feedback(std::shared_ptr<MTCExecution::Feedback> feedback,
                        std::shared_ptr<GoalHandleMTCExecution> goal_handle,
                        size_t current_step, size_t total_steps, const std::string& action,
                        const std::string& status_message);

};

#endif // MTC_ORCHESTRATOR_ACTION_SERVER_HPP
