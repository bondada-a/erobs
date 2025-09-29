#ifndef MTC_ORCHESTRATOR_ACTION_SERVER_HPP
#define MTC_ORCHESTRATOR_ACTION_SERVER_HPP

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Third-party includes
#include <nlohmann/json.hpp>

// Standard library includes
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

class SimpleProcessManager;

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


    MTCOrchestratorActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MTCOrchestratorActionServer() override;

private:
    ActionServer::SharedPtr action_server_;
    std::atomic<bool> is_executing_;
    std::unique_ptr<SimpleProcessManager> process_manager_;
    
    
    // Action clients to call modular action servers
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
    bool execute_step(const std::string& task_type, const nlohmann::json& step, 
                     const nlohmann::json& poses, const std::string& robot_ip);
    

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
    );

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
                        size_t current_step, size_t total_steps, const std::string& task_type,
                        const std::string& status_message);

};

#endif // MTC_ORCHESTRATOR_ACTION_SERVER_HPP
