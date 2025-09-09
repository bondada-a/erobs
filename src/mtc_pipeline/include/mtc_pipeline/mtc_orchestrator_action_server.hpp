#ifndef MTC_ORCHESTRATOR_ACTION_SERVER_HPP
#define MTC_ORCHESTRATOR_ACTION_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter_client.hpp>
#include <memory>
#include <thread>
#include <chrono>
#include <future>
#include <atomic>
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <string>
#include <iostream>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <linux/limits.h>
#include <cstdlib>

#include "mtc_pipeline/action/mtc_execution.hpp"
#include "mtc_pipeline/action/move_to_action.hpp"
#include "mtc_pipeline/action/end_effector_action.hpp"
#include "mtc_pipeline/action/tool_exchange_action.hpp"
#include "mtc_pipeline/action/pick_place_action.hpp"
#include "mtc_pipeline/pick_place_stages.hpp"
#include "mtc_pipeline/tool_exchange_stages.hpp"
#include "mtc_pipeline/moveto_stages.hpp"
#include "mtc_pipeline/end_effector_stages.hpp"

using namespace std::chrono_literals;
using MTCExecution = mtc_pipeline::action::MTCExecution;
using GoalHandleMTCExecution = rclcpp_action::ServerGoalHandle<MTCExecution>;
using MoveToAction = mtc_pipeline::action::MoveToAction;
using EndEffectorAction = mtc_pipeline::action::EndEffectorAction;
using ToolExchangeAction = mtc_pipeline::action::ToolExchangeAction;
using PickPlaceAction = mtc_pipeline::action::PickPlaceAction;

namespace {
    // Wait for ROS2 service to become available
    bool wait_for_service(rclcpp::Node::SharedPtr node, const std::string& service_name, std::chrono::seconds timeout = 30s);
    
    // Execute shell command and check if output contains expected string
    bool check_command_output(const std::string& cmd, const std::string& expected);
    
    // Update robot description from another node
    bool update_robot_description_from(const std::string& source_node, rclcpp::Node::SharedPtr target_node);
}

// Process management class
class Orchestrator {
public:
    // Launch new MoveIt configuration process
    pid_t launch(const std::string& cmd);
    
    // Kill all active processes and wait for them to finish
    void kill_all_and_wait();
    
    // Check if any processes are still running
    bool has_active_processes() const;
    
    // Gripper management
    void set_current_gripper(const std::string& g);
    const std::string& get_current_gripper() const;
    
private:
    std::vector<pid_t> active_pids_;
    std::string current_gripper_ = "none";
};

class MTCOrchestratorActionServer : public rclcpp::Node
{
public:
    using ActionServer = rclcpp_action::Server<MTCExecution>;

    MTCOrchestratorActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    ActionServer::SharedPtr action_server_;
    std::unique_ptr<Orchestrator> orchestrator_;
    bool is_executing_;
    
    // Embedded action servers
    rclcpp_action::Server<MoveToAction>::SharedPtr moveto_action_server_;
    rclcpp_action::Server<EndEffectorAction>::SharedPtr endeffector_action_server_;
    rclcpp_action::Server<ToolExchangeAction>::SharedPtr toolexchange_action_server_;
    rclcpp_action::Server<PickPlaceAction>::SharedPtr pickplace_action_server_;
    
    // Action clients to call embedded actions
    rclcpp_action::Client<MoveToAction>::SharedPtr moveto_action_client_;
    rclcpp_action::Client<EndEffectorAction>::SharedPtr endeffector_action_client_;
    rclcpp_action::Client<ToolExchangeAction>::SharedPtr toolexchange_action_client_;
    rclcpp_action::Client<PickPlaceAction>::SharedPtr pickplace_action_client_;
    
    // Execution state for abort capability
    std::atomic<bool> moveto_abort_requested_{false};
    std::atomic<bool> endeffector_abort_requested_{false};
    std::atomic<bool> toolexchange_abort_requested_{false};
    std::atomic<bool> pickplace_abort_requested_{false};
    
    // Reusable stage instances - created once, reused multiple times
    std::shared_ptr<MoveToStages> moveto_instance_;
    std::shared_ptr<EndEffectorStages> endeffector_instance_;
    std::shared_ptr<ToolExchangeStages> toolexchange_instance_;
    std::shared_ptr<PickPlaceStages> pickplace_instance_;

    // Main MTC execution action server handlers
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MTCExecution::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMTCExecution> goal_handle);
    
    void handle_accepted(const std::shared_ptr<GoalHandleMTCExecution> goal_handle);
    
    void execute(const std::shared_ptr<GoalHandleMTCExecution> goal_handle);

    // Embedded MoveTo action server handlers
    rclcpp_action::GoalResponse handle_moveto_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveToAction::Goal> goal);
    
    rclcpp_action::CancelResponse handle_moveto_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToAction>> goal_handle);
    
    void handle_moveto_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToAction>> goal_handle);
    
    void execute_moveto_embedded(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToAction>> goal_handle);

    // Embedded EndEffector action server handlers
    rclcpp_action::GoalResponse handle_endeffector_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const EndEffectorAction::Goal> goal);
    
    rclcpp_action::CancelResponse handle_endeffector_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<EndEffectorAction>> goal_handle);
    
    void handle_endeffector_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<EndEffectorAction>> goal_handle);
    
    void execute_endeffector_embedded(const std::shared_ptr<rclcpp_action::ServerGoalHandle<EndEffectorAction>> goal_handle);

    // Embedded ToolExchange action server handlers
    rclcpp_action::GoalResponse handle_toolexchange_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ToolExchangeAction::Goal> goal);
    
    rclcpp_action::CancelResponse handle_toolexchange_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ToolExchangeAction>> goal_handle);
    
    void handle_toolexchange_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ToolExchangeAction>> goal_handle);
    
    void execute_toolexchange_embedded(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ToolExchangeAction>> goal_handle);

    // Embedded PickPlace action server handlers
    rclcpp_action::GoalResponse handle_pickplace_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PickPlaceAction::Goal> goal);
    
    rclcpp_action::CancelResponse handle_pickplace_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceAction>> goal_handle);
    
    void handle_pickplace_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceAction>> goal_handle);
    
    void execute_pickplace_embedded(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceAction>> goal_handle);

    // Internal execution methods (used by execute_step)
    bool execute_moveto_embedded_internal(const nlohmann::json& step, const nlohmann::json& poses);
    bool execute_endeffector_embedded_internal(const nlohmann::json& step, const nlohmann::json& poses);
    bool execute_toolexchange_embedded_internal(const nlohmann::json& step, const nlohmann::json& poses);
    bool execute_pickplace_embedded_internal(const nlohmann::json& step, const nlohmann::json& poses);

    // Main execution logic
    bool execute_step(const std::string& action, const nlohmann::json& step, 
                     const nlohmann::json& poses, const std::string& robot_ip);
    
    // Gripper switching logic
    bool switch_gripper(const std::string& new_gripper, const std::string& robot_ip);
    
    // Template for simple action handlers
    template<typename ActionType>
    rclcpp_action::GoalResponse handle_simple_goal(const rclcpp_action::GoalUUID& uuid, 
                                                   std::shared_ptr<const typename ActionType::Goal> goal,
                                                   const std::string& action_name);
    
    template<typename ActionType>
    rclcpp_action::CancelResponse handle_simple_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle,
                                                       std::atomic<bool>& abort_flag,
                                                       const std::string& action_name);
    
    template<typename ActionType>
    void handle_simple_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle,
                                void (MTCOrchestratorActionServer::*execute_func)(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>>));
};

#endif // MTC_ORCHESTRATOR_ACTION_SERVER_HPP
