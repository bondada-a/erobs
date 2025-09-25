#ifndef MTC_ORCHESTRATOR_ACTION_SERVER_HPP
#define MTC_ORCHESTRATOR_ACTION_SERVER_HPP

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_srvs/srv/trigger.hpp>

// Third-party includes
#include <nlohmann/json.hpp>

// Standard library includes
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <linux/limits.h>
#include <memory>
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

namespace {
    // Wait for ROS2 service to become available
    bool wait_for_service(rclcpp::Node::SharedPtr node, const std::string& service_name, std::chrono::seconds timeout = 30s);
    
    
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
    
    // Action client methods to call modular action servers via ROS2 actions
    bool call_moveto_action(const nlohmann::json& step, const nlohmann::json& poses);
    bool call_endeffector_action(const nlohmann::json& step, const nlohmann::json& poses);
    bool call_toolexchange_action(const nlohmann::json& step, const nlohmann::json& poses);
    bool call_pickplace_action(const nlohmann::json& step, const nlohmann::json& poses);
    
};

#endif // MTC_ORCHESTRATOR_ACTION_SERVER_HPP
