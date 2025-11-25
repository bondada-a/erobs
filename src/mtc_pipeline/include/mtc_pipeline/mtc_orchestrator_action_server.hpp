// MTC Orchestrator: coordinates multi-step robot tasks with gripper/MoveIt management.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "mtc_pipeline/action/mtc_execution.hpp"
#include "mtc_pipeline/action/move_to_action.hpp"
#include "mtc_pipeline/action/end_effector_action.hpp"
#include "mtc_pipeline/action/tool_exchange_action.hpp"
#include "mtc_pipeline/action/pick_place_action.hpp"
#include "mtc_pipeline/action/vision_move_to_action.hpp"
#include "mtc_pipeline/action/pipettor_action.hpp"

using MTCExecution = mtc_pipeline::action::MTCExecution;
using GoalHandleMTCExecution = rclcpp_action::ServerGoalHandle<MTCExecution>;
using MoveToAction = mtc_pipeline::action::MoveToAction;
using EndEffectorAction = mtc_pipeline::action::EndEffectorAction;
using ToolExchangeAction = mtc_pipeline::action::ToolExchangeAction;
using PickPlaceAction = mtc_pipeline::action::PickPlaceAction;
using VisionMoveToAction = mtc_pipeline::action::VisionMoveToAction;
using PipettorAction = mtc_pipeline::action::PipettorAction;

class MTCOrchestratorActionServer : public rclcpp::Node
{
public:
    using ActionServer = rclcpp_action::Server<MTCExecution>;

    MTCOrchestratorActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MTCOrchestratorActionServer() override;

private:
    ActionServer::SharedPtr action_server_;
    std::atomic<bool> is_executing_;

    // MoveIt process management
    std::string current_gripper_;
    pid_t moveit_pid_{0};
    pid_t launch_moveit_process(const std::string& command);
    void kill_moveit_process();

    // Action clients
    rclcpp_action::Client<MoveToAction>::SharedPtr moveto_action_client_;
    rclcpp_action::Client<EndEffectorAction>::SharedPtr endeffector_action_client_;
    rclcpp_action::Client<ToolExchangeAction>::SharedPtr toolexchange_action_client_;
    rclcpp_action::Client<PickPlaceAction>::SharedPtr pickplace_action_client_;
    rclcpp_action::Client<VisionMoveToAction>::SharedPtr vision_action_client_;
    rclcpp_action::Client<PipettorAction>::SharedPtr pipettor_action_client_;

    // Action server handlers
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const MTCExecution::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMTCExecution> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleMTCExecution> goal_handle);
    void execute(const std::shared_ptr<GoalHandleMTCExecution> goal_handle);

    // Step execution
    bool execute_step(const std::string& task_type, const nlohmann::json& step,
                     const std::string& poses_json, const std::string& robot_ip);
    bool initialize_moveit_stack(const std::string& gripper, const std::string& robot_ip);
    bool set_tool_voltage_via_socket(const std::string& robot_ip, int voltage);

    // Generic send-and-wait helper for action clients
    template<typename ActionType>
    bool send_and_wait(
        typename rclcpp_action::Client<ActionType>::SharedPtr client,
        const typename ActionType::Goal& goal,
        const std::string& name,
        std::chrono::seconds timeout);

    // Action client calls
    bool call_moveto_action(const nlohmann::json& step, const std::string& poses_json);
    bool call_endeffector_action(const nlohmann::json& step, const std::string& poses_json);
    bool call_toolexchange_action(const nlohmann::json& step, const std::string& poses_json);
    bool call_pickplace_action(const nlohmann::json& step, const std::string& poses_json);
    bool call_vision_action(const nlohmann::json& step, const std::string& poses_json);
    bool call_pipettor_action(const nlohmann::json& step, const std::string& poses_json);
    bool handle_tool_exchange(const nlohmann::json& step, const std::string& poses_json, const std::string& robot_ip);

    void update_feedback(std::shared_ptr<MTCExecution::Feedback> feedback,
                        std::shared_ptr<GoalHandleMTCExecution> goal_handle,
                        size_t current_step, size_t total_steps, const std::string& task_type);
};
