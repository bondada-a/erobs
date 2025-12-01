// MTC Orchestrator: coordinates multi-step robot tasks with gripper/MoveIt management.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include "mtc_pipeline/action/mtc_execution.hpp"
#include "mtc_pipeline/action/move_to_action.hpp"
#include "mtc_pipeline/action/end_effector_action.hpp"
#include "mtc_pipeline/action/tool_exchange_action.hpp"
#include "mtc_pipeline/action/pick_place_action.hpp"
#include "mtc_pipeline/action/vision_move_to_action.hpp"
#include "mtc_pipeline/action/pipettor_action.hpp"
#include "mtc_pipeline/gripper_config_registry.hpp"
#include "mtc_pipeline/core/moveit_lifecycle_manager.hpp"
#include "mtc_pipeline/core/ur_tool_interface.hpp"

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

    /// @brief Construct MTC orchestrator action server
    MTCOrchestratorActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MTCOrchestratorActionServer() override;

private:
    ActionServer::SharedPtr action_server_;
    std::atomic<bool> is_executing_;

    std::shared_ptr<mtc_pipeline::GripperConfigRegistry> gripper_registry_;

    std::unique_ptr<mtc_pipeline::core::URToolInterface> tool_interface_;
    std::unique_ptr<mtc_pipeline::core::MoveItLifecycleManager> moveit_manager_;

    rclcpp_action::Client<MoveToAction>::SharedPtr moveto_action_client_;
    rclcpp_action::Client<EndEffectorAction>::SharedPtr endeffector_action_client_;
    rclcpp_action::Client<ToolExchangeAction>::SharedPtr toolexchange_action_client_;
    rclcpp_action::Client<PickPlaceAction>::SharedPtr pickplace_action_client_;
    rclcpp_action::Client<VisionMoveToAction>::SharedPtr vision_action_client_;
    rclcpp_action::Client<PipettorAction>::SharedPtr pipettor_action_client_;

    /// @brief Handle incoming goal request
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const MTCExecution::Goal> goal);

    /// @brief Handle goal cancellation request
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMTCExecution> goal_handle);

    /// @brief Handle accepted goal by spawning execution thread
    void handle_accepted(const std::shared_ptr<GoalHandleMTCExecution> goal_handle);

    /// @brief Execute goal by processing all tasks in sequence
    void execute(const std::shared_ptr<GoalHandleMTCExecution> goal_handle);

    /// @brief Execute single task step by dispatching to appropriate action
    bool execute_step(const std::string& task_type, const nlohmann::json& step,
                     const std::string& poses_json, const std::string& robot_ip);

    /// @brief Send action goal and wait for result with timeout
    template<typename ActionType>
    bool send_and_wait(
        typename rclcpp_action::Client<ActionType>::SharedPtr client,
        const typename ActionType::Goal& goal,
        const std::string& name,
        std::chrono::seconds timeout);

    /// @brief Call MoveTo action with step parameters
    bool call_moveto_action(const nlohmann::json& step, const std::string& poses_json);

    /// @brief Call EndEffector action with step parameters
    bool call_endeffector_action(const nlohmann::json& step, const std::string& poses_json);

    /// @brief Call ToolExchange action with step parameters
    bool call_toolexchange_action(const nlohmann::json& step, const std::string& poses_json);

    /// @brief Call PickPlace action with step parameters
    bool call_pickplace_action(const nlohmann::json& step, const std::string& poses_json);

    /// @brief Call Vision action with step parameters
    bool call_vision_action(const nlohmann::json& step, const std::string& poses_json);

    /// @brief Call Pipettor action with step parameters
    bool call_pipettor_action(const nlohmann::json& step, const std::string& poses_json);

    /// @brief Handle tool exchange and update MoveIt configuration
    bool handle_tool_exchange(const nlohmann::json& step, const std::string& poses_json, const std::string& robot_ip);

    /// @brief Update and publish feedback during execution
    void update_feedback(std::shared_ptr<MTCExecution::Feedback> feedback,
                        std::shared_ptr<GoalHandleMTCExecution> goal_handle,
                        size_t current_step, size_t total_steps, const std::string& task_type);

    struct ParsedGoal {
        std::string robot_ip;
        std::string start_gripper;
        nlohmann::json tasks;
        std::string poses_json;

        size_t task_count() const { return tasks.size(); }
    };

    class ExecutionGuard {
    public:
        explicit ExecutionGuard(std::atomic<bool>& flag) : flag_(flag) {
            flag_ = true;
        }

        ~ExecutionGuard() {
            flag_ = false;
        }

        ExecutionGuard(const ExecutionGuard&) = delete;
        ExecutionGuard& operator=(const ExecutionGuard&) = delete;
        ExecutionGuard(ExecutionGuard&&) = delete;
        ExecutionGuard& operator=(ExecutionGuard&&) = delete;

    private:
        std::atomic<bool>& flag_;
    };

    /// @brief Parse and validate goal JSON into structured data
    std::optional<ParsedGoal> parse_and_validate_goal(
        const MTCExecution::Goal::ConstSharedPtr& goal,
        std::shared_ptr<MTCExecution::Result>& result);

    /// @brief Execute all tasks in parsed goal sequentially
    bool execute_all_tasks(
        const ParsedGoal& parsed_goal,
        std::shared_ptr<MTCExecution::Feedback>& feedback,
        std::shared_ptr<GoalHandleMTCExecution> goal_handle,
        std::shared_ptr<MTCExecution::Result>& result);

    /// @brief Execute single task from parsed goal
    bool execute_single_task(
        size_t task_index,
        const ParsedGoal& parsed_goal,
        std::shared_ptr<MTCExecution::Feedback>& feedback,
        std::shared_ptr<GoalHandleMTCExecution> goal_handle,
        std::shared_ptr<MTCExecution::Result>& result);
};
