// Coordinates multi-step robot tasks by dispatching to specialized action servers.

#include "mtc_pipeline/mtc_orchestrator_action_server.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

// Construction & destruction

MTCOrchestratorActionServer::MTCOrchestratorActionServer(const rclcpp::NodeOptions& options)
    : Node("mtc_orchestrator_action_server", options), is_executing_(false)
{
    try {
        gripper_registry_ = std::make_shared<mtc_pipeline::GripperConfigRegistry>(
            this, "config/grippers.yaml");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(),
                     "Failed to load gripper configuration: %s", e.what());
        throw;
    }

    tool_interface_ = std::make_unique<mtc_pipeline::core::URToolInterface>(
        this, ""
    );
    moveit_manager_ = std::make_unique<mtc_pipeline::core::MoveItLifecycleManager>(
        this,
        gripper_registry_,
        tool_interface_.get()
    );

    action_server_ = rclcpp_action::create_server<MTCExecution>(
        this, "mtc_execution",
        [this](const auto& uuid, const auto& goal) { return handle_goal(uuid, goal); },
        [this](const auto& goal_handle) { return handle_cancel(goal_handle); },
        [this](const auto& goal_handle) { handle_accepted(goal_handle); });

    moveto_action_client_ = rclcpp_action::create_client<MoveToAction>(this, "move_to_action");
    endeffector_action_client_ = rclcpp_action::create_client<EndEffectorAction>(this, "end_effector_action");
    toolexchange_action_client_ = rclcpp_action::create_client<ToolExchangeAction>(this, "tool_exchange_action");
    pickplace_action_client_ = rclcpp_action::create_client<PickPlaceAction>(this, "pick_place_action");
    vision_action_client_ = rclcpp_action::create_client<VisionMoveToAction>(this, "vision_move_to_action");
    pipettor_action_client_ = rclcpp_action::create_client<PipettorAction>(this, "pipettor_action");

    this->declare_parameter("obstacle_config_path", "config/beamline_scene.yaml");

    RCLCPP_INFO(this->get_logger(), "MTC Orchestrator Action Server started");
}

MTCOrchestratorActionServer::~MTCOrchestratorActionServer()
{
    // Cleanup handled by component destructors
}

// Action server callbacks

rclcpp_action::GoalResponse MTCOrchestratorActionServer::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const MTCExecution::Goal> /*goal*/)
{
    if (is_executing_) {
        RCLCPP_WARN(this->get_logger(), "Goal rejected: another task is already executing");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MTCOrchestratorActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleMTCExecution> /*goal_handle*/)
{
    RCLCPP_INFO(this->get_logger(), "Cancel request received - will stop after current task completes");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MTCOrchestratorActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
{
    // Execute in detached thread to avoid blocking
    std::thread{[this, self = shared_from_this(), goal_handle]() {
        execute(goal_handle);
    }}.detach();
}

// Task execution helpers

std::optional<MTCOrchestratorActionServer::ParsedGoal>
MTCOrchestratorActionServer::parse_and_validate_goal(
    const MTCExecution::Goal::ConstSharedPtr& goal,
    std::shared_ptr<MTCExecution::Result>& result)
{
    if (goal->robot_ip.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Goal missing required robot_ip");
        result->success = false;
        result->error_message = "Goal missing required robot_ip";
        return std::nullopt;
    }

    nlohmann::json full_script;
    try {
        full_script = nlohmann::json::parse(goal->full_json);
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid JSON: %s", e.what());
        result->success = false;
        result->error_message = std::string("Invalid JSON: ") + e.what();
        return std::nullopt;
    }

    if (!full_script.contains("start_gripper") || !full_script["start_gripper"].is_string()) {
        RCLCPP_ERROR(this->get_logger(), "Task script missing required start_gripper");
        result->success = false;
        result->error_message = "Task script missing required start_gripper";
        return std::nullopt;
    }

    if (!full_script.contains("tasks") || !full_script["tasks"].is_array()) {
        RCLCPP_ERROR(this->get_logger(), "Task script missing required 'tasks' array");
        result->success = false;
        result->error_message = "Task script missing required 'tasks' array";
        return std::nullopt;
    }

    ParsedGoal parsed;
    parsed.robot_ip = goal->robot_ip;
    parsed.start_gripper = full_script["start_gripper"].get<std::string>();
    parsed.tasks = full_script["tasks"];
    parsed.poses_json = full_script.value("poses", nlohmann::json::object()).dump();

    return parsed;
}

bool MTCOrchestratorActionServer::execute_single_task(
    size_t task_index,
    const ParsedGoal& parsed_goal,
    std::shared_ptr<MTCExecution::Feedback>& feedback,
    std::shared_ptr<GoalHandleMTCExecution> goal_handle,
    std::shared_ptr<MTCExecution::Result>& result)
{
    const auto& step = parsed_goal.tasks[task_index];
    std::string task_type = step.value("task_type", "");

    if (task_type.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Step %zu missing 'task_type' field", task_index);
        result->success = false;
        result->error_message = "Step " + std::to_string(task_index) +
                                " missing required 'task_type' field";
        result->completed_steps = task_index;
        return false;
    }

    update_feedback(feedback, goal_handle, task_index + 1, parsed_goal.task_count(), task_type);

    if (!execute_step(task_type, step, parsed_goal.poses_json, parsed_goal.robot_ip)) {
        RCLCPP_ERROR(this->get_logger(), "%s step failed", task_type.c_str());
        result->success = false;
        result->error_message = task_type + " step failed";
        result->completed_steps = task_index;
        return false;
    }

    return true;
}

bool MTCOrchestratorActionServer::execute_all_tasks(
    const ParsedGoal& parsed_goal,
    std::shared_ptr<MTCExecution::Feedback>& feedback,
    std::shared_ptr<GoalHandleMTCExecution> goal_handle,
    std::shared_ptr<MTCExecution::Result>& result)
{
    for (size_t i = 0; i < parsed_goal.task_count(); ++i) {
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->error_message = "Task was canceled";
            goal_handle->canceled(result);
            return false;
        }

        if (!execute_single_task(i, parsed_goal, feedback, goal_handle, result)) {
            return false;
        }

        result->completed_steps = i + 1;
    }

    return true;
}

// Main execution

void MTCOrchestratorActionServer::execute(
    const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    ExecutionGuard guard(is_executing_);

    auto result = std::make_shared<MTCExecution::Result>();
    auto feedback = std::make_shared<MTCExecution::Feedback>();

    auto parsed_goal = parse_and_validate_goal(goal_handle->get_goal(), result);
    if (!parsed_goal) {
        goal_handle->abort(result);
        return;
    }

    update_feedback(feedback, goal_handle, 0, parsed_goal->task_count(), "Initializing MoveIt");
    if (!tool_interface_->set_robot_ip(parsed_goal->robot_ip)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid robot IP address: %s", parsed_goal->robot_ip.c_str());
        result->success = false;
        result->error_message = "Invalid robot IP address format";
        goal_handle->abort(result);
        return;
    }
    if (!moveit_manager_->launch_for_gripper(parsed_goal->start_gripper, parsed_goal->robot_ip)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt stack");
        result->success = false;
        result->error_message = "Failed to initialize MoveIt stack";
        goal_handle->abort(result);
        return;
    }

    if (!execute_all_tasks(*parsed_goal, feedback, goal_handle, result)) {
        return;
    }

    result->success = true;
    result->total_steps = parsed_goal->task_count();
    update_feedback(feedback, goal_handle, parsed_goal->task_count(), parsed_goal->task_count(), "");
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded - keeping MoveIt running");
}

bool MTCOrchestratorActionServer::execute_step(
    const std::string& task_type,
    const nlohmann::json& step,
    const std::string& poses_json,
    const std::string& robot_ip)
{
    if (task_type == "moveto")         return call_moveto_action(step, poses_json);
    if (task_type == "end_effector")   return call_endeffector_action(step, poses_json);
    if (task_type == "pick_and_place") return call_pickplace_action(step, poses_json);
    if (task_type == "vision_moveto")  return call_vision_action(step, poses_json);
    if (task_type == "pipettor")       return call_pipettor_action(step, poses_json);
    if (task_type == "tool_exchange")  return handle_tool_exchange(step, poses_json, robot_ip);

    RCLCPP_ERROR(this->get_logger(), "Unknown task type: %s", task_type.c_str());
    return false;
}

// Action client calls

template<typename ActionType>
bool MTCOrchestratorActionServer::send_and_wait(
    typename rclcpp_action::Client<ActionType>::SharedPtr client,
    const typename ActionType::Goal& goal,
    const std::string& name,
    std::chrono::seconds timeout)
{
    if (!client->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(), "%s action server unavailable", name.c_str());
        return false;
    }

    auto goal_handle = client->async_send_goal(goal).get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send %s goal", name.c_str());
        return false;
    }

    auto result_future = client->async_get_result(goal_handle);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "%s timed out after %lds", name.c_str(), timeout.count());
        client->async_cancel_goal(goal_handle);
        return false;
    }

    auto result = result_future.get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
}

bool MTCOrchestratorActionServer::call_moveto_action(
    const nlohmann::json& step,
    const std::string& poses_json)
{
    MoveToAction::Goal goal;
    goal.target = step.value("target", "");
    goal.planning_type = step.value("planning_type", "joint");
    goal.direction = step.value("direction", "");
    goal.distance = step.value("distance", 0.0);
    goal.poses_json = poses_json;

    return send_and_wait<MoveToAction>(moveto_action_client_, goal, "moveto", 120s);
}

bool MTCOrchestratorActionServer::call_endeffector_action(
    const nlohmann::json& step,
    const std::string& poses_json)
{
    EndEffectorAction::Goal goal;
    goal.end_effector_type = step.value("end_effector_type", "");
    goal.end_effector_action = step.value("end_effector_action", "");
    goal.poses_json = poses_json;

    return send_and_wait<EndEffectorAction>(endeffector_action_client_, goal, "end_effector", 30s);
}

bool MTCOrchestratorActionServer::call_pickplace_action(
    const nlohmann::json& step,
    const std::string& poses_json)
{
    PickPlaceAction::Goal goal;
    goal.gripper = step.value("gripper", "");
    goal.pick_approach = step.value("pick_approach", "");
    goal.pick_target = step.value("pick_target", "");
    goal.place_approach = step.value("place_approach", "");
    goal.place_target = step.value("place_target", "");
    goal.poses_json = poses_json;

    return send_and_wait<PickPlaceAction>(pickplace_action_client_, goal, "pick_place", 180s);
}

bool MTCOrchestratorActionServer::call_vision_action(
    const nlohmann::json& step,
    const std::string& poses_json)
{
    VisionMoveToAction::Goal goal;
    goal.tag_id = step.value("tag_id", 0);
    goal.timeout = step.value("timeout", 5.0);
    goal.poses_json = poses_json;

    return send_and_wait<VisionMoveToAction>(vision_action_client_, goal, "vision_moveto", 60s);
}

bool MTCOrchestratorActionServer::call_pipettor_action(
    const nlohmann::json& step,
    const std::string& poses_json)
{
    PipettorAction::Goal goal;
    goal.operation = step.value("operation", "");
    goal.volume_pct = step.value("volume_pct", 0.0);
    goal.poses_json = poses_json;

    if (step.contains("led_color")) {
        goal.led_color.r = step["led_color"].value("r", 0.0);
        goal.led_color.g = step["led_color"].value("g", 0.0);
        goal.led_color.b = step["led_color"].value("b", 0.0);
        goal.led_color.a = step["led_color"].value("a", 1.0);
    }

    return send_and_wait<PipettorAction>(pipettor_action_client_, goal, "pipettor", 60s);
}

bool MTCOrchestratorActionServer::call_toolexchange_action(
    const nlohmann::json& step,
    const std::string& poses_json)
{
    ToolExchangeAction::Goal goal;
    goal.operation = step.value("operation", "");
    goal.gripper = step.value("gripper", "");
    goal.current_attached_gripper = moveit_manager_->current_gripper();
    goal.dock_number = step.value("dock_number", 0);
    goal.approach_pose = step.value("approach_pose", "");
    goal.poses_json = poses_json;

    return send_and_wait<ToolExchangeAction>(toolexchange_action_client_, goal, "tool_exchange", 180s);
}

bool MTCOrchestratorActionServer::handle_tool_exchange(
    const nlohmann::json& step,
    const std::string& poses_json,
    const std::string& robot_ip)
{
    if (!call_toolexchange_action(step, poses_json)) {
        return false;
    }

    const std::string operation = step.value("operation", "");

    if (operation == "dock") {
        return moveit_manager_->launch_for_gripper("none", robot_ip);
    }
    if (operation == "load") {
        return moveit_manager_->launch_for_gripper(
            step.value("gripper", moveit_manager_->current_gripper()), robot_ip);
    }

    return true;
}

// Utilities

void MTCOrchestratorActionServer::update_feedback(
    std::shared_ptr<MTCExecution::Feedback> feedback,
    std::shared_ptr<GoalHandleMTCExecution> goal_handle,
    size_t current_step,
    size_t total_steps,
    const std::string& task_type)
{
    feedback->current_step = current_step;
    feedback->current_action = task_type;
    feedback->progress_percentage = static_cast<float>(current_step) / total_steps * 100.0f;
    feedback->status_message = task_type.empty() ? "Task completed" : "Executing: " + task_type;
    feedback->current_gripper = moveit_manager_->current_gripper();

    goal_handle->publish_feedback(feedback);
}

// Main

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<MTCOrchestratorActionServer>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
