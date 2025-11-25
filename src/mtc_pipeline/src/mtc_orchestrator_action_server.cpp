/**
 * MTC Orchestrator Action Server
 *
 * Coordinates multi-step robot tasks by:
 * 1. Receiving task scripts (JSON) from clients
 * 2. Managing MoveIt process lifecycle (launch/kill based on gripper type)
 * 3. Dispatching individual steps to specialized action servers
 *
 * Execution Flow:
 *   Client → handle_goal → handle_accepted → execute → execute_step → call_*_action
 */

#include "mtc_pipeline/mtc_orchestrator_action_server.hpp"
#include "mtc_pipeline/obstacle_loader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

// ============================================================================
// CONSTRUCTION & DESTRUCTION
// ============================================================================

MTCOrchestratorActionServer::MTCOrchestratorActionServer(const rclcpp::NodeOptions& options)
    : Node("mtc_orchestrator_action_server", options), is_executing_(false)
{
    // Create action server (receives task scripts from clients)
    action_server_ = rclcpp_action::create_server<MTCExecution>(
        this, "mtc_execution",
        [this](const auto& uuid, const auto& goal) { return handle_goal(uuid, goal); },
        [this](const auto& goal_handle) { return handle_cancel(goal_handle); },
        [this](const auto& goal_handle) { handle_accepted(goal_handle); });

    // Create action clients (dispatch steps to specialized servers)
    moveto_action_client_ = rclcpp_action::create_client<MoveToAction>(this, "move_to_action");
    endeffector_action_client_ = rclcpp_action::create_client<EndEffectorAction>(this, "end_effector_action");
    toolexchange_action_client_ = rclcpp_action::create_client<ToolExchangeAction>(this, "tool_exchange_action");
    pickplace_action_client_ = rclcpp_action::create_client<PickPlaceAction>(this, "pick_place_action");
    vision_action_client_ = rclcpp_action::create_client<VisionMoveToAction>(this, "vision_move_to_action");
    pipettor_action_client_ = rclcpp_action::create_client<PipettorAction>(this, "pipettor_action");

    // Parameters
    this->declare_parameter("obstacle_config_path", "config/beamline_scene.yaml");

    RCLCPP_INFO(this->get_logger(), "MTC Orchestrator Action Server started");
}

MTCOrchestratorActionServer::~MTCOrchestratorActionServer()
{
    kill_moveit_process();
}

// ============================================================================
// ACTION SERVER CALLBACKS (Entry Points)
// ============================================================================

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
    // Execute in detached thread to avoid blocking the action server
    std::thread{[this, self = shared_from_this(), goal_handle]() {
        execute(goal_handle);
    }}.detach();
}

// ============================================================================
// TASK EXECUTION (Main Workflow)
// ============================================================================

void MTCOrchestratorActionServer::execute(
    const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    is_executing_ = true;

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MTCExecution::Feedback>();
    auto result = std::make_shared<MTCExecution::Result>();

    // --- Parse and validate input ---

    nlohmann::json full_script;
    try {
        full_script = nlohmann::json::parse(goal->full_json);
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid JSON: %s", e.what());
        result->success = false;
        result->error_message = std::string("Invalid JSON: ") + e.what();
        goal_handle->abort(result);
        is_executing_ = false;
        return;
    }

    if (goal->robot_ip.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Goal missing required robot_ip");
        result->success = false;
        result->error_message = "Goal missing required robot_ip";
        goal_handle->abort(result);
        is_executing_ = false;
        return;
    }

    if (!full_script.contains("start_gripper") || !full_script["start_gripper"].is_string()) {
        RCLCPP_ERROR(this->get_logger(), "Task script missing required start_gripper");
        result->success = false;
        result->error_message = "Task script missing required start_gripper";
        goal_handle->abort(result);
        is_executing_ = false;
        return;
    }

    const std::string start_gripper = full_script["start_gripper"].get<std::string>();
    const auto& tasks = full_script["tasks"];
    const std::string poses_json = full_script["poses"].dump();

    // --- Initialize MoveIt ---

    update_feedback(feedback, goal_handle, 0, tasks.size(), "Initializing MoveIt");

    if (!initialize_moveit_stack(start_gripper, goal->robot_ip)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt stack");
        result->success = false;
        result->error_message = "Failed to initialize MoveIt stack";
        goal_handle->abort(result);
        is_executing_ = false;
        return;
    }

    // --- Execute task steps ---

    for (size_t i = 0; i < tasks.size(); ++i) {
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->error_message = "Task was canceled";
            goal_handle->canceled(result);
            is_executing_ = false;
            return;
        }

        const auto& step = tasks[i];
        std::string task_type = step.value("task_type", "");

        if (task_type.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Step missing 'task_type' field");
            result->success = false;
            result->error_message = "Step missing 'task_type' field";
            result->completed_steps = i;
            goal_handle->abort(result);
            is_executing_ = false;
            return;
        }

        update_feedback(feedback, goal_handle, i + 1, tasks.size(), task_type);

        if (!execute_step(task_type, step, poses_json, goal->robot_ip)) {
            RCLCPP_ERROR(this->get_logger(), "%s step failed", task_type.c_str());
            result->success = false;
            result->error_message = task_type + " step failed";
            result->completed_steps = i;
            goal_handle->abort(result);
            is_executing_ = false;
            return;
        }

        result->completed_steps = i + 1;
    }

    // --- Success ---

    result->success = true;
    result->total_steps = tasks.size();
    update_feedback(feedback, goal_handle, tasks.size(), tasks.size(), "");
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded - keeping MoveIt running");
    is_executing_ = false;
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

// ============================================================================
// MOVEIT PROCESS MANAGEMENT
// ============================================================================

bool MTCOrchestratorActionServer::initialize_moveit_stack(
    const std::string& gripper,
    const std::string& robot_ip)
{
    // Reuse existing MoveIt if same gripper
    if (moveit_pid_ > 0 && current_gripper_ == gripper) {
        RCLCPP_INFO(this->get_logger(), "MoveIt already running for %s, reusing", gripper.c_str());
        return true;
    }

    // Kill existing MoveIt if different gripper
    if (moveit_pid_ > 0) {
        RCLCPP_INFO(this->get_logger(), "Switching gripper: %s → %s",
                    current_gripper_.c_str(), gripper.c_str());
        kill_moveit_process();
    }

    // Gripper → MoveIt config package mapping
    static const std::unordered_map<std::string, std::string> gripper_packages = {
        {"none",     "ur_standalone_moveit_config"},
        {"epick",    "ur_zivid_epick_moveit_config"},
        {"hande",    "ur_zivid_hande_moveit_config"},
        {"pipettor", "ur_zivid_pipettor_moveit_config"}
    };

    // Gripper → tool voltage mapping
    static const std::unordered_map<std::string, int> gripper_voltages = {
        {"none", 0}, {"epick", 24}, {"hande", 24}, {"pipettor", 24}
    };

    // Step 1: Set tool voltage (must happen BEFORE MoveIt launches)
    int voltage = gripper_voltages.at(gripper);
    RCLCPP_INFO(this->get_logger(), "Setting tool voltage: %dV", voltage);
    if (!set_tool_voltage_via_socket(robot_ip, voltage)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set tool voltage");
        return false;
    }

    // Step 2: Launch MoveIt
    RCLCPP_INFO(this->get_logger(), "Launching MoveIt for %s gripper", gripper.c_str());
    std::string launch_cmd = "ros2 launch " + gripper_packages.at(gripper) +
                             " robot_bringup.launch.py robot_ip:=" + robot_ip;
    launch_moveit_process(launch_cmd);

    // Step 3: Wait for MoveIt to be ready
    auto plan_client = this->create_client<moveit_msgs::srv::GetMotionPlan>("/plan_kinematic_path");
    if (!plan_client->wait_for_service(30s)) {
        RCLCPP_ERROR(this->get_logger(), "MoveIt planning service not ready within 30s");
        kill_moveit_process();
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "MoveIt ready");

    // Step 4: Load collision obstacles (REQUIRED for safety)
    std::string config_file = this->get_parameter("obstacle_config_path").as_string();
    if (!config_file.empty() && config_file[0] != '/') {
        try {
            config_file = ament_index_cpp::get_package_share_directory("mtc_pipeline") + "/" + config_file;
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Failed to resolve obstacle config path");
            kill_moveit_process();
            return false;
        }
    }
    if (config_file.empty() || !mtc_pipeline::loadPlanningSceneObstacles(this->get_logger(), config_file)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load obstacles - aborting for safety");
        kill_moveit_process();
        return false;
    }

    // Step 5: Restart UR external_control program (voltage command stops it)
    auto dashboard = this->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
    dashboard->wait_for_service(30s);
    dashboard->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    current_gripper_ = gripper;
    RCLCPP_INFO(this->get_logger(), "Robot ready with %s configuration", gripper.c_str());
    return true;
}

pid_t MTCOrchestratorActionServer::launch_moveit_process(const std::string& command)
{
    pid_t pid = fork();

    if (pid == 0) {
        // Child: create new process group and exec
        setsid();
        execl("/bin/bash", "bash", "-c", command.c_str(), static_cast<char*>(nullptr));
        _exit(1);
    }

    if (pid > 0) {
        moveit_pid_ = pid;
    }

    return pid;
}

void MTCOrchestratorActionServer::kill_moveit_process()
{
    if (moveit_pid_ <= 0) return;

    // Send SIGTERM to process group
    kill(-moveit_pid_, SIGTERM);

    // Wait up to 2s for graceful exit
    auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline && kill(moveit_pid_, 0) == 0) {
        std::this_thread::sleep_for(50ms);
    }

    // Force kill if still alive
    if (kill(moveit_pid_, 0) == 0) {
        kill(-moveit_pid_, SIGKILL);
    }

    // Wait for process to fully exit
    int status;
    waitpid(moveit_pid_, &status, 0);
    moveit_pid_ = 0;
}

bool MTCOrchestratorActionServer::set_tool_voltage_via_socket(
    const std::string& robot_ip,
    int voltage)
{
    // Uses raw socket because this runs BEFORE MoveIt/ROS services are available

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
        return false;
    }

    // 2-second timeout
    struct timeval timeout = {2, 0};
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    // Connect to UR secondary interface (port 30002)
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(30002);
    inet_pton(AF_INET, robot_ip.c_str(), &addr.sin_addr);

    if (connect(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(sockfd);
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to %s:30002", robot_ip.c_str());
        return false;
    }

    // Send URScript command
    std::string cmd = "set_tool_voltage(" + std::to_string(voltage) + ")\n";
    bool success = send(sockfd, cmd.c_str(), cmd.length(), 0) > 0;
    close(sockfd);

    return success;
}

// ============================================================================
// ACTION CLIENT CALLS
// ============================================================================

// --- Helper: Send goal and wait for result ---

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

// --- Basic movement actions ---

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

// --- Complex manipulation actions ---

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

// --- Tool exchange (special: requires MoveIt restart) ---

bool MTCOrchestratorActionServer::call_toolexchange_action(
    const nlohmann::json& step,
    const std::string& poses_json)
{
    ToolExchangeAction::Goal goal;
    goal.operation = step.value("operation", "");
    goal.gripper = step.value("gripper", "");
    goal.current_attached_gripper = current_gripper_;
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
    // First execute the physical tool exchange motion
    if (!call_toolexchange_action(step, poses_json)) {
        return false;
    }

    // Then restart MoveIt with new gripper configuration
    const std::string operation = step.value("operation", "");

    if (operation == "dock") {
        return initialize_moveit_stack("none", robot_ip);
    }
    if (operation == "load") {
        return initialize_moveit_stack(step.value("gripper", current_gripper_), robot_ip);
    }

    return true;
}

// ============================================================================
// UTILITIES
// ============================================================================

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
    feedback->current_gripper = current_gripper_.empty() ? "none" : current_gripper_;

    goal_handle->publish_feedback(feedback);
}

// ============================================================================
// MAIN
// ============================================================================

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
