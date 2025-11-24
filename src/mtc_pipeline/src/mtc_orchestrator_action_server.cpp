#include "mtc_pipeline/mtc_orchestrator_action_server.hpp"
#include "mtc_pipeline/obstacle_loader.hpp"

using namespace std::chrono_literals;

// Simple class to manage MoveIt processes
class SimpleProcessManager {
public:
  pid_t launch_process(const std::string& command) {
    pid_t pid = fork();

    if (pid == 0) {  // Child process
      setsid();  // Create new process group for clean termination
      execl("/bin/bash", "bash", "-c", command.c_str(), static_cast<char*>(nullptr));
      _exit(1);  // Exit if exec fails
    }

    if (pid > 0) {  // Parent process
      moveit_pid_ = pid;
    }

    return pid;
  }

  void kill_moveit_process() {
    if (moveit_pid_ > 0) {
      kill(-moveit_pid_, SIGTERM);  // Graceful termination of process group

      // Wait up to 3 seconds for graceful shutdown
      for (int i = 0; i < 30; i++) {
        if (kill(moveit_pid_, 0) != 0) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      kill(-moveit_pid_, SIGKILL);  // Force kill if still alive
      waitpid(moveit_pid_, nullptr, WNOHANG);  // Clean up zombie process
      moveit_pid_ = 0;
    }
  }

  std::string current_gripper_ = "";  // Active gripper configuration
  pid_t moveit_pid_ = 0;
};

// MTCOrchestratorActionServer implementation
MTCOrchestratorActionServer::MTCOrchestratorActionServer(const rclcpp::NodeOptions& options)
        : Node("mtc_orchestrator_action_server", options), is_executing_(false) {
        process_manager_ = std::make_unique<SimpleProcessManager>();

        // Initialize the action server
        this->action_server_ = rclcpp_action::create_server<MTCExecution>(
            this,
            "mtc_execution",
            [this](const auto& uuid, const auto& goal) { return handle_goal(uuid, goal); },
            [this](const auto& goal_handle) { return handle_cancel(goal_handle); },
            [this](const auto& goal_handle) { handle_accepted(goal_handle); });

        // Initialize action clients to call modular action servers
        moveto_action_client_ = rclcpp_action::create_client<MoveToAction>(this, "move_to_action");
        endeffector_action_client_ = rclcpp_action::create_client<EndEffectorAction>(this, "end_effector_action");
        toolexchange_action_client_ = rclcpp_action::create_client<ToolExchangeAction>(this, "tool_exchange_action");
        pickplace_action_client_ = rclcpp_action::create_client<PickPlaceAction>(this, "pick_place_action");
        vision_action_client_ = rclcpp_action::create_client<VisionMoveToAction>(this, "vision_move_to_action");
        pipettor_action_client_ = rclcpp_action::create_client<PipettorAction>(this, "pipettor_action");

        // Subscribe to tool data for voltage monitoring
        tool_data_sub_ = this->create_subscription<ur_msgs::msg::ToolDataMsg>(
            "/io_and_status_controller/tool_data", 10,
            std::bind(&MTCOrchestratorActionServer::tool_data_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MTC Orchestrator Action Server started");
    }

MTCOrchestratorActionServer::~MTCOrchestratorActionServer() {
    if (process_manager_) {
        process_manager_->kill_moveit_process();
    }
}

// === MAIN EXECUTION FLOW ===

void MTCOrchestratorActionServer::execute(const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    is_executing_ = true;

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MTCExecution::Feedback>();
    auto result = std::make_shared<MTCExecution::Result>();

    // Parse full JSON
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

    // Get task parameters
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

    // Send initial feedback
    update_feedback(feedback, goal_handle, 0, tasks.size(), "Initializing MoveIt");

    // Initialize MoveIt stack
    if (!initialize_moveit_stack(start_gripper, goal->robot_ip)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt stack");
        result->success = false;
        result->error_message = "Failed to initialize MoveIt stack";
        goal_handle->abort(result);
        is_executing_ = false;
        return;
    }

    // Execute tasks
    for (size_t i = 0; i < tasks.size(); ++i) {
        // Check if goal was cancelled
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

        // Update feedback
        update_feedback(feedback, goal_handle, i + 1, tasks.size(), task_type);

        // Execute step
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

    // Success
    result->success = true;
    result->error_message = "";
    result->total_steps = tasks.size();

    update_feedback(feedback, goal_handle, tasks.size(), tasks.size(), "");

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded - keeping MoveIt running");

    is_executing_ = false;
}

bool MTCOrchestratorActionServer::execute_step(const std::string& task_type, const nlohmann::json& step,
                 const std::string& poses_json, const std::string& robot_ip) {
    if (task_type == "tool_exchange") return handle_tool_exchange(step, poses_json, robot_ip);
    if (task_type == "pick_and_place") return call_pickplace_action(step, poses_json);
    if (task_type == "moveto") return call_moveto_action(step, poses_json);
    if (task_type == "end_effector") return call_endeffector_action(step, poses_json);
    if (task_type == "vision_moveto") return call_vision_action(step, poses_json);
    if (task_type == "pipettor") return call_pipettor_action(step, poses_json);

    return false;
}

// === ACTION SERVER HANDLERS (ORCHESTRATOR) ===

rclcpp_action::GoalResponse MTCOrchestratorActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MTCExecution::Goal> goal)
{
    if (is_executing_) {
        RCLCPP_WARN(this->get_logger(), "Goal rejected: another task is already executing");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MTCOrchestratorActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Cancel request received - will stop after current task completes");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MTCOrchestratorActionServer::handle_accepted(const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
{
    std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
}

// === MOVEIT STACK MANAGEMENT ===

bool MTCOrchestratorActionServer::set_tool_voltage_via_socket(const std::string& robot_ip, int voltage) {
    RCLCPP_INFO(this->get_logger(), "Setting tool voltage to %dV via direct socket connection", voltage);

    // Create socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create socket for voltage setting");
        return false;
    }

    // Set timeout to avoid hanging
    struct timeval timeout;
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    // Configure server address
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(30002);  // UR secondary client interface

    if (inet_pton(AF_INET, robot_ip.c_str(), &server_addr.sin_addr) <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid robot IP address: %s", robot_ip.c_str());
        close(sockfd);
        return false;
    }

    // Connect to robot
    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot at %s:30002", robot_ip.c_str());
        close(sockfd);
        return false;
    }

    // Send voltage command
    std::string command = "set_tool_voltage(" + std::to_string(voltage) + ")\n";
    ssize_t bytes_sent = send(sockfd, command.c_str(), command.length(), 0);

    close(sockfd);

    if (bytes_sent < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send voltage command");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Tool voltage command sent successfully: set_tool_voltage(%d)", voltage);

    // Small delay to let robot process command
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return true;
}

bool MTCOrchestratorActionServer::restart_robot_program() {
    RCLCPP_INFO(this->get_logger(), "Restarting robot program (required after URScript commands)");

    // Wait for robot to stabilize after voltage change
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Send play command to robot dashboard to restart external_control program
    auto client = this->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");

    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Dashboard play service not available");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Send request asynchronously without waiting (fire-and-forget)
    // We can't use spin_until_future_complete here because this node is already spinning
    // in the main executor, which would cause a deadlock
    client->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Dashboard play command sent, waiting for robot to restart...");

    // Wait for robot program to restart and be ready
    // This is a simple blocking wait - the dashboard service will process the request
    // and the robot will restart the external_control program
    std::this_thread::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(this->get_logger(), "Robot program restart complete");
    return true;
}

void MTCOrchestratorActionServer::tool_data_callback(const ur_msgs::msg::ToolDataMsg::SharedPtr msg) {
    // Update the current tool voltage reading
    current_tool_voltage_.store(msg->tool_output_voltage);
}

bool MTCOrchestratorActionServer::verify_tool_voltage(int expected_voltage) {
    RCLCPP_INFO(this->get_logger(), "Verifying tool voltage is %dV...", expected_voltage);

    // Wait a bit for voltage readings to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Get the current voltage (updated by the callback)
    double actual_voltage = current_tool_voltage_.load();

    // Check if we have valid readings
    if (actual_voltage < 0) {
        RCLCPP_ERROR(this->get_logger(), "No valid tool voltage reading available!");
        return false;
    }

    // Allow some tolerance (±2V)
    const double tolerance = 2.0;
    bool voltage_ok = std::abs(actual_voltage - expected_voltage) <= tolerance;

    if (voltage_ok) {
        RCLCPP_INFO(this->get_logger(), "✓ Tool voltage verified: %.1fV (expected %dV)",
                    actual_voltage, expected_voltage);
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "✗ CRITICAL: Tool voltage mismatch! Actual: %.1fV, Expected: %dV",
                     actual_voltage, expected_voltage);
        RCLCPP_ERROR(this->get_logger(),
                     "Stopping operation to prevent hardware damage!");
        return false;
    }
}

bool MTCOrchestratorActionServer::initialize_moveit_stack(const std::string& start_gripper, const std::string& robot_ip) {
    // Check if we already have the right gripper running
    if (process_manager_->moveit_pid_ > 0 && process_manager_->current_gripper_ == start_gripper) {
        RCLCPP_INFO(this->get_logger(), "MoveIt already running for gripper: %s, reusing", start_gripper.c_str());
        return true;  // Reuse existing MoveIt!
    }

    // Kill any existing MoveIt processes (different gripper or dead process)
    if (process_manager_->moveit_pid_ > 0) {
        RCLCPP_INFO(this->get_logger(), "Switching from %s to %s gripper",
                   process_manager_->current_gripper_.c_str(), start_gripper.c_str());
        process_manager_->kill_moveit_process();
    }

    // Map gripper types to MoveIt config packages                                          //TODO : Add gripper payload for each gripper
    static const std::unordered_map<std::string, std::string> gripper_packages = {
        {"none", "ur_standalone_moveit_config"},
        {"epick", "ur_zivid_epick_moveit_config"},
        {"hande", "ur_zivid_hande_moveit_config"},
        {"pipettor", "ur_zivid_pipettor_moveit_config"}
    };

    // Map gripper types to required tool voltages
    static const std::unordered_map<std::string, int> gripper_voltages = {
        {"none", 0},      // No gripper attached
        {"epick", 24},    // EPick vacuum gripper requires 24V
        {"hande", 24},    // Hand-E gripper requires 24V
        {"pipettor", 24}  // Pipettor tool requires 24V
    };

    // Set tool voltage BEFORE launching MoveIt (critical for gripper initialization)
    auto voltage_it = gripper_voltages.find(start_gripper);
    if (voltage_it != gripper_voltages.end()) {
        int required_voltage = voltage_it->second;
        RCLCPP_INFO(this->get_logger(), "Setting tool voltage to %dV for %s gripper (before MoveIt launch)",
                    required_voltage, start_gripper.c_str());

        if (!set_tool_voltage_via_socket(robot_ip, required_voltage)) {
            RCLCPP_WARN(this->get_logger(), "Failed to set tool voltage, continuing anyway...");
            // Don't fail the entire initialization - gripper might still work
        }
    }

    // Start MoveIt configuration
    RCLCPP_INFO(this->get_logger(), "Starting MoveIt configuration for gripper: %s", start_gripper.c_str());
    auto it = gripper_packages.find(start_gripper);
    const std::string launch_cmd = "ros2 launch " + it->second + " robot_bringup.launch.py robot_ip:=" + robot_ip;
    process_manager_->launch_process(launch_cmd);

    // Wait for planning service (loaded after OMPL pipeline initialization)
    auto plan_client = this->create_client<moveit_msgs::srv::GetMotionPlan>("/plan_kinematic_path");
    if (!plan_client->wait_for_service(30s)) {
        RCLCPP_ERROR(this->get_logger(), "MoveIt planning service not ready within 30s");
        process_manager_->kill_moveit_process();
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "MoveIt fully initialized and ready for planning");

    // Load planning scene obstacles
    std::string obstacle_config = "/root/ws/erobs/src/mtc_pipeline/config/beamline_scene.yaml";
    if (!mtc_pipeline::loadPlanningSceneObstacles(this->get_logger(), obstacle_config)) {
        RCLCPP_WARN(this->get_logger(), "Failed to load planning scene obstacles, continuing anyway");
    }

    // Wait for robot hardware to be ready
    RCLCPP_INFO(this->get_logger(), "Waiting for robot hardware to initialize...");
    std::this_thread::sleep_for(5s);

    // CRITICAL: First set the correct voltage BEFORE stopping/starting the program
    // This ensures the voltage is correct when the new program loads
    auto voltage_check_it = gripper_voltages.find(start_gripper);
    if (voltage_check_it != gripper_voltages.end()) {
        int expected_voltage = voltage_check_it->second;

        // Read current voltage
        double actual_voltage = current_tool_voltage_.load();
        RCLCPP_INFO(this->get_logger(), "Current voltage is %.1fV, need %dV for %s",
                    actual_voltage, expected_voltage, start_gripper.c_str());

        // Set voltage if different (do this BEFORE stopping the program)
        if (std::abs(actual_voltage - expected_voltage) > 1.0) {
            RCLCPP_INFO(this->get_logger(), "Setting tool voltage to %dV before program restart",
                        expected_voltage);
            if (!set_tool_voltage_via_socket(robot_ip, expected_voltage)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set tool voltage to %dV", expected_voltage);
                process_manager_->kill_moveit_process();
                return false;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // Now stop any running program to force reload
    auto stop_client = this->create_client<std_srvs::srv::Trigger>("/dashboard_client/stop");
    if (stop_client->wait_for_service(5s)) {
        stop_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        RCLCPP_INFO(this->get_logger(), "Stopped existing program to force reload");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Send play command to robot dashboard - this will load and start the NEW program
    auto client = this->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
    client->wait_for_service(30s);
    client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    // CRITICAL: Wait for robot program to fully restart after dashboard play
    // Without this wait, motion might start before program is ready, causing voltage spikes
    RCLCPP_INFO(this->get_logger(), "Waiting for robot program to stabilize after restart...");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // CHECK VOLTAGE AFTER DASHBOARD PLAY
    // CRITICAL: We cannot send URScript commands after dashboard play as it kills external_control
    // We must rely on the tool_voltage parameter being correctly applied by ur_robot_driver
    RCLCPP_INFO(this->get_logger(), "Checking voltage AFTER dashboard play...");
    if (voltage_check_it != gripper_voltages.end()) {
        int expected_voltage = voltage_check_it->second;

        // Wait a bit for voltage to stabilize after program load
        std::this_thread::sleep_for(std::chrono::seconds(2));

        double actual_voltage = current_tool_voltage_.load();
        RCLCPP_INFO(this->get_logger(), "AFTER dashboard play: Current voltage is %.1fV (expecting %dV)",
                    actual_voltage, expected_voltage);

        if (std::abs(actual_voltage - expected_voltage) > 1.0) {
            RCLCPP_ERROR(this->get_logger(),
                        "CRITICAL: Voltage mismatch after dashboard play! Expected %dV but got %.1fV",
                        expected_voltage, actual_voltage);
            RCLCPP_ERROR(this->get_logger(),
                        "The tool_voltage parameter may not be correctly set in the launch file for %s",
                        start_gripper.c_str());
            RCLCPP_ERROR(this->get_logger(), "Killing MoveIt process due to voltage mismatch");
            process_manager_->kill_moveit_process();
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "✓ Tool voltage is correct at %.1fV for %s configuration",
                    actual_voltage, start_gripper.c_str());
    }

    process_manager_->current_gripper_ = start_gripper;
    RCLCPP_INFO(this->get_logger(), "Robot ready with %s configuration", start_gripper.c_str());
    return true;
}

// === ACTION CLIENT METHODS ===

template<typename ActionType>
bool MTCOrchestratorActionServer::call_action_generic(
    typename rclcpp_action::Client<ActionType>::SharedPtr client,
    const std::string& task_type,
    const nlohmann::json& step,
    const std::string& poses_json,
    std::function<void(typename ActionType::Goal&, const nlohmann::json&, const std::string&)> populate_goal
) {
    // Check if action server is available
    if (!client->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(), "%s action server unavailable", task_type.c_str());
        return false;
    }

    // Create and populate goal using lambda function
    auto goal = typename ActionType::Goal();
    populate_goal(goal, step, poses_json);

    // Send goal and get handle
    auto future = client->async_send_goal(goal);
    auto goal_handle = future.get();

    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send %s goal", task_type.c_str());
        return false;
    }

    // Wait for result with timeout
    auto result_future = client->async_get_result(goal_handle);
    if (result_future.wait_for(120s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "%s action timed out after 120 seconds", task_type.c_str());
        client->async_cancel_goal(goal_handle);
        return false;
    }

    // Check if action succeeded
    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return result.result->success;
    }
    return false;
}

bool MTCOrchestratorActionServer::call_moveto_action(const nlohmann::json& step, const std::string& poses_json) {
    return call_action_generic<MoveToAction>(moveto_action_client_, "moveto", step, poses_json, [](MoveToAction::Goal& goal, const nlohmann::json& step, const std::string& poses_json) {
        goal.target = step.value("target", "");
        goal.planning_type = step.value("planning_type", "joint");
        goal.direction = step.value("direction", "");
        goal.distance = step.value("distance", 0.0);
        goal.poses_json = poses_json;
    });
}

bool MTCOrchestratorActionServer::call_endeffector_action(const nlohmann::json& step, const std::string& poses_json) {
    return call_action_generic<EndEffectorAction>(endeffector_action_client_, "end_effector", step, poses_json, [](EndEffectorAction::Goal& goal, const nlohmann::json& step, const std::string& poses_json) {
        goal.end_effector_type = step.value("end_effector_type", "");
        goal.end_effector_action = step.value("end_effector_action", "");
        goal.poses_json = poses_json;
    });
}

bool MTCOrchestratorActionServer::call_pickplace_action(const nlohmann::json& step, const std::string& poses_json) {
    return call_action_generic<PickPlaceAction>(pickplace_action_client_, "pick_and_place", step, poses_json, [](PickPlaceAction::Goal& goal, const nlohmann::json& step, const std::string& poses_json) {
        goal.gripper = step.value("gripper", "");
        goal.pick_approach = step.value("pick_approach", "");
        goal.pick_target = step.value("pick_target", "");
        goal.place_approach = step.value("place_approach", "");
        goal.place_target = step.value("place_target", "");
        goal.poses_json = poses_json;
    });
}

bool MTCOrchestratorActionServer::call_vision_action(const nlohmann::json& step, const std::string& poses_json) {
    return call_action_generic<VisionMoveToAction>(vision_action_client_, "vision_moveto", step, poses_json, [](VisionMoveToAction::Goal& goal, const nlohmann::json& step, const std::string& poses_json) {
        goal.tag_id = step.value("tag_id", 0);
        goal.timeout = step.value("timeout", 5.0);
        goal.poses_json = poses_json;
    });
}

bool MTCOrchestratorActionServer::call_pipettor_action(const nlohmann::json& step, const std::string& poses_json) {
    return call_action_generic<PipettorAction>(pipettor_action_client_, "pipettor", step, poses_json, [](PipettorAction::Goal& goal, const nlohmann::json& step, const std::string& poses_json) {
        goal.operation = step.value("operation", "");
        goal.volume_pct = step.value("volume_pct", 0.0);
        goal.poses_json = poses_json;
        // LED color if specified
        if (step.contains("led_color")) {
            goal.led_color.r = step["led_color"].value("r", 0.0);
            goal.led_color.g = step["led_color"].value("g", 0.0);
            goal.led_color.b = step["led_color"].value("b", 0.0);
            goal.led_color.a = step["led_color"].value("a", 1.0);
        }
    });
}

bool MTCOrchestratorActionServer::handle_tool_exchange(const nlohmann::json& step, const std::string& poses_json, const std::string& robot_ip) {
    const std::string operation = step.value("operation", "");
    const std::string requested_tool = step.value("gripper", process_manager_->current_gripper_);

    // For LOAD operation: Voltage should ALREADY be 0V from previous dock/standalone initialization
    // DO NOT send voltage commands here - they stop the running program and create race conditions
    if (operation == "load") {
        RCLCPP_INFO(this->get_logger(), "Loading %s - voltage should already be 0V from previous standalone mode",
                    requested_tool.c_str());

        // CRITICAL: Verify voltage is 0V before attempting to attach tool
        // This prevents hardware damage from attaching with voltage present
        if (!verify_tool_voltage(0)) {
            RCLCPP_ERROR(this->get_logger(),
                        "CRITICAL: Cannot load %s - voltage is not 0V! Aborting to prevent damage!",
                        requested_tool.c_str());
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Voltage confirmed at 0V, safe to proceed with tool attachment");
    }

    // For DOCK operation: Keep voltage ON during detachment (gripper controller needs power)
    // Voltage will be set to 0V AFTER detachment in initialize_moveit_stack("none")
    if (operation == "dock") {
        // Verify voltage is appropriate for the currently attached gripper
        static const std::unordered_map<std::string, int> gripper_voltages = {
            {"none", 0}, {"epick", 24}, {"hande", 24}, {"pipettor", 24}
        };

        auto voltage_it = gripper_voltages.find(process_manager_->current_gripper_);
        if (voltage_it != gripper_voltages.end()) {
            int expected_voltage = voltage_it->second;
            if (!verify_tool_voltage(expected_voltage)) {
                RCLCPP_ERROR(this->get_logger(),
                            "CRITICAL: Cannot dock %s - voltage is not %dV! Current gripper may lose power!",
                            process_manager_->current_gripper_.c_str(), expected_voltage);
                return false;
            }
        }
    }

    // Execute tool exchange action (physical motion)
    if (!call_toolexchange_action(step, poses_json)) {
        return false;
    }

    // Handle gripper switching after tool exchange
    if (operation == "dock") {
        // After docking, switch to standalone mode (this will set voltage to 0V)
        return initialize_moveit_stack("none", robot_ip);
    } else if (operation == "load") {
        // After loading, power up the tool to its required voltage (e.g., 24V for pipettor)
        return initialize_moveit_stack(requested_tool, robot_ip);
    }
    return true;
}

bool MTCOrchestratorActionServer::call_toolexchange_action(const nlohmann::json& step, const std::string& poses_json) {
    return call_action_generic<ToolExchangeAction>(toolexchange_action_client_, "tool_exchange", step, poses_json, [this](ToolExchangeAction::Goal& goal, const nlohmann::json& step, const std::string& poses_json) {
        goal.operation = step.value("operation", "");
        goal.gripper = step.value("gripper", "");
        goal.current_attached_gripper = process_manager_->current_gripper_;
        goal.dock_number = step.value("dock_number", 0);
        goal.approach_pose = step.value("approach_pose", "");
        goal.poses_json = poses_json;
    });
}



// === UTILITY FUNCTIONS ===

void MTCOrchestratorActionServer::update_feedback(std::shared_ptr<MTCExecution::Feedback> feedback,
                    std::shared_ptr<GoalHandleMTCExecution> goal_handle,
                    size_t current_step, size_t total_steps, const std::string& task_type) {
    feedback->current_step = current_step;
    feedback->current_action = task_type;
    feedback->progress_percentage = static_cast<float>(current_step) / total_steps * 100.0f;
    feedback->status_message = task_type.empty() ? "Task completed successfully" : "Executing: " + task_type;
    feedback->current_gripper = process_manager_ ? process_manager_->current_gripper_ : std::string("none");
    goal_handle->publish_feedback(feedback);
}


// === MAIN ===

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
