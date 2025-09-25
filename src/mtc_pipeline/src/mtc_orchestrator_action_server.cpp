#include "mtc_pipeline/mtc_orchestrator_action_server.hpp"

namespace {

    // Secure command parsing - validates and sanitizes launch commands
    bool validate_launch_command(const std::string& cmd, std::string& package, std::string& launch_file, std::string& robot_ip) {
        // Expected format: "source /path/install/setup.bash && ros2 launch PACKAGE LAUNCH_FILE robot_ip:=IP"

        // Find the ros2 launch part
        size_t ros2_pos = cmd.find("ros2 launch ");
        if (ros2_pos == std::string::npos) {
            return false;
        }

        std::string launch_part = cmd.substr(ros2_pos + 12); // Skip "ros2 launch "
        std::istringstream iss(launch_part);
        std::string token;

        // Extract package name
        if (!(iss >> package) || package.find("moveit_config") == std::string::npos) {
            return false; // Invalid package
        }

        // Extract launch file
        if (!(iss >> launch_file) || launch_file.find(".launch.py") == std::string::npos) {
            return false; // Invalid launch file
        }

        // Extract robot_ip parameter
        std::string ip_param;
        while (iss >> token) {
            if (token.find("robot_ip:=") == 0) {
                robot_ip = token.substr(10);
                break;
            }
        }

        // Validate IP format (basic validation)
        if (robot_ip.empty()) {
            return false;
        }

        return true; // Command structure is valid
    }
}

// Manages MoveIt configuration processes - Secure version
pid_t Orchestrator::launch(const std::string& cmd) {
        // Validate and parse command securely
        std::string package, launch_file, robot_ip;
        if (!validate_launch_command(cmd, package, launch_file, robot_ip)) {
            return -1; // Invalid command - reject
        }

        pid_t pid = fork();
        if (pid == 0) {
            // Child process - execute ros2 launch directly (no shell)
            if (setsid() == -1) {
                exit(1);
            }

            // Set up environment for ROS2 (equivalent to sourcing setup.bash)
            // The current environment should already have ROS2 sourced

            // Execute ros2 launch with validated parameters - NO SHELL
            execl("/opt/ros/humble/bin/ros2", "ros2", "launch",
                  package.c_str(), launch_file.c_str(),
                  ("robot_ip:=" + robot_ip).c_str(),
                  (char*)nullptr);

            exit(1); // Only reached if execl fails
        }

        if (pid > 0) {
            std::lock_guard<std::mutex> lock(pids_mutex_);
            active_pids_.push_back(pid);
        }
        return pid;
    }

void Orchestrator::kill_all_and_wait() {
        std::vector<pid_t> pids_copy;
        {
            std::lock_guard<std::mutex> lock(pids_mutex_);
            pids_copy = active_pids_;
        }

        for (pid_t pid : pids_copy)
            ::kill(-pid, SIGINT);

        auto start_time = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(15);

        while (std::chrono::steady_clock::now() - start_time < timeout) {
            bool all_terminated = true;
            for (pid_t pid : pids_copy) {
                int status;
                if (waitpid(pid, &status, WNOHANG) == 0) {
                    all_terminated = false;
                    break;
                }
            }
            if (all_terminated) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        for (pid_t pid : pids_copy) {
            waitpid(pid, nullptr, WNOHANG);
        }

        {
            std::lock_guard<std::mutex> lock(pids_mutex_);
            active_pids_.clear();
        }
    }

void Orchestrator::set_current_gripper(const std::string& g) { current_gripper_ = g; }
const std::string& Orchestrator::get_current_gripper() const { return current_gripper_; }

// MTCOrchestratorActionServer implementation
MTCOrchestratorActionServer::MTCOrchestratorActionServer(const rclcpp::NodeOptions& options) 
        : Node("mtc_orchestrator_action_server", options), is_executing_(false) {
        // Declare essential parameters only if they don't already exist (launch file compatibility)
        if (!this->has_parameter("robot_description")) {
            this->declare_parameter("robot_description", "");
        }
        if (!this->has_parameter("robot_description_semantic")) {
            this->declare_parameter("robot_description_semantic", "");
        }
        
        // Declare OMPL parameters only if they don't already exist
        if (!this->has_parameter("ompl.planning_plugin")) {
            this->declare_parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner");
        }
        if (!this->has_parameter("ompl.request_adapters")) {
            this->declare_parameter("ompl.request_adapters", "default_planner_request_adapters/AddTimeOptimalParameterization");
        }
        if (!this->has_parameter("ompl.path_tolerance")) {
            this->declare_parameter("ompl.path_tolerance", 0.1);
        }
        if (!this->has_parameter("ompl.resample_dt")) {
            this->declare_parameter("ompl.resample_dt", 0.1);
        }
        if (!this->has_parameter("ompl.min_angle_change")) {
            this->declare_parameter("ompl.min_angle_change", 0.001);
        }
        
        // Initialize orchestrator
        orchestrator_ = std::make_unique<Orchestrator>();
        
        // Initialize the action server
        this->action_server_ = rclcpp_action::create_server<MTCExecution>(
            this,
            "mtc_execution",
            std::bind(&MTCOrchestratorActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MTCOrchestratorActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MTCOrchestratorActionServer::handle_accepted, this, std::placeholders::_1));

        // Initialize action clients to call modular action servers
        moveto_action_client_ = rclcpp_action::create_client<MoveToAction>(this, "moveto_action");
        endeffector_action_client_ = rclcpp_action::create_client<EndEffectorAction>(this, "endeffector_action");
        toolexchange_action_client_ = rclcpp_action::create_client<ToolExchangeAction>(this, "toolexchange_action");
        pickplace_action_client_ = rclcpp_action::create_client<PickPlaceAction>(this, "pickplace_action");

        RCLCPP_INFO(this->get_logger(), "MTC Orchestrator Action Server started - using delegation to modular action servers");
    }


rclcpp_action::GoalResponse MTCOrchestratorActionServer::handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MTCExecution::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        
        if (is_executing_) {
            RCLCPP_WARN(this->get_logger(), "Goal rejected: another task is already executing");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Goal accepted");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

rclcpp_action::CancelResponse MTCOrchestratorActionServer::handle_cancel(
        const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_DEBUG(this->get_logger(), "Received request to cancel goal");
        
        if (is_executing_) {
            // Cancel the execution
            orchestrator_->kill_all_and_wait();
            is_executing_ = false;
        }
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }

void MTCOrchestratorActionServer::handle_accepted(const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
    {
        std::thread{std::bind(&MTCOrchestratorActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }






    // Switch to different gripper configuration
bool MTCOrchestratorActionServer::switch_gripper(const std::string& new_gripper, const std::string& robot_ip) {
        if (orchestrator_->get_current_gripper() == new_gripper) return true;

        // Just reuse the initialization logic - switching gripper IS reinitializing MoveIt
        if (!initialize_moveit_stack(new_gripper, robot_ip)) {
            return false;
        }

        orchestrator_->set_current_gripper(new_gripper);
        return true;
    }





// Execute a single task step
bool MTCOrchestratorActionServer::execute_step(const std::string& action, const nlohmann::json& step, 
                     const nlohmann::json& poses, const std::string& robot_ip) {
        
        // Handle tool exchange tasks - using delegation to modular action servers
        if (action == "tool_exchange") {
            const std::string operation = step.value("operation", "");
            const std::string requested_tool = step.value("gripper", orchestrator_->get_current_gripper());

            // Execute via delegation to modular action servers
            bool success = call_toolexchange_action(step, poses);
            if (!success) return false;

            // Handle gripper switching after tool exchange
            if (operation == "dock") {
                return switch_gripper("none", robot_ip);
            } else if (operation == "load") {
                return switch_gripper(requested_tool, robot_ip);
            }
            return true;
        }

        // Handle pick and place tasks - using delegation to modular action servers
        if (action == "pick_and_place") {
            std::string need = step.value("gripper", orchestrator_->get_current_gripper());
            if (!switch_gripper(need, robot_ip))
                return false;

            return call_pickplace_action(step, poses);
        }

        // Handle simple move-to tasks - using delegation to modular action servers
        if (action == "moveto") {
            return call_moveto_action(step, poses);
        }

        // Handle end effector operations - using delegation to modular action servers
        if (action == "end_effector") {
            return call_endeffector_action(step, poses);
        }

    return false;
}

// Action client methods to call embedded actions via ROS2 actions
bool MTCOrchestratorActionServer::call_moveto_action(const nlohmann::json& step, const nlohmann::json& poses) {
    return call_action_generic<MoveToAction>(
        moveto_action_client_,
        "MoveTo",
        step,
        poses,
        [](MoveToAction::Goal& goal, const nlohmann::json& step, const nlohmann::json& poses) {
            goal.target_type = step.value("target_type", "");
            goal.target = step.value("target", "");
            goal.planning_type = step.value("planning_type", "joint");
            goal.direction = step.value("direction", "");
            goal.distance = step.value("distance", 0.0);
            goal.poses_json = poses.dump();
        }
    );
}

bool MTCOrchestratorActionServer::call_endeffector_action(const nlohmann::json& step, const nlohmann::json& poses) {
    return call_action_generic<EndEffectorAction>(
        endeffector_action_client_,
        "EndEffector",
        step,
        poses,
        [](EndEffectorAction::Goal& goal, const nlohmann::json& step, const nlohmann::json& poses) {
            goal.end_effector_type = step.value("end_effector_type", "");
            goal.end_effector_action = step.value("end_effector_action", "");
            goal.poses_json = poses.dump();
        }
    );
}

bool MTCOrchestratorActionServer::call_toolexchange_action(const nlohmann::json& step, const nlohmann::json& poses) {
    return call_action_generic<ToolExchangeAction>(
        toolexchange_action_client_,
        "ToolExchange",
        step,
        poses,
        [](ToolExchangeAction::Goal& goal, const nlohmann::json& step, const nlohmann::json& poses) {
            goal.operation = step.value("operation", "");
            goal.gripper = step.value("gripper", "");
            goal.dock_number = step.value("dock_number", 0);
            // Note: approach_poses would need to be parsed from JSON array if present
            goal.poses_json = poses.dump();
        }
    );
}

bool MTCOrchestratorActionServer::call_pickplace_action(const nlohmann::json& step, const nlohmann::json& poses) {
    return call_action_generic<PickPlaceAction>(
        pickplace_action_client_,
        "PickPlace",
        step,
        poses,
        [](PickPlaceAction::Goal& goal, const nlohmann::json& step, const nlohmann::json& poses) {
            goal.gripper = step.value("gripper", "");
            goal.pick_pose = step.value("pick_pose", "");
            goal.place_pose = step.value("place_pose", "");
            goal.planning_type = step.value("planning_type", "joint");
            goal.poses_json = poses.dump();
        }
    );
}

bool MTCOrchestratorActionServer::parse_and_validate_task_script(const std::string& json_str, nlohmann::json& task_script) {
    try {
        task_script = nlohmann::json::parse(json_str);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
        return false;
    }

    // Validate required fields exist
    if (!task_script.contains("tasks") || !task_script.contains("poses")) {
        RCLCPP_ERROR(this->get_logger(), "JSON missing required 'tasks' or 'poses' fields");
        return false;
    }

    // Validate field types
    if (!task_script["tasks"].is_array() || !task_script["poses"].is_object()) {
        RCLCPP_ERROR(this->get_logger(), "JSON field types invalid: 'tasks' must be array, 'poses' must be object");
        return false;
    }

    return true;
}

bool MTCOrchestratorActionServer::initialize_moveit_stack(const std::string& start_gripper, const std::string& robot_ip) {
    // Start MoveIt configuration
    RCLCPP_INFO(this->get_logger(), "Starting MoveIt configuration for gripper: %s", start_gripper.c_str());
    orchestrator_->kill_all_and_wait();

    // Map gripper types to MoveIt config packages
    static const std::unordered_map<std::string, std::string> gripper_packages = {
        {"none", "ur_standalone_moveit_config"},
        {"epick", "ur_zivid_epick_moveit_config"},
        {"hande", "ur_zivid_hande_moveit_config"}
    };

    auto it = gripper_packages.find(start_gripper);
    if (it == gripper_packages.end()) {
        RCLCPP_ERROR(this->get_logger(), "Unknown gripper type: %s", start_gripper.c_str());
        return false;
    }

    const std::string launch_cmd = "ros2 launch " + it->second + " move_group.launch.py robot_ip:=" + robot_ip;
    RCLCPP_DEBUG(this->get_logger(), "Launch command: %s", launch_cmd.c_str());
    orchestrator_->launch(launch_cmd);

    // Wait for MoveIt to become ready
    RCLCPP_DEBUG(this->get_logger(), "Waiting for MoveIt to become ready...");
    bool moveit_ready = false;
    while (!moveit_ready) {
        auto node_names = this->get_node_names();

        for (const auto& name : node_names) {
            if (name.find("move_group") != std::string::npos) {
                RCLCPP_DEBUG(this->get_logger(), "MoveIt is ready!");
                moveit_ready = true;
                break;
            }
        }

        if (!moveit_ready) {
            std::this_thread::sleep_for(500ms);
        }
    }

    // Wait for joint states to stabilize after controller initialization
    // Controllers are loaded automatically by launch files, but need brief time for state synchronization
    // This prevents position tolerance violations during first trajectory execution
    RCLCPP_DEBUG(this->get_logger(), "Allowing time for joint state synchronization...");
    std::this_thread::sleep_for(3s);

    // Send play command to robot dashboard
    auto client = this->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
    if (client->wait_for_service(30s)) {
        auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        auto result = future.get();  // Block until complete
        if (!result->success) {
            RCLCPP_WARN(this->get_logger(), "Dashboard 'play' failed");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Dashboard 'play' service not available");
    }


    orchestrator_->set_current_gripper(start_gripper);
    return true;
}

void MTCOrchestratorActionServer::execute(const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        is_executing_ = true;
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MTCExecution::Feedback>();
        auto result = std::make_shared<MTCExecution::Result>();

        try {
            // Parse and validate JSON task script
            nlohmann::json task_script;
            if (!parse_and_validate_task_script(goal->task_script_json, task_script)) {
                result->success = false;
                result->error_message = "Failed to parse or validate JSON task script";
                goal_handle->abort(result);
                is_executing_ = false;
                return;
            }

            // Get task parameters
            const std::string robot_ip = goal->robot_ip.empty() ? "192.168.1.101" : goal->robot_ip;
            const std::string start_gripper = task_script.value("start_gripper", "none");

            const auto& operations = task_script["tasks"];
            const auto& poses = task_script["poses"];

            result->total_steps = operations.size();
            result->completed_steps = 0;

            // Send initial feedback
            feedback->current_step = 0;
            feedback->current_action = "Initializing MoveIt";
            feedback->progress_percentage = 0.0f;
            feedback->status_message = "Starting MoveIt configuration";
            feedback->current_gripper = start_gripper;
            goal_handle->publish_feedback(feedback);

            // Initialize MoveIt stack
            if (!initialize_moveit_stack(start_gripper, robot_ip)) {
                throw std::runtime_error("Failed to initialize MoveIt stack");
            }


            // Execute tasks
            for (size_t i = 0; i < operations.size(); ++i) {
                // Check if goal was cancelled
                if (goal_handle->is_canceling()) {
                    RCLCPP_DEBUG(this->get_logger(), "Goal canceled");
                    result->success = false;
                    result->error_message = "Task was canceled";
                    goal_handle->canceled(result);
                    is_executing_ = false;
                    return;
                }

                const auto& step = operations[i];
                std::string action;
                try {
                    if (step.contains("type") && step["type"].is_string()) {
                        action = step["type"].get<std::string>();
                    } else if (step.contains("action") && step["action"].is_string()) {
                        action = step["action"].get<std::string>();
                    } else {
                        throw std::runtime_error("Step missing valid 'type' or 'action' field");
                    }
                } catch (const std::exception& e) {
                    throw std::runtime_error("Failed to read action from step " + std::to_string(i + 1) + ": " + e.what());
                }

                // Update feedback
                feedback->current_step = i + 1;
                feedback->current_action = action;
                feedback->progress_percentage = static_cast<float>(i + 1) / operations.size() * 100.0f;
                feedback->status_message = "Executing: " + action;
                feedback->current_gripper = orchestrator_->get_current_gripper();
                goal_handle->publish_feedback(feedback);
                
                RCLCPP_DEBUG(this->get_logger(), "Executing step %zu: %s", i + 1, action.c_str());
                
                // Execute step (stage parameters unused now - using embedded instances instead)
                if (!execute_step(action, step, poses, robot_ip)) {
                    RCLCPP_ERROR(this->get_logger(), "%s step failed", action.c_str());
                    result->success = false;
                    result->error_message = action + " step failed";
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
            
            feedback->progress_percentage = 100.0f;
            feedback->status_message = "Task completed successfully";
            goal_handle->publish_feedback(feedback);
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Execution failed: %s", e.what());
            result->success = false;
            result->error_message = std::string("Execution failed: ") + e.what();
            goal_handle->abort(result);
        }

        // Cleanup - always execute regardless of success or failure
        try {
            orchestrator_->kill_all_and_wait();
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Cleanup failed: %s", e.what());
        }

        is_executing_ = false;
    }

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
