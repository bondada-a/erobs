#include "mtc_pipeline/mtc_orchestrator_action_server.hpp"

namespace {
    // Wait for ROS2 service to become available
    bool wait_for_service(rclcpp::Node::SharedPtr node, const std::string& service_name, std::chrono::seconds timeout) {
        auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
        return client->wait_for_service(timeout);
    }


    // Wait for MoveIt stack to be ready - Simple, reliable approach
    bool wait_for_moveit_ready(rclcpp::Node::SharedPtr node, std::chrono::seconds timeout = 30s) {
        RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt to become ready...");

        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < timeout) {
            // Check if move_group node exists - this indicates MoveIt is fully ready
            auto node_names = node->get_node_names();
            bool move_group_found = std::any_of(node_names.begin(), node_names.end(),
                [](const std::string& name) { return name.find("move_group") != std::string::npos; });

            if (move_group_found) {
                RCLCPP_INFO(node->get_logger(), "MoveIt is ready (move_group node detected)");
                return true;
            }

            std::this_thread::sleep_for(100ms);
        }

        RCLCPP_ERROR(node->get_logger(), "MoveIt failed to become ready within timeout!");
        return false;
    }


    // Copy robot description parameters for orchestrator
    bool update_robot_description_from(const std::string& source_node, rclcpp::Node::SharedPtr node) {
        RCLCPP_INFO(node->get_logger(), "Attempting to get robot description from %s", source_node.c_str());
        
        // First, wait for the node to be available
        auto start_time = std::chrono::steady_clock::now();
        const auto node_timeout = std::chrono::seconds(30);
        
        while (std::chrono::steady_clock::now() - start_time < node_timeout) {
            auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, source_node);
            if (client->wait_for_service(2s)) {
                RCLCPP_INFO(node->get_logger(), "Parameter service for %s is available", source_node.c_str());
                break;
            }
            RCLCPP_INFO(node->get_logger(), "Waiting for parameter service of %s to become available...", source_node.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, source_node);
        if (!client->wait_for_service(5s)) {
            RCLCPP_ERROR(node->get_logger(), "Could not contact parameter service of %s", source_node.c_str());
            return false;
        }

        // Now wait for the parameters to be declared and available
        start_time = std::chrono::steady_clock::now();
        const auto param_timeout = std::chrono::seconds(30);
        
        while (std::chrono::steady_clock::now() - start_time < param_timeout) {
            try {
                RCLCPP_INFO(node->get_logger(), "Getting robot and OMPL parameters from %s", source_node.c_str());
                auto urdf_future = client->get_parameters({"robot_description"});
                auto srdf_future = client->get_parameters({"robot_description_semantic"});
                auto ompl_plugin_future = client->get_parameters({"ompl.planning_plugin"});
                auto ompl_adapters_future = client->get_parameters({"ompl.request_adapters"});
                
                // Wait for both futures to complete
                auto future_start_time = std::chrono::steady_clock::now();
                const auto future_timeout = std::chrono::seconds(10);
                
                while (std::chrono::steady_clock::now() - future_start_time < future_timeout) {
                    if (urdf_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready &&
                        srdf_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready &&
                        ompl_plugin_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready &&
                        ompl_adapters_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                        
                        auto urdf_params = urdf_future.get();
                        auto srdf_params = srdf_future.get();
                        auto ompl_plugin_params = ompl_plugin_future.get();
                        auto ompl_adapters_params = ompl_adapters_future.get();
                        
                        if (urdf_params.size() > 0 && srdf_params.size() > 0) {
                            // Check if parameters are not empty strings
                            std::string urdf_value = urdf_params[0].as_string();
                            std::string srdf_value = srdf_params[0].as_string();
                            
                            if (!urdf_value.empty() && !srdf_value.empty()) {
                                // Set robot description parameters
                                node->set_parameters({
                                    {"robot_description", urdf_value}, 
                                    {"robot_description_semantic", srdf_value}
                                });
                                
                                // Set OMPL parameters if available
                                if (ompl_plugin_params.size() > 0) {
                                    std::string ompl_plugin = ompl_plugin_params[0].as_string();
                                    if (!ompl_plugin.empty()) {
                                        node->set_parameter(rclcpp::Parameter("ompl.planning_plugin", ompl_plugin));
                                        RCLCPP_INFO(node->get_logger(), "Set ompl.planning_plugin: %s", ompl_plugin.c_str());
                                    }
                                }
                                
                                if (ompl_adapters_params.size() > 0) {
                                    std::string ompl_adapters = ompl_adapters_params[0].as_string();
                                    if (!ompl_adapters.empty()) {
                                        node->set_parameter(rclcpp::Parameter("ompl.request_adapters", ompl_adapters));
                                        RCLCPP_INFO(node->get_logger(), "Set ompl.request_adapters: %s", ompl_adapters.c_str());
                                    }
                                }
                                
                                RCLCPP_INFO(node->get_logger(), "Robot and OMPL params synced from [%s]", source_node.c_str());
                                return true;
                            } else {
                                RCLCPP_WARN(node->get_logger(), "Got empty parameter values from %s, retrying...", source_node.c_str());
                            }
                        } else {
                            RCLCPP_WARN(node->get_logger(), "Got empty parameter list from %s, retrying...", source_node.c_str());
                        }
                        break; // Exit the inner while loop and retry from outer loop
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                
                RCLCPP_WARN(node->get_logger(), "Parameter futures not ready, retrying in 1 second...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
            } catch (const std::exception& e) {
                RCLCPP_WARN(node->get_logger(), "Exception while getting parameters from %s: %s, retrying...", source_node.c_str(), e.what());
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        
        RCLCPP_ERROR(node->get_logger(), "Timeout getting robot description from %s after %ld seconds", source_node.c_str(), 
                    std::chrono::duration_cast<std::chrono::seconds>(param_timeout).count());
        return false;
    }

    // Send play command to robot dashboard
    bool play_dashboard_client(rclcpp::Node::SharedPtr node) {
        RCLCPP_INFO(node->get_logger(), "Waiting for dashboard service...");
        if (!wait_for_service(node, "/dashboard_client/play", 30s)) {
            RCLCPP_ERROR(node->get_logger(), "Dashboard 'play' service not available!");
            return false;
        }
        
        auto client = node->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
        auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        
        // Wait for the future to complete without using spin_until_future_complete
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(5);
        
        while (std::chrono::steady_clock::now() - start_time < timeout) {
            if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                auto result = future.get();
                if (result->success) {
                    RCLCPP_INFO(node->get_logger(), "Dashboard 'play' called successfully.");
                    return true;
                } else {
                    RCLCPP_WARN(node->get_logger(), "Dashboard 'play' failed");
                    return false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        RCLCPP_WARN(node->get_logger(), "Dashboard 'play' timed out");
        return false;
    }

    // Get launch command for gripper type
    std::string launch_cmd_for_gripper(const std::string& g, const std::string& ip) {
        // Get the current working directory to find the workspace
        char cwd[PATH_MAX];
        if (getcwd(cwd, sizeof(cwd)) == nullptr) {
            return "";
        }
        std::string workspace_path = std::string(cwd);
        
        // Create the setup command that sources the workspace
        std::string setup_cmd = "source " + workspace_path + "/install/setup.bash && ";
        
        
        if (g == "none") return setup_cmd + "ros2 launch ur_standalone_moveit_config move_group.launch.py robot_ip:=" + ip;
        if (g == "epick") return setup_cmd + "ros2 launch ur_zivid_epick_moveit_config move_group.launch.py robot_ip:=" + ip;
        if (g == "hande") return setup_cmd + "ros2 launch ur_zivid_hande_moveit_config move_group.launch.py robot_ip:=" + ip;
        return "";
    }
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
            active_pids_.push_back(pid);
        }
        return pid;
    }

void Orchestrator::kill_all_and_wait() {
        for (pid_t pid : active_pids_)
            ::kill(-pid, SIGINT);
        
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(15);
        
        while (std::chrono::steady_clock::now() - start_time < timeout) {
            bool all_terminated = true;
            for (pid_t pid : active_pids_) {
                int status;
                if (waitpid(pid, &status, WNOHANG) == 0) {
                    all_terminated = false;
                    break;
                }
            }
            if (all_terminated) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        for (pid_t pid : active_pids_) {
            waitpid(pid, nullptr, WNOHANG);
        }
        active_pids_.clear();
    }

void Orchestrator::set_current_gripper(const std::string& g) { current_gripper_ = g; }
const std::string& Orchestrator::get_current_gripper() const { return current_gripper_; }

// MTCOrchestratorActionServer implementation
MTCOrchestratorActionServer::MTCOrchestratorActionServer(const rclcpp::NodeOptions& options) 
        : Node("mtc_orchestrator_action_server", options), is_executing_(false) {
        // Declare parameters only if they don't already exist (launch file compatibility)
        if (!this->has_parameter("robot_description")) {
            this->declare_parameter("robot_description", "");
        }
        if (!this->has_parameter("robot_description_semantic")) {
            this->declare_parameter("robot_description_semantic", "");
        }
        if (!this->has_parameter("robot_description_planning")) {
            this->declare_parameter("robot_description_planning", "");
        }
        if (!this->has_parameter("robot_description_kinematics")) {
            this->declare_parameter("robot_description_kinematics", "");
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
        
        RCLCPP_INFO(this->get_logger(), "Goal accepted");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

rclcpp_action::CancelResponse MTCOrchestratorActionServer::handle_cancel(
        const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        
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
        
        orchestrator_->kill_all_and_wait();
        orchestrator_->launch(launch_cmd_for_gripper(new_gripper, robot_ip));
        
        if (!wait_for_moveit_ready(this->shared_from_this(), 30s) ||
            !update_robot_description_from("move_group", this->shared_from_this()))
            return false;
        
        play_dashboard_client(this->shared_from_this()); 
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
    if (!moveto_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "MoveTo action server not available");
        return false;
    }
    
    auto goal = MoveToAction::Goal();
    goal.target_type = step.value("target_type", "");
    goal.target = step.value("target", "");
    goal.planning_type = step.value("planning_type", "joint");
    goal.direction = step.value("direction", "");
    goal.distance = step.value("distance", 0.0);
    goal.poses_json = poses.dump();
    
    auto future = moveto_action_client_->async_send_goal(goal);
    auto goal_handle = future.get();
    
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send MoveTo goal");
        return false;
    }
    
    auto result_future = moveto_action_client_->async_get_result(goal_handle);
    auto result = result_future.get();
    
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return result.result->success;
    }
    return false;
}

bool MTCOrchestratorActionServer::call_endeffector_action(const nlohmann::json& step, const nlohmann::json& poses) {
    if (!endeffector_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "EndEffector action server not available");
        return false;
    }
    
    auto goal = EndEffectorAction::Goal();
    goal.end_effector_type = step.value("end_effector_type", "");
    goal.end_effector_action = step.value("end_effector_action", "");
    goal.poses_json = poses.dump();
    
    auto future = endeffector_action_client_->async_send_goal(goal);
    auto goal_handle = future.get();
    
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send EndEffector goal");
        return false;
    }
    
    auto result_future = endeffector_action_client_->async_get_result(goal_handle);
    auto result = result_future.get();
    
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return result.result->success;
    }
    return false;
}

bool MTCOrchestratorActionServer::call_toolexchange_action(const nlohmann::json& step, const nlohmann::json& poses) {
    if (!toolexchange_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "ToolExchange action server not available");
        return false;
    }
    
    auto goal = ToolExchangeAction::Goal();
    goal.operation = step.value("operation", "");
    goal.gripper = step.value("gripper", "");
    goal.dock_number = step.value("dock_number", 0);
    // Note: approach_poses would need to be parsed from JSON array if present
    goal.poses_json = poses.dump();
    
    auto future = toolexchange_action_client_->async_send_goal(goal);
    auto goal_handle = future.get();
    
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send ToolExchange goal");
        return false;
    }
    
    auto result_future = toolexchange_action_client_->async_get_result(goal_handle);
    auto result = result_future.get();
    
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return result.result->success;
    }
    return false;
}

bool MTCOrchestratorActionServer::call_pickplace_action(const nlohmann::json& step, const nlohmann::json& poses) {
    if (!pickplace_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "PickPlace action server not available");
        return false;
    }
    
    auto goal = PickPlaceAction::Goal();
    goal.gripper = step.value("gripper", "");
    goal.pick_pose = step.value("pick_pose", "");
    goal.place_pose = step.value("place_pose", "");
    goal.planning_type = step.value("planning_type", "joint");
    goal.poses_json = poses.dump();
    
    auto future = pickplace_action_client_->async_send_goal(goal);
    auto goal_handle = future.get();
    
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send PickPlace goal");
        return false;
    }
    
    auto result_future = pickplace_action_client_->async_get_result(goal_handle);
    auto result = result_future.get();
    
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return result.result->success;
    }
    return false;
}



void MTCOrchestratorActionServer::execute(const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        is_executing_ = true;
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MTCExecution::Feedback>();
        auto result = std::make_shared<MTCExecution::Result>();

        try {
            // Parse the JSON task script
            nlohmann::json task_script;
            try {
                task_script = nlohmann::json::parse(goal->task_script_json);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
                result->success = false;
                result->error_message = "Failed to parse JSON task script";
                goal_handle->abort(result);
                is_executing_ = false;
                return;
            }

            // Get task parameters
            std::string robot_ip = goal->robot_ip.empty() ? "192.168.1.101" : goal->robot_ip;
            std::string start_gripper = task_script.value("start_gripper", "none");
            
            
            // Get tasks from JSON
            const auto& operations = task_script["tasks"];
            const auto& poses = task_script["poses"];

            result->total_steps = operations.size();
            result->completed_steps = 0;
            
            // Start MoveIt configuration
            RCLCPP_INFO(this->get_logger(), "Starting MoveIt configuration for gripper: %s", start_gripper.c_str());
            orchestrator_->kill_all_and_wait();
            std::string launch_cmd = launch_cmd_for_gripper(start_gripper, robot_ip);
            RCLCPP_INFO(this->get_logger(), "Launch command: %s", launch_cmd.c_str());
            orchestrator_->launch(launch_cmd);
            
            // Send feedback
            feedback->current_step = 0;
            feedback->current_action = "Initializing MoveIt";
            feedback->progress_percentage = 0.0f;
            feedback->status_message = "Starting MoveIt configuration";
            feedback->current_gripper = start_gripper;
            goal_handle->publish_feedback(feedback);
            
            if (!wait_for_moveit_ready(this->shared_from_this(), 30s)) {
                throw std::runtime_error("Failed to initialize MoveIt stack");
            }
            
            // Wait for joint states to stabilize after controller initialization
            // Controllers are loaded automatically by launch files, but need brief time for state synchronization
            // This prevents position tolerance violations during first trajectory execution
            RCLCPP_INFO(this->get_logger(), "Allowing time for joint state synchronization...");
            std::this_thread::sleep_for(3s);
            
            play_dashboard_client(this->shared_from_this());
            
            if (!update_robot_description_from("move_group", this->shared_from_this())) {
                throw std::runtime_error("Failed to update robot description");
            }
            
            orchestrator_->set_current_gripper(start_gripper);


            // Execute tasks
            for (size_t i = 0; i < operations.size(); ++i) {
                // Check if goal was cancelled
                if (goal_handle->is_canceling()) {
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    result->success = false;
                    result->error_message = "Task was canceled";
                    goal_handle->canceled(result);
                    is_executing_ = false;
                    return;
                }

                const auto& step = operations[i];
                const std::string action = step.contains("type") ? step["type"].get<std::string>() : step["action"].get<std::string>();

                // Update feedback
                feedback->current_step = i + 1;
                feedback->current_action = action;
                feedback->progress_percentage = static_cast<float>(i + 1) / operations.size() * 100.0f;
                feedback->status_message = "Executing: " + action;
                feedback->current_gripper = orchestrator_->get_current_gripper();
                goal_handle->publish_feedback(feedback);
                
                RCLCPP_INFO(this->get_logger(), "Executing step %zu: %s", i + 1, action.c_str());
                
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
        
        // Cleanup
        orchestrator_->kill_all_and_wait();
        
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
