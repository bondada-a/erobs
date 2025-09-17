#include "mtc_pipeline/mtc_orchestrator_action_server.hpp"

namespace {
    // Wait for ROS2 service to become available
    bool wait_for_service(rclcpp::Node::SharedPtr node, const std::string& service_name, std::chrono::seconds timeout) {
        auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
        return client->wait_for_service(timeout);
    }

    // Execute shell command and check if output contains expected string
    bool check_command_output(const std::string& cmd, const std::string& expected) {
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) return false; 
        char buf[128]; 
        bool found = false;
        while (fgets(buf, 128, pipe)) {
            if (std::string(buf).find(expected) != std::string::npos) {
                found = true;
                break;
            }
        }
        pclose(pipe);
        return found;
    }

    // Wait for MoveIt stack to be ready
    bool wait_for_moveit_ready(rclcpp::Node::SharedPtr node, std::chrono::seconds timeout = 30s) {
        RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt to become ready...");
        
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < timeout) {
            if (!check_command_output("ros2 node list", "move_group") ||
                !check_command_output("ros2 topic list | grep joint_states", "joint_states")) {
                std::this_thread::sleep_for(100ms);
                continue; 
            }
            
            // Wait a bit more for move_group to fully initialize
            std::this_thread::sleep_for(5s);
            
            // Now check if the parameters are available
            auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "move_group");
            if (client->wait_for_service(2s)) {
                try {
                    auto urdf_future = client->get_parameters({"robot_description"});
                    auto srdf_future = client->get_parameters({"robot_description_semantic"});
                    
                    // Wait for parameters to be available
                    auto param_start_time = std::chrono::steady_clock::now();
                    const auto param_timeout = std::chrono::seconds(10);
                    
                    while (std::chrono::steady_clock::now() - param_start_time < param_timeout) {
                        if (urdf_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready &&
                            srdf_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                            
                            auto urdf_params = urdf_future.get();
                            auto srdf_params = srdf_future.get();
                            
                            if (urdf_params.size() > 0 && srdf_params.size() > 0) {
                                std::string urdf_value = urdf_params[0].as_string();
                                std::string srdf_value = srdf_params[0].as_string();
                                
                                if (!urdf_value.empty() && !srdf_value.empty()) {
                                    RCLCPP_INFO(node->get_logger(), "MoveIt is ready with parameters!");
                                    return true;
                                }
                            }
                            break; // Exit inner loop and retry from outer loop
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                } catch (const std::exception& e) {
                    RCLCPP_WARN(node->get_logger(), "Exception checking parameters: %s, retrying...", e.what());
                }
            }
            
            std::this_thread::sleep_for(1s);
        }
        
        RCLCPP_ERROR(node->get_logger(), "MoveIt failed to become ready!");
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
}

// Manages MoveIt configuration processes
// Orchestrator class implementation
pid_t Orchestrator::launch(const std::string& cmd) {
        pid_t pid = fork(); 
        if (pid == 0) {
            execl("/usr/bin/setsid", "setsid", "bash", "-c", cmd.c_str(), (char*)nullptr);
            exit(1); 
        }
        active_pids_.push_back(pid);
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

        // Initialize embedded MoveTo action server
        moveto_action_server_ = rclcpp_action::create_server<MoveToAction>(
            this,
            "moveto_action",
            std::bind(&MTCOrchestratorActionServer::handle_moveto_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MTCOrchestratorActionServer::handle_moveto_cancel, this, std::placeholders::_1),
            std::bind(&MTCOrchestratorActionServer::handle_moveto_accepted, this, std::placeholders::_1));

        // Initialize embedded EndEffector action server
        endeffector_action_server_ = rclcpp_action::create_server<EndEffectorAction>(
            this,
            "endeffector_action",
            std::bind(&MTCOrchestratorActionServer::handle_endeffector_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MTCOrchestratorActionServer::handle_endeffector_cancel, this, std::placeholders::_1),
            std::bind(&MTCOrchestratorActionServer::handle_endeffector_accepted, this, std::placeholders::_1));

        // Initialize embedded ToolExchange action server
        toolexchange_action_server_ = rclcpp_action::create_server<ToolExchangeAction>(
            this,
            "toolexchange_action",
            std::bind(&MTCOrchestratorActionServer::handle_toolexchange_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MTCOrchestratorActionServer::handle_toolexchange_cancel, this, std::placeholders::_1),
            std::bind(&MTCOrchestratorActionServer::handle_toolexchange_accepted, this, std::placeholders::_1));

        // Initialize embedded PickPlace action server
        pickplace_action_server_ = rclcpp_action::create_server<PickPlaceAction>(
            this,
            "pickplace_action",
            std::bind(&MTCOrchestratorActionServer::handle_pickplace_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MTCOrchestratorActionServer::handle_pickplace_cancel, this, std::placeholders::_1),
            std::bind(&MTCOrchestratorActionServer::handle_pickplace_accepted, this, std::placeholders::_1));

        // Initialize action clients to call embedded actions
        moveto_action_client_ = rclcpp_action::create_client<MoveToAction>(this, "moveto_action");
        endeffector_action_client_ = rclcpp_action::create_client<EndEffectorAction>(this, "endeffector_action");
        toolexchange_action_client_ = rclcpp_action::create_client<ToolExchangeAction>(this, "toolexchange_action");
        pickplace_action_client_ = rclcpp_action::create_client<PickPlaceAction>(this, "pickplace_action");

        RCLCPP_INFO(this->get_logger(), "MTC Orchestrator with All Embedded Actions started");
    }

// Template implementations for simple action handlers
template<typename ActionType>
rclcpp_action::GoalResponse MTCOrchestratorActionServer::handle_simple_goal(
    const rclcpp_action::GoalUUID& uuid, 
    std::shared_ptr<const typename ActionType::Goal> goal,
    const std::string& action_name) {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received %s goal", action_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template<typename ActionType>
rclcpp_action::CancelResponse MTCOrchestratorActionServer::handle_simple_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle,
    std::atomic<bool>& abort_flag,
    const std::string& action_name) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "%s goal cancellation requested", action_name.c_str());
    abort_flag = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}

template<typename ActionType>
void MTCOrchestratorActionServer::handle_simple_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle,
    void (MTCOrchestratorActionServer::*execute_func)(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>>)) {
    std::thread{std::bind(execute_func, this, std::placeholders::_1), goal_handle}.detach();
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


    // ========== EMBEDDED MOVETO ACTION SERVER HANDLERS ==========
    
rclcpp_action::GoalResponse MTCOrchestratorActionServer::handle_moveto_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveToAction::Goal> goal)
    {
        return handle_simple_goal<MoveToAction>(uuid, goal, "MoveTo");
    }

rclcpp_action::CancelResponse MTCOrchestratorActionServer::handle_moveto_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToAction>> goal_handle)
    {
        return handle_simple_cancel<MoveToAction>(goal_handle, moveto_abort_requested_, "MoveTo");
    }

void MTCOrchestratorActionServer::handle_moveto_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToAction>> goal_handle)
    {
        handle_simple_accepted<MoveToAction>(goal_handle, &MTCOrchestratorActionServer::execute_moveto_embedded);
    }

void MTCOrchestratorActionServer::execute_moveto_embedded(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToAction>> goal_handle)
    {
        moveto_abort_requested_ = false;
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveToAction::Feedback>();
        auto result = std::make_shared<MoveToAction::Result>();
        
        try {
            // Parse poses JSON
            nlohmann::json poses;
            try {
                poses = nlohmann::json::parse(goal->poses_json);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse poses JSON: %s", e.what());
                result->success = false;
                result->error_message = "Invalid poses JSON";
                goal_handle->abort(result);
                return;
            }

            // Create step JSON from goal
            nlohmann::json step;
            step["target_type"] = goal->target_type;
            step["target"] = goal->target;
            step["planning_type"] = goal->planning_type;
            step["arm_group"] = goal->arm_group;
            if (!goal->direction.empty()) {
                step["direction"] = goal->direction;
            }
            if (goal->distance != 0.0) {
                step["distance"] = goal->distance;
            }

            // Provide continuous feedback during execution
            feedback->current_operation = "Initializing MoveTo task";
            feedback->progress_percentage = 10.0f;
            goal_handle->publish_feedback(feedback);

            // Check for abort before starting
            if (moveto_abort_requested_ || goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "MoveTo goal canceled before execution");
                result->success = false;
                result->error_message = "Task was canceled";
                goal_handle->canceled(result);
                return;
            }

            feedback->current_operation = "Syncing robot parameters";
            feedback->progress_percentage = 20.0f;
            goal_handle->publish_feedback(feedback);

            // CRITICAL: Sync robot description from move_group (same as working version)
            if (!update_robot_description_from("move_group", this->shared_from_this())) {
                RCLCPP_ERROR(this->get_logger(), "Failed to sync robot description from move_group");
                result->success = false;
                result->error_message = "Failed to sync robot description";
                goal_handle->abort(result);
                return;
            }

            feedback->current_operation = "Planning trajectory";
            feedback->progress_percentage = 40.0f;
            goal_handle->publish_feedback(feedback);

            // Use the reusable MoveTo instance (same as working version)
            if (!moveto_instance_) {
                RCLCPP_ERROR(this->get_logger(), "MoveTo instance not initialized");
                result->success = false;
                result->error_message = "MoveTo instance not available";
                goal_handle->abort(result);
                return;
            }

            // Execute using FSM-style behavior (no cancellation callback)
            bool success = moveto_instance_->run(step, poses, this->shared_from_this());

            // Check for abort after execution
            if (moveto_abort_requested_ || goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "MoveTo goal canceled during execution");
                result->success = false;
                result->error_message = "Task was canceled";
                goal_handle->canceled(result);
                return;
            }

            feedback->current_operation = "MoveTo completed";
            feedback->progress_percentage = 100.0f;
            goal_handle->publish_feedback(feedback);

            result->success = success;
            if (success) {
                result->error_message = "";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "MoveTo goal succeeded");
            } else {
                result->error_message = "MoveTo execution failed";
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "MoveTo goal failed");
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "MoveTo execution exception: %s", e.what());
            result->success = false;
            result->error_message = std::string("Exception: ") + e.what();
            goal_handle->abort(result);
        }

        // Don't reset the reusable instance - keep it for next use
    }

    // ========== EMBEDDED ENDEFFECTOR ACTION SERVER HANDLERS ==========
    
rclcpp_action::GoalResponse MTCOrchestratorActionServer::handle_endeffector_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const EndEffectorAction::Goal> goal)
    {
        return handle_simple_goal<EndEffectorAction>(uuid, goal, "EndEffector");
    }

rclcpp_action::CancelResponse MTCOrchestratorActionServer::handle_endeffector_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<EndEffectorAction>> goal_handle)
    {
        return handle_simple_cancel<EndEffectorAction>(goal_handle, endeffector_abort_requested_, "EndEffector");
    }

void MTCOrchestratorActionServer::handle_endeffector_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<EndEffectorAction>> goal_handle)
    {
        handle_simple_accepted<EndEffectorAction>(goal_handle, &MTCOrchestratorActionServer::execute_endeffector_embedded);
    }

void MTCOrchestratorActionServer::execute_endeffector_embedded(const std::shared_ptr<rclcpp_action::ServerGoalHandle<EndEffectorAction>> goal_handle)
    {
        endeffector_abort_requested_ = false;
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<EndEffectorAction::Feedback>();
        auto result = std::make_shared<EndEffectorAction::Result>();
        
        try {
            // Parse poses JSON
            nlohmann::json poses;
            try {
                poses = nlohmann::json::parse(goal->poses_json);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse poses JSON: %s", e.what());
                result->success = false;
                result->error_message = "Invalid poses JSON";
                goal_handle->abort(result);
                return;
            }

            // Create step JSON from goal
            nlohmann::json step;
            step["end_effector_type"] = goal->end_effector_type;
            step["end_effector_action"] = goal->end_effector_action;
            if (goal->position != 0.0) {
                step["position"] = goal->position;
            }
            if (goal->force != 0.0) {
                step["force"] = goal->force;
            }
            if (goal->pressure != 0.0) {
                step["pressure"] = goal->pressure;
            }

            // Provide continuous feedback during execution
            feedback->current_operation = "Initializing EndEffector control";
            feedback->progress_percentage = 10.0f;
            goal_handle->publish_feedback(feedback);

            // Check for abort before starting
            if (endeffector_abort_requested_ || goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "EndEffector goal canceled before execution");
                result->success = false;
                result->error_message = "Task was canceled";
                goal_handle->canceled(result);
                return;
            }

            feedback->current_operation = "Executing " + goal->end_effector_action + " on " + goal->end_effector_type;
            feedback->progress_percentage = 50.0f;
            goal_handle->publish_feedback(feedback);

            // Use the reusable EndEffector instance
            if (!endeffector_instance_) {
                RCLCPP_ERROR(this->get_logger(), "EndEffector instance not initialized");
                result->success = false;
                result->error_message = "EndEffector instance not available";
                goal_handle->abort(result);
                return;
            }

            // Execute using the same pattern as the working version
            bool success = endeffector_instance_->run(step, poses, this->shared_from_this());

            // Check for abort after execution
            if (endeffector_abort_requested_ || goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "EndEffector goal canceled during execution");
                result->success = false;
                result->error_message = "Task was canceled";
                goal_handle->canceled(result);
                return;
            }

            feedback->current_operation = "EndEffector control completed";
            feedback->progress_percentage = 100.0f;
            goal_handle->publish_feedback(feedback);

            result->success = success;
            if (success) {
                result->error_message = "";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "EndEffector goal succeeded");
            } else {
                result->error_message = "EndEffector execution failed";
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "EndEffector goal failed");
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "EndEffector execution exception: %s", e.what());
            result->success = false;
            result->error_message = std::string("Exception: ") + e.what();
            goal_handle->abort(result);
        }

        // Don't reset the reusable instance - keep it for next use
    }

    // ========== EMBEDDED TOOLEXCHANGE ACTION SERVER HANDLERS ==========
    
rclcpp_action::GoalResponse MTCOrchestratorActionServer::handle_toolexchange_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ToolExchangeAction::Goal> goal)
    {
        return handle_simple_goal<ToolExchangeAction>(uuid, goal, "ToolExchange");
    }

rclcpp_action::CancelResponse MTCOrchestratorActionServer::handle_toolexchange_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ToolExchangeAction>> goal_handle)
    {
        return handle_simple_cancel<ToolExchangeAction>(goal_handle, toolexchange_abort_requested_, "ToolExchange");
    }

void MTCOrchestratorActionServer::handle_toolexchange_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ToolExchangeAction>> goal_handle)
    {
        handle_simple_accepted<ToolExchangeAction>(goal_handle, &MTCOrchestratorActionServer::execute_toolexchange_embedded);
    }

void MTCOrchestratorActionServer::execute_toolexchange_embedded(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ToolExchangeAction>> goal_handle)
    {
        toolexchange_abort_requested_ = false;
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ToolExchangeAction::Feedback>();
        auto result = std::make_shared<ToolExchangeAction::Result>();
        
        try {
            // Parse poses JSON
            nlohmann::json poses;
            try {
                poses = nlohmann::json::parse(goal->poses_json);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse poses JSON: %s", e.what());
                result->success = false;
                result->error_message = "Invalid poses JSON";
                goal_handle->abort(result);
                return;
            }

            // Create step JSON from goal
            nlohmann::json step;
            step["operation"] = goal->operation;
            step["gripper"] = goal->gripper;
            step["dock_number"] = goal->dock_number;
            step["poses"] = goal->approach_poses;

            // Provide continuous feedback
            feedback->current_operation = "Initializing ToolExchange: " + goal->operation;
            feedback->progress_percentage = 10.0f;
            goal_handle->publish_feedback(feedback);

            // Check for abort
            if (toolexchange_abort_requested_ || goal_handle->is_canceling()) {
                result->success = false;
                result->error_message = "Task was canceled";
                goal_handle->canceled(result);
                return;
            }

            feedback->current_operation = "Syncing robot parameters";
            feedback->progress_percentage = 20.0f;
            goal_handle->publish_feedback(feedback);

            // Sync robot description from move_group
            if (!update_robot_description_from("move_group", this->shared_from_this())) {
                result->success = false;
                result->error_message = "Failed to sync robot description";
                goal_handle->abort(result);
                return;
            }

            feedback->current_operation = "Executing " + goal->operation + " operation";
            feedback->progress_percentage = 50.0f;
            goal_handle->publish_feedback(feedback);

            // Use reusable instance
            if (!toolexchange_instance_) {
                result->success = false;
                result->error_message = "ToolExchange instance not available";
                goal_handle->abort(result);
                return;
            }

            bool success = toolexchange_instance_->run(step, poses, this->shared_from_this());

            // Check for abort after execution
            if (toolexchange_abort_requested_ || goal_handle->is_canceling()) {
                result->success = false;
                result->error_message = "Task was canceled";
                goal_handle->canceled(result);
                return;
            }

            feedback->current_operation = "ToolExchange completed";
            feedback->progress_percentage = 100.0f;
            goal_handle->publish_feedback(feedback);

            result->success = success;
            if (success) {
                result->error_message = "";
                goal_handle->succeed(result);
            } else {
                result->error_message = "ToolExchange execution failed";
                goal_handle->abort(result);
            }

        } catch (const std::exception& e) {
            result->success = false;
            result->error_message = std::string("Exception: ") + e.what();
            goal_handle->abort(result);
        }
    }

    // ========== EMBEDDED PICKPLACE ACTION SERVER HANDLERS ==========
    
rclcpp_action::GoalResponse MTCOrchestratorActionServer::handle_pickplace_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PickPlaceAction::Goal> goal)
    {
        return handle_simple_goal<PickPlaceAction>(uuid, goal, "PickPlace");
    }

rclcpp_action::CancelResponse MTCOrchestratorActionServer::handle_pickplace_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceAction>> goal_handle)
    {
        return handle_simple_cancel<PickPlaceAction>(goal_handle, pickplace_abort_requested_, "PickPlace");
    }

void MTCOrchestratorActionServer::handle_pickplace_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceAction>> goal_handle)
    {
        handle_simple_accepted<PickPlaceAction>(goal_handle, &MTCOrchestratorActionServer::execute_pickplace_embedded);
    }

void MTCOrchestratorActionServer::execute_pickplace_embedded(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceAction>> goal_handle)
    {
        pickplace_abort_requested_ = false;
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<PickPlaceAction::Feedback>();
        auto result = std::make_shared<PickPlaceAction::Result>();
        
        try {
            // Parse poses JSON
            nlohmann::json poses;
            try {
                poses = nlohmann::json::parse(goal->poses_json);
            } catch (const std::exception& e) {
                result->success = false;
                result->error_message = "Invalid poses JSON";
                goal_handle->abort(result);
                return;
            }

            // Create step JSON from goal
            nlohmann::json step;
            step["gripper"] = goal->gripper;
            step["pick_pose"] = goal->pick_pose;
            step["place_pose"] = goal->place_pose;
            step["approach_distance"] = goal->approach_distance;
            step["planning_type"] = goal->planning_type;
            step["arm_group"] = goal->arm_group;

            // Multi-phase feedback
            feedback->current_operation = "Initializing PickPlace";
            feedback->current_phase = "initialization";
            feedback->progress_percentage = 5.0f;
            goal_handle->publish_feedback(feedback);

            if (pickplace_abort_requested_ || goal_handle->is_canceling()) {
                result->success = false;
                result->error_message = "Task was canceled";
                goal_handle->canceled(result);
                return;
            }

            feedback->current_operation = "Syncing robot parameters";
            feedback->progress_percentage = 10.0f;
            goal_handle->publish_feedback(feedback);

            if (!update_robot_description_from("move_group", this->shared_from_this())) {
                result->success = false;
                result->error_message = "Failed to sync robot description";
                goal_handle->abort(result);
                return;
            }

            feedback->current_operation = "Starting pick phase";
            feedback->current_phase = "pick";
            feedback->progress_percentage = 25.0f;
            goal_handle->publish_feedback(feedback);

            if (!pickplace_instance_) {
                result->success = false;
                result->error_message = "PickPlace instance not available";
                goal_handle->abort(result);
                return;
            }

            bool success = pickplace_instance_->run(step, poses, this->shared_from_this());

            if (pickplace_abort_requested_ || goal_handle->is_canceling()) {
                result->success = false;
                result->error_message = "Task was canceled";
                goal_handle->canceled(result);
                return;
            }

            feedback->current_operation = "PickPlace completed";
            feedback->current_phase = "completed";
            feedback->progress_percentage = 100.0f;
            goal_handle->publish_feedback(feedback);

            result->success = success;
            if (success) {
                result->error_message = "";
                goal_handle->succeed(result);
            } else {
                result->error_message = "PickPlace execution failed";
                goal_handle->abort(result);
            }

        } catch (const std::exception& e) {
            result->success = false;
            result->error_message = std::string("Exception: ") + e.what();
            goal_handle->abort(result);
        }
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
        
        // Handle tool exchange tasks - using embedded action approach
        if (action == "tool_exchange") {
            const std::string operation = step.value("operation", "");
            const std::string requested_tool = step.value("gripper", orchestrator_->get_current_gripper());

            // Execute via embedded action (call the embedded action server via ROS2 actions)
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

        // Handle pick and place tasks - using embedded action approach
        if (action == "pick_and_place") {
            std::string need = step.value("gripper", orchestrator_->get_current_gripper());
            if (!switch_gripper(need, robot_ip))
                return false;

            return call_pickplace_action(step, poses);
        }

        // Handle simple move-to tasks - using embedded action approach
        if (action == "moveto") {
            return call_moveto_action(step, poses);
        }

        // Handle end effector operations - using embedded action approach
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
    goal.arm_group = step.value("arm_group", "ur_arm");
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
    goal.position = step.value("position", 0.0);
    goal.force = step.value("force", 0.0);
    goal.pressure = step.value("pressure", 0.0);
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
    goal.approach_distance = step.value("approach_distance", 0.1);
    goal.planning_type = step.value("planning_type", "joint");
    goal.arm_group = step.value("arm_group", "ur_arm");
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

    // Internal methods to execute embedded actions without action server overhead
bool MTCOrchestratorActionServer::execute_moveto_embedded_internal(const nlohmann::json& step, const nlohmann::json& poses) {
        try {
            // CRITICAL: Sync robot description from move_group
            if (!update_robot_description_from("move_group", this->shared_from_this())) {
                RCLCPP_ERROR(this->get_logger(), "Failed to sync robot description from move_group");
                return false;
            }

            // Use the reusable MoveTo instance
            if (!moveto_instance_) {
                RCLCPP_ERROR(this->get_logger(), "MoveTo instance not initialized");
                return false;
            }

            // Execute using the same pattern as the working version
            bool success = moveto_instance_->run(step, poses, this->shared_from_this());
            return success;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "MoveTo execution exception: %s", e.what());
            return false;
        }
    }

bool MTCOrchestratorActionServer::execute_endeffector_embedded_internal(const nlohmann::json& step, const nlohmann::json& poses) {
        try {
            // Use the reusable EndEffector instance
            if (!endeffector_instance_) {
                RCLCPP_ERROR(this->get_logger(), "EndEffector instance not initialized");
                return false;
            }

            // Execute using the same pattern as the working version
            bool success = endeffector_instance_->run(step, poses, this->shared_from_this());
            return success;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "EndEffector execution exception: %s", e.what());
            return false;
        }
    }

bool MTCOrchestratorActionServer::execute_toolexchange_embedded_internal(const nlohmann::json& step, const nlohmann::json& poses) {
        try {
            // CRITICAL: Sync robot description from move_group
            if (!update_robot_description_from("move_group", this->shared_from_this())) {
                RCLCPP_ERROR(this->get_logger(), "Failed to sync robot description from move_group");
                return false;
            }

            // Use the reusable ToolExchange instance
            if (!toolexchange_instance_) {
                RCLCPP_ERROR(this->get_logger(), "ToolExchange instance not initialized");
                return false;
            }

            bool success = toolexchange_instance_->run(step, poses, this->shared_from_this());
            return success;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "ToolExchange execution exception: %s", e.what());
        return false;
        }
    }

bool MTCOrchestratorActionServer::execute_pickplace_embedded_internal(const nlohmann::json& step, const nlohmann::json& poses) {
        try {
            // CRITICAL: Sync robot description from move_group
            if (!update_robot_description_from("move_group", this->shared_from_this())) {
                RCLCPP_ERROR(this->get_logger(), "Failed to sync robot description from move_group");
                return false;
            }

            // Use the reusable PickPlace instance
            if (!pickplace_instance_) {
                RCLCPP_ERROR(this->get_logger(), "PickPlace instance not initialized");
                return false;
            }

            bool success = pickplace_instance_->run(step, poses, this->shared_from_this());
            return success;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "PickPlace execution exception: %s", e.what());
            return false;
        }
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
            
            
            const auto& sequence = task_script["sequence"];
            const auto& poses = task_script["poses"];
            
            result->total_steps = sequence.size();
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
            
            // Activate robot controller
            RCLCPP_INFO(this->get_logger(), "Activating scaled_joint_trajectory_controller...");
            system("ros2 control switch_controllers --activate scaled_joint_trajectory_controller");
            
            play_dashboard_client(this->shared_from_this());
            
            if (!update_robot_description_from("move_group", this->shared_from_this())) {
                throw std::runtime_error("Failed to update robot description");
            }
            
            orchestrator_->set_current_gripper(start_gripper);

            // Initialize reusable instances for embedded actions
            moveto_instance_ = std::make_shared<MoveToStages>(this->shared_from_this(), task_script);
            endeffector_instance_ = std::make_shared<EndEffectorStages>(this->shared_from_this(), task_script);
            toolexchange_instance_ = std::make_shared<ToolExchangeStages>(this->shared_from_this(), task_script);
            pickplace_instance_ = std::make_shared<PickPlaceStages>(this->shared_from_this(), task_script);

            // Execute task sequence
            for (size_t i = 0; i < sequence.size(); ++i) {
                // Check if goal was cancelled
                if (goal_handle->is_canceling()) {
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    result->success = false;
                    result->error_message = "Task was canceled";
                    goal_handle->canceled(result);
                    is_executing_ = false;
                    return;
                }
                
                const auto& step = sequence[i];
                const std::string action = step["action"];
                
                // Update feedback
                feedback->current_step = i + 1;
                feedback->current_action = action;
                feedback->progress_percentage = static_cast<float>(i + 1) / sequence.size() * 100.0f;
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
        
        // Reset instances for next task
        moveto_instance_.reset();
        endeffector_instance_.reset();
        toolexchange_instance_.reset();
        pickplace_instance_.reset();
        
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
