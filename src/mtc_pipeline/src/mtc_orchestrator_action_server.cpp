#include "mtc_pipeline/mtc_orchestrator_action_server.hpp"

// MTCOrchestratorActionServer implementation
MTCOrchestratorActionServer::MTCOrchestratorActionServer(const rclcpp::NodeOptions& options)
        : Node("mtc_orchestrator_action_server", options), is_executing_(false) {
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

        RCLCPP_INFO(this->get_logger(), "MTC Orchestrator Action Server started");
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
        if (current_gripper_ == new_gripper) return true;

        // Just reuse the initialization logic - switching gripper IS reinitializing MoveIt
        if (!initialize_moveit_stack(new_gripper, robot_ip)) {
            return false;
        }

        current_gripper_ = new_gripper;
        return true;
    }


// Execute a single task step
bool MTCOrchestratorActionServer::execute_step(const std::string& action, const nlohmann::json& step,
                     const nlohmann::json& poses, const std::string& robot_ip) {

        if (action == "tool_exchange") return handle_tool_exchange(step, poses, robot_ip);
        if (action == "pick_and_place") return handle_pick_and_place(step, poses);
        if (action == "moveto") return call_moveto_action(step, poses);
        if (action == "end_effector") return call_endeffector_action(step, poses);

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

// Helper functions for execute_step
bool MTCOrchestratorActionServer::handle_tool_exchange(const nlohmann::json& step, const nlohmann::json& poses, const std::string& robot_ip) {
    const std::string operation = step.value("operation", "");
    const std::string requested_tool = step.value("gripper", current_gripper_);

    // Execute via delegation to modular action servers
    if (!call_toolexchange_action(step, poses)) {
        return false;
    }

    // Handle gripper switching after tool exchange
    if (operation == "dock") {
        return switch_gripper("none", robot_ip);
    } else if (operation == "load") {
        return switch_gripper(requested_tool, robot_ip);
    }
    return true;
}

bool MTCOrchestratorActionServer::handle_pick_and_place(const nlohmann::json& step, const nlohmann::json& poses) {
    // Use whatever gripper is currently attached - gripper switching should be done via tool_exchange
    return call_pickplace_action(step, poses);
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

    const std::string launch_cmd = "ros2 launch " + it->second + " move_group.launch.py robot_ip:=" + robot_ip + " &";
    RCLCPP_DEBUG(this->get_logger(), "Launch command: %s", launch_cmd.c_str());
    std::system(launch_cmd.c_str());

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

    // Wait for PlanningScene service - this is what action servers actually need
    RCLCPP_DEBUG(this->get_logger(), "Waiting for PlanningScene service...");
    auto ps_client = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!ps_client->wait_for_service(30s)) {
        RCLCPP_ERROR(this->get_logger(), "PlanningScene service not ready within timeout");
        return false;
    }

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


    current_gripper_ = start_gripper;
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
                feedback->current_gripper = current_gripper_;
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

        // Cleanup - processes will clean up automatically when orchestrator exits

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
