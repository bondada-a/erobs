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
        if (action == "pick_and_place") return call_pickplace_action(step, poses);
        if (action == "moveto") return call_moveto_action(step, poses);
        if (action == "end_effector") return call_endeffector_action(step, poses);

        return false;
}

// Action client methods to call modular action servers via ROS2 actions
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

    // Wait for PlanningScene service (this confirms MoveIt is ready)
    auto ps_client = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!ps_client->wait_for_service(30s)) {
        RCLCPP_ERROR(this->get_logger(), "MoveIt not ready within 30s");
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

void MTCOrchestratorActionServer::update_feedback(std::shared_ptr<MTCExecution::Feedback> feedback,
                    std::shared_ptr<GoalHandleMTCExecution> goal_handle,
                    size_t current_step, size_t total_steps, const std::string& action,
                    const std::string& status_message) {
    feedback->current_step = current_step;
    feedback->current_action = action;
    feedback->progress_percentage = static_cast<float>(current_step) / total_steps * 100.0f;
    feedback->status_message = status_message;
    feedback->current_gripper = current_gripper_;
    goal_handle->publish_feedback(feedback);
}

// Template implementation
template<typename ActionType>
bool MTCOrchestratorActionServer::call_action_generic(
    typename rclcpp_action::Client<ActionType>::SharedPtr client,
    const std::string& action_name,
    const nlohmann::json& step,
    const nlohmann::json& poses,
    std::function<void(typename ActionType::Goal&, const nlohmann::json&, const nlohmann::json&)> populate_goal
) {
    if (!client->wait_for_action_server(ACTION_SERVER_TIMEOUT)) {
        RCLCPP_ERROR(this->get_logger(), "%s action server unavailable", action_name.c_str());
        return false;
    }

    auto goal = typename ActionType::Goal();
    populate_goal(goal, step, poses);

    auto future = client->async_send_goal(goal);
    auto goal_handle = future.get();

    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send %s goal", action_name.c_str());
        return false;
    }

    auto result_future = client->async_get_result(goal_handle);
    auto result = result_future.get();

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return result.result->success;
    }
    return false;
}

// Explicit template instantiations
template bool MTCOrchestratorActionServer::call_action_generic<MoveToAction>(
    rclcpp_action::Client<MoveToAction>::SharedPtr, const std::string&, const nlohmann::json&, const nlohmann::json&,
    std::function<void(MoveToAction::Goal&, const nlohmann::json&, const nlohmann::json&)>);

template bool MTCOrchestratorActionServer::call_action_generic<EndEffectorAction>(
    rclcpp_action::Client<EndEffectorAction>::SharedPtr, const std::string&, const nlohmann::json&, const nlohmann::json&,
    std::function<void(EndEffectorAction::Goal&, const nlohmann::json&, const nlohmann::json&)>);

template bool MTCOrchestratorActionServer::call_action_generic<ToolExchangeAction>(
    rclcpp_action::Client<ToolExchangeAction>::SharedPtr, const std::string&, const nlohmann::json&, const nlohmann::json&,
    std::function<void(ToolExchangeAction::Goal&, const nlohmann::json&, const nlohmann::json&)>);

template bool MTCOrchestratorActionServer::call_action_generic<PickPlaceAction>(
    rclcpp_action::Client<PickPlaceAction>::SharedPtr, const std::string&, const nlohmann::json&, const nlohmann::json&,
    std::function<void(PickPlaceAction::Goal&, const nlohmann::json&, const nlohmann::json&)>);



void MTCOrchestratorActionServer::execute(const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        is_executing_ = true;
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MTCExecution::Feedback>();
        auto result = std::make_shared<MTCExecution::Result>();

        try {
            // Parse JSON task script
            nlohmann::json task_script = nlohmann::json::parse(goal->task_script_json);

            // Get task parameters
            const std::string robot_ip = goal->robot_ip.empty() ? "192.168.1.101" : goal->robot_ip;
            const std::string start_gripper = task_script.value("start_gripper", "none");

            const auto& operations = task_script["tasks"];
            const auto& poses = task_script["poses"];

            result->total_steps = operations.size();
            result->completed_steps = 0;

            // Send initial feedback
            update_feedback(feedback, goal_handle, 0, operations.size(), "Initializing MoveIt", "Starting MoveIt configuration");

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
                std::string action = step.value("action", step.value("type", ""));
                if (action.empty()) {
                    throw std::runtime_error("Step missing 'action' or 'type' field");
                }

                // Update feedback
                update_feedback(feedback, goal_handle, i + 1, operations.size(), action, "Executing: " + action);
                
                RCLCPP_DEBUG(this->get_logger(), "Executing step %zu: %s", i + 1, action.c_str());
                
                // Execute step
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
            
            update_feedback(feedback, goal_handle, operations.size(), operations.size(), "", "Task completed successfully");
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Execution failed: %s", e.what());
            result->success = false;
            result->error_message = std::string("Execution failed: ") + e.what();
            goal_handle->abort(result);
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
