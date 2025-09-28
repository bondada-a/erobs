#include "mtc_pipeline/mtc_orchestrator_action_server.hpp"

#include <algorithm>
#include <rclcpp/parameter.hpp>

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

// === MAIN EXECUTION FLOW ===

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
        if (goal->robot_ip.empty()) {
            throw std::runtime_error("Goal missing required robot_ip");
        }
        const std::string robot_ip = goal->robot_ip;

        if (!task_script.contains("start_gripper") || !task_script["start_gripper"].is_string()) {
            throw std::runtime_error("Task script missing required start_gripper");
        }
        const std::string start_gripper = task_script["start_gripper"].get<std::string>();

        const auto& tasks = task_script["tasks"];
        const auto& poses = task_script["poses"];

        result->total_steps = tasks.size();
        result->completed_steps = 0;

        // Send initial feedback
        update_feedback(feedback, goal_handle, 0, tasks.size(), "Initializing MoveIt", "Starting MoveIt configuration");

        // Initialize MoveIt stack
        if (!initialize_moveit_stack(start_gripper, robot_ip)) {
            throw std::runtime_error("Failed to initialize MoveIt stack");
        }

        // Execute tasks
        for (size_t i = 0; i < tasks.size(); ++i) {
            // Check if goal was cancelled
            if (goal_handle->is_canceling()) {
                RCLCPP_DEBUG(this->get_logger(), "Goal canceled");
                result->success = false;
                result->error_message = "Task was canceled";
                goal_handle->canceled(result);
                is_executing_ = false;
                return;
            }

            const auto& step = tasks[i];
            std::string task_type = step.value("task_type", "");
            if (task_type.empty()) {
                throw std::runtime_error("Step missing 'task_type' field");
            }

            // Update feedback
            update_feedback(feedback, goal_handle, i + 1, tasks.size(), task_type, "Executing: " + task_type);

            RCLCPP_DEBUG(this->get_logger(), "Executing step %zu: %s", i + 1, task_type.c_str());

            // Execute step
            if (!execute_step(task_type, step, poses, robot_ip)) {
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

        update_feedback(feedback, goal_handle, tasks.size(), tasks.size(), "", "Task completed successfully");

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

bool MTCOrchestratorActionServer::execute_step(const std::string& task_type, const nlohmann::json& step,
                 const nlohmann::json& poses, const std::string& robot_ip) {
    if (task_type == "tool_exchange") return handle_tool_exchange(step, poses, robot_ip);
    if (task_type == "pick_and_place") return call_pickplace_action(step, poses);
    if (task_type == "moveto") return call_moveto_action(step, poses);
    if (task_type == "end_effector") return call_endeffector_action(step, poses);

    return false;
}

// === ACTION SERVER HANDLERS ===

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

// === MOVEIT STACK MANAGEMENT ===

bool MTCOrchestratorActionServer::initialize_moveit_stack(const std::string& start_gripper, const std::string& robot_ip) {
    // Kill any existing MoveIt processes first
    RCLCPP_INFO(this->get_logger(), "Killing existing MoveIt processes...");
    const int kill_moveit = std::system("pkill -f 'ros2 launch.*moveit_config.*move_group.launch.py'");
    const int kill_ur = std::system("pkill -f 'ur_ros2_control_node'");
    const int kill_move_group = std::system("pkill -f 'move_group'");
    const int kill_rsp = std::system("pkill -f 'robot_state_publisher'");
    const int kill_rviz = std::system("pkill -f 'rviz2'");
    (void)kill_moveit;
    (void)kill_ur;
    (void)kill_move_group;
    (void)kill_rsp;
    (void)kill_rviz;

    // Wait a bit for processes to die
    std::this_thread::sleep_for(3s);

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
    const int launch_ret = std::system(launch_cmd.c_str());
    (void)launch_ret;

    // Wait for PlanningScene service (this confirms MoveIt is ready)
    auto ps_client = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!ps_client->wait_for_service(30s)) {
        RCLCPP_ERROR(this->get_logger(), "MoveIt not ready within 30s");
        return false;
    }

    if (!sync_robot_descriptions()) {
        RCLCPP_WARN(this->get_logger(), "Failed to sync robot descriptions after launching MoveIt");
    }

    // Wait for robot hardware to be ready - simple timeout approach
    RCLCPP_INFO(this->get_logger(), "Waiting for robot hardware to initialize...");
    std::this_thread::sleep_for(5s);

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

bool MTCOrchestratorActionServer::sync_robot_descriptions()
{
    auto node = this->shared_from_this();
    auto source_client = std::make_shared<rclcpp::SyncParametersClient>(node, "move_group");

    if (!source_client->wait_for_service(5s)) {
        RCLCPP_ERROR(this->get_logger(), "Unable to reach move_group parameter service for robot description sync");
        return false;
    }

    std::vector<std::string> param_names = {
        "robot_description",
        "robot_description_semantic",
        "robot_description_kinematics",
        "robot_description_planning"
    };

    std::vector<rclcpp::Parameter> description_params;
    try {
        description_params = source_client->get_parameters(param_names);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to retrieve robot description parameters: %s", e.what());
        return false;
    }

    description_params.erase(
        std::remove_if(description_params.begin(), description_params.end(),
                       [](const rclcpp::Parameter& param) { return param.get_name().empty(); }),
        description_params.end());

    if (description_params.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No robot_description parameters retrieved from move_group");
        return false;
    }

    // Update orchestrator's own parameters
    this->set_parameters(description_params);

    static const std::vector<std::string> target_nodes = {
        "moveto_action_server",
        "endeffector_action_server",
        "pickplace_action_server",
        "toolexchange_action_server"
    };

    for (const auto& target : target_nodes) {
        auto target_client = std::make_shared<rclcpp::SyncParametersClient>(node, target);
        if (!target_client->wait_for_service(5s)) {
            RCLCPP_WARN(this->get_logger(), "Parameter service for %s unavailable during robot description sync", target.c_str());
            continue;
        }

        try {
            target_client->set_parameters(description_params);
            RCLCPP_INFO(this->get_logger(), "Synced robot description parameters to %s", target.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to sync robot description to %s: %s", target.c_str(), e.what());
        }
    }

    return true;
}

bool MTCOrchestratorActionServer::switch_gripper(const std::string& new_gripper, const std::string& robot_ip) {
    if (current_gripper_ == new_gripper) return true;

    // Just reuse the initialization logic - switching gripper IS reinitializing MoveIt
    if (!initialize_moveit_stack(new_gripper, robot_ip)) {
        return false;
    }

    current_gripper_ = new_gripper;
    return true;
}

// === STEP EXECUTION HELPERS ===

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

// === ACTION CLIENT METHODS ===

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

    // Add timeout for action execution (2 minutes)
    constexpr auto ACTION_TIMEOUT = 120s;
    if (result_future.wait_for(ACTION_TIMEOUT) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "%s action timed out after %ld seconds", action_name.c_str(), ACTION_TIMEOUT.count());
        client->async_cancel_goal(goal_handle);
        return false;
    }

    auto result = result_future.get();

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return result.result->success;
    }
    return false;
}

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

// === UTILITY FUNCTIONS ===

void MTCOrchestratorActionServer::update_feedback(std::shared_ptr<MTCExecution::Feedback> feedback,
                    std::shared_ptr<GoalHandleMTCExecution> goal_handle,
                    size_t current_step, size_t total_steps, const std::string& task_type,
                    const std::string& status_message) {
    feedback->current_step = current_step;
    feedback->current_action = task_type;
    feedback->progress_percentage = static_cast<float>(current_step) / total_steps * 100.0f;
    feedback->status_message = status_message;
    feedback->current_gripper = current_gripper_;
    goal_handle->publish_feedback(feedback);
}

// === TEMPLATE INSTANTIATIONS ===

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
