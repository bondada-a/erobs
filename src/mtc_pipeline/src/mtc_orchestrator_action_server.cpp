#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter_client.hpp>
#include <memory>
#include <thread>
#include <chrono>
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <string>
#include <iostream>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <linux/limits.h>
#include <cstdlib>

#include "mtc_pipeline/action/mtc_execution.hpp"
#include "mtc_pipeline/pick_place_stages.hpp"
#include "mtc_pipeline/tool_exchange_stages.hpp"
#include "mtc_pipeline/moveto_stages.hpp"
#include "mtc_pipeline/end_effector_stages.hpp"

using namespace std::chrono_literals;
using MTCExecution = mtc_pipeline::action::MTCExecution;
using GoalHandleMTCExecution = rclcpp_action::ServerGoalHandle<MTCExecution>;

namespace {
    // Wait for ROS2 service to become available
    bool wait_for_service(rclcpp::Node::SharedPtr node, const std::string& service_name, std::chrono::seconds timeout = 30s) {
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
                RCLCPP_INFO(node->get_logger(), "Getting robot_description parameter from %s", source_node.c_str());
                auto urdf_future = client->get_parameters({"robot_description"});
                RCLCPP_INFO(node->get_logger(), "Getting robot_description_semantic parameter from %s", source_node.c_str());
                auto srdf_future = client->get_parameters({"robot_description_semantic"});
                
                // Wait for both futures to complete
                auto future_start_time = std::chrono::steady_clock::now();
                const auto future_timeout = std::chrono::seconds(10);
                
                while (std::chrono::steady_clock::now() - future_start_time < future_timeout) {
                    if (urdf_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready &&
                        srdf_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                        
                        auto urdf_params = urdf_future.get();
                        auto srdf_params = srdf_future.get();
                        
                        if (urdf_params.size() > 0 && srdf_params.size() > 0) {
                            // Check if parameters are not empty strings
                            std::string urdf_value = urdf_params[0].as_string();
                            std::string srdf_value = srdf_params[0].as_string();
                            
                            if (!urdf_value.empty() && !srdf_value.empty()) {
                                node->set_parameters({{"robot_description", urdf_value}, {"robot_description_semantic", srdf_value}});
                                RCLCPP_INFO(node->get_logger(), "Robot params synced from [%s]", source_node.c_str());
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
        if (g == "epick") return setup_cmd + "ros2 launch ur_epick_moveit_config move_group.launch.py robot_ip:=" + ip;
        if (g == "hande") return setup_cmd + "ros2 launch ur_hande_moveit_config move_group.launch.py robot_ip:=" + ip;
        return "";
    }
}

// Manages MoveIt configuration processes
class Orchestrator {
    std::vector<pid_t> active_pids_; 
    std::string current_gripper_ = "none"; 
    
public:
    // Launch new MoveIt configuration process
    pid_t launch(const std::string& cmd) {
        pid_t pid = fork(); 
        if (pid == 0) {
            execl("/usr/bin/setsid", "setsid", "bash", "-c", cmd.c_str(), (char*)nullptr);
            exit(1); 
        }
        active_pids_.push_back(pid);
        return pid;
    }

    // Gracefully terminate all processes
    void kill_all_and_wait() {
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

    void set_current_gripper(const std::string& g) { current_gripper_ = g; }
    const std::string& get_current_gripper() const { return current_gripper_; }
};

class MTCOrchestratorActionServer : public rclcpp::Node
{
public:
    using ActionServer = rclcpp_action::Server<MTCExecution>;

    MTCOrchestratorActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
        : Node("mtc_orchestrator_action_server", options), is_executing_(false) {
        // Parameters are now automatically declared from overrides
        // No need to manually declare them
        
        // Initialize orchestrator
        orchestrator_ = std::make_unique<Orchestrator>();
        
        // Initialize the action server
        this->action_server_ = rclcpp_action::create_server<MTCExecution>(
            this,
            "mtc_execution",
            std::bind(&MTCOrchestratorActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MTCOrchestratorActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MTCOrchestratorActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MTC Orchestrator Action Server started");
    }

private:
    ActionServer::SharedPtr action_server_;
    std::unique_ptr<Orchestrator> orchestrator_;
    bool is_executing_;

    rclcpp_action::GoalResponse handle_goal(
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

    rclcpp_action::CancelResponse handle_cancel(
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

    void handle_accepted(const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
    {
        std::thread{std::bind(&MTCOrchestratorActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // Switch to different gripper configuration
    bool switch_gripper(const std::string& new_gripper, const std::string& robot_ip) {
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
    bool execute_step(const std::string& action, const nlohmann::json& step, 
                     const nlohmann::json& poses, const std::string& robot_ip,
                     PickPlaceStages& pick_place, ToolExchangeStages& tool_exch,
                     MoveToStages& moveto, EndEffectorStages& end_effector) {
        
        // Handle tool exchange tasks
        if (action == "tool_exchange") {
            const std::string operation = step.value("operation", "");
            const std::string requested_tool = step.value("gripper", orchestrator_->get_current_gripper());

            if (!update_robot_description_from("move_group", this->shared_from_this()) || 
                !tool_exch.run(step, poses, this->shared_from_this()))
                return false;

            if (operation == "dock") {
                return switch_gripper("none", robot_ip);
            } else if (operation == "load") {
                return switch_gripper(requested_tool, robot_ip);
            }
            return true;
        }

        // Handle pick and place tasks
        if (action == "pick_and_place") {
            std::string need = step.value("gripper", orchestrator_->get_current_gripper());
            if (!switch_gripper(need, robot_ip))
                return false;

            return update_robot_description_from("move_group", this->shared_from_this()) && 
                   pick_place.run(step, poses, this->shared_from_this());
        }

        // Handle simple move-to tasks
        if (action == "moveto") {
            return update_robot_description_from("move_group", this->shared_from_this()) && 
                   moveto.run(step, poses, this->shared_from_this());
        }

        // Handle end effector operations
        if (action == "end_effector") {
            return end_effector.run(step, poses, this->shared_from_this());
        }

        return false;
    }

    void execute(const std::shared_ptr<GoalHandleMTCExecution> goal_handle)
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

            // Create task handlers
            PickPlaceStages pick_place(this->shared_from_this(), task_script);
            ToolExchangeStages tool_exch(this->shared_from_this(), task_script);
            MoveToStages moveto(this->shared_from_this(), task_script);
            EndEffectorStages end_effector(this->shared_from_this(), task_script);

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
                
                // Execute step
                if (!execute_step(action, step, poses, robot_ip, pick_place, tool_exch, moveto, end_effector)) {
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
};

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
