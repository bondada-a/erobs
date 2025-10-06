#include "mtc_pipeline/action/mtc_execution.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <filesystem>
#include <optional>
#include <signal.h>
#include <atomic>

using MTCExecution = mtc_pipeline::action::MTCExecution;
using GoalHandleMTCExecution = rclcpp_action::ClientGoalHandle<MTCExecution>;

namespace {
// Signal flag for Ctrl+C cancellation (accessed by signal handler and main thread)
std::atomic<bool> should_cancel{false};
}

/**
 * Client node that sends task execution requests to the MTC action server
 * Reads JSON task files and sends them as action goals
 */
class MTCActionClient : public rclcpp::Node {
private:
    rclcpp_action::Client<MTCExecution>::SharedPtr action_client_;
    std::chrono::seconds timeout_;

public:
    explicit MTCActionClient(std::chrono::seconds timeout = std::chrono::seconds(300)) 
        : Node("mtc_action_client"), timeout_(timeout) {
        action_client_ = rclcpp_action::create_client<MTCExecution>(this, "mtc_execution");
    }
    
    /**
     * Wait for the action server to become available
     * @param timeout How long to wait for the server
     * @return true if server is available, false if timeout
     */
    bool wait_for_server(std::chrono::seconds timeout = std::chrono::seconds(10)) {
        return action_client_->wait_for_action_server(timeout);
    }

    /**
     * Cancel the current goal using industry-standard async cancellation
     * Follows ROS2 naming convention: cancel_goal (not cancel_current_goal)
     */
    bool cancel_goal(std::shared_ptr<GoalHandleMTCExecution> goal_handle) {
        if (!goal_handle) {
            RCLCPP_WARN(get_logger(), "No active goal to cancel");
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "Sending cancel request...");
        
        // Industry standard: Use async cancellation
        auto cancel_future = action_client_->async_cancel_goal(goal_handle);
        
        // Wait for cancellation response with timeout
        auto status = rclcpp::spin_until_future_complete(shared_from_this(), cancel_future, std::chrono::seconds(5));
        
        if (status == rclcpp::FutureReturnCode::SUCCESS) {
            auto cancel_response = cancel_future.get();
            if (cancel_response->goals_canceling.size() > 0) {
                RCLCPP_INFO(get_logger(), "Cancel request accepted by server");
                return true;
            } else {
                RCLCPP_WARN(get_logger(), "Cancel request rejected by server");
                return false;
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Cancel request timed out");
            return false;
        }
    }
    
    /**
     * Execute a task from a JSON file
     * @param json_file_path Path to the JSON task file
     * @param robot_ip IP address of the robot
     * @return 0 = success, 1 = failed, 2 = cancelled
     */
    int execute_task(const std::string& json_file_path, const std::string& robot_ip = "192.168.1.101") {
        // Create and send goal
        auto goal_msg = create_goal(json_file_path, robot_ip);
        if (!goal_msg) {
            return 1;  // Failed
        }

        auto goal_handle = send_goal(*goal_msg);
        if (!goal_handle) {
            return 1;  // Failed
        }

        // Wait for completion with cancellation support
        int result = wait_for_completion(goal_handle);

        return result;
    }

private:
    /**
     * Create action goal from JSON file
     */
    std::optional<MTCExecution::Goal> create_goal(const std::string& json_file_path, const std::string& robot_ip) {
        try {
            std::ifstream file(json_file_path);
            std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

            // Validate JSON by parsing
            nlohmann::json::parse(content);

            // Create goal with full JSON
            MTCExecution::Goal goal;
            goal.full_json = content;
            goal.robot_ip = robot_ip;
            return goal;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error creating goal: %s", e.what());
            return std::nullopt;
        }
    }
    
    /**
     * Send goal to action server
     */
    std::shared_ptr<GoalHandleMTCExecution> send_goal(const MTCExecution::Goal& goal) {
        RCLCPP_INFO(get_logger(), "Sending task execution goal...");
        
        auto send_goal_options = rclcpp_action::Client<MTCExecution>::SendGoalOptions();
        send_goal_options.feedback_callback =
            [this](auto, const auto& feedback) { feedback_callback({}, feedback); };
        send_goal_options.result_callback =
            [this](const auto& result) { result_callback(result); };
        
        auto goal_handle_future = action_client_->async_send_goal(goal, send_goal_options);
        
        // Wait for goal acceptance with timeout
        auto status = rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future, timeout_);
        if (status != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Failed to send goal (timeout or error)");
            return nullptr;
        }
        
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
            return nullptr;
        }
        
        RCLCPP_INFO(get_logger(), "Goal accepted, task execution started");
        return goal_handle;
    }
    
    /**
     * Wait for task completion with industry-standard cancellation support
     * Returns: 0 = success, 1 = failed, 2 = cancelled
     */
    int wait_for_completion(std::shared_ptr<GoalHandleMTCExecution> goal_handle) {
        auto result_future = action_client_->async_get_result(goal_handle);
        
        // Industry standard: Use shorter timeouts and check for cancellation
        const auto check_interval = std::chrono::milliseconds(100);
        auto start_time = std::chrono::steady_clock::now();
        
        while (true) {
            // Check if we should cancel (Ctrl+C was pressed)
            if (should_cancel.load()) {
                RCLCPP_INFO(get_logger(), "Cancellation requested, stopping execution...");
                cancel_goal(goal_handle);
                return 2;  // Cancelled
            }

            // Check if result is ready with short timeout (allows for cancellation)
            auto status = rclcpp::spin_until_future_complete(shared_from_this(), result_future, check_interval);

            if (status == rclcpp::FutureReturnCode::SUCCESS) {
                auto result = result_future.get();
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    return 0;  // Success
                } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    return 2;  // Cancelled
                } else {
                    return 1;  // Failed
                }
            }

            // Check for overall timeout
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > timeout_) {
                RCLCPP_ERROR(get_logger(), "Task execution timed out");
                return 1;  // Failed
            }
        }
    }

    /**
     * Callback for receiving progress updates during task execution
     */
    void feedback_callback(GoalHandleMTCExecution::SharedPtr,
                          const std::shared_ptr<const MTCExecution::Feedback> feedback) {
        RCLCPP_INFO(get_logger(), 
                   "Progress: %.1f%% - Step %d - Action: %s - Status: %s - Gripper: %s",
                   feedback->progress_percentage,
                   feedback->current_step,
                   feedback->current_action.c_str(),
                   feedback->status_message.c_str(),
                   feedback->current_gripper.c_str());
    }
    
    /**
     * Callback for receiving the final result when task completes
     */
    void result_callback(const GoalHandleMTCExecution::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "Task completed successfully! (%d/%d steps)", 
                           result.result->completed_steps, result.result->total_steps);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "Task aborted: %s", result.result->error_message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(get_logger(), "Task canceled");
                break;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown result code");
                break;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Parse command line arguments
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <json_file_path> [robot_ip] [timeout_seconds]" << std::endl;
        std::cout << "Example: " << argv[0] << " ./script.json 192.168.1.101 300" << std::endl;
        std::cout << "Press Ctrl+C to cancel execution at any time" << std::endl;
        return 1;
    }
    
    std::string json_file = argv[1];
    std::string robot_ip = (argc > 2) ? argv[2] : "192.168.1.101";
    int timeout_seconds = (argc > 3) ? std::stoi(argv[3]) : 300;
    
    // Industry standard: Setup signal handler for Ctrl+C
    signal(SIGINT, [](int) {
        should_cancel.store(true);
    });
    
    // Create the action client with timeout
    auto client = std::make_shared<MTCActionClient>(std::chrono::seconds(timeout_seconds));
    
    // Wait for the action server to become available
    RCLCPP_INFO(client->get_logger(), "Waiting for action server...");
    if (!client->wait_for_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(client->get_logger(), "Action server not available after 10 seconds");
        rclcpp::shutdown();
        return 1;
    }
    
    // Execute the task
    RCLCPP_INFO(client->get_logger(), "Starting task execution. Press Ctrl+C to cancel at any time.");
    int exit_code = client->execute_task(json_file, robot_ip);

    // Handle different exit conditions
    if (exit_code == 0) {
        RCLCPP_INFO(client->get_logger(), "Task completed successfully");
    } else if (exit_code == 2) {
        RCLCPP_WARN(client->get_logger(), "Task was cancelled by user");
    } else {
        RCLCPP_ERROR(client->get_logger(), "Task failed");
    }

    rclcpp::shutdown();
    return exit_code;
}
