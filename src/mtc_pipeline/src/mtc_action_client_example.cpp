#include "mtc_pipeline/action/mtc_execution.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <filesystem>
#include <optional>

using MTCExecution = mtc_pipeline::action::MTCExecution;
using GoalHandleMTCExecution = rclcpp_action::ClientGoalHandle<MTCExecution>;

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
    
    rclcpp_action::Client<MTCExecution>::SharedPtr get_action_client() {
        return action_client_;
    }
    
    /**
     * Execute a task from a JSON file
     * @param json_file_path Path to the JSON task file
     * @param robot_ip IP address of the robot
     * @return true if task completed successfully, false otherwise
     */
    bool execute_task(const std::string& json_file_path, const std::string& robot_ip = "192.168.1.101") {
        // Validate and read JSON file
        auto json_content = read_json_file(json_file_path);
        if (!json_content) {
            return false;
        }
        
        // Create and send goal
        auto goal_msg = create_goal(*json_content, robot_ip);
        auto goal_handle = send_goal(goal_msg);
        if (!goal_handle) {
            return false;
        }
        
        // Wait for completion
        return wait_for_completion(goal_handle);
    }

private:
    /**
     * Read and validate JSON file
     */
    std::optional<std::string> read_json_file(const std::string& file_path) {
        // Check if file exists and is readable
        if (!std::filesystem::exists(file_path)) {
            RCLCPP_ERROR(get_logger(), "File does not exist: %s", file_path.c_str());
            return std::nullopt;
        }
        
        auto file_size = std::filesystem::file_size(file_path);
        if (file_size > 10 * 1024 * 1024) { // 10MB limit
            RCLCPP_ERROR(get_logger(), "File too large: %s (%zu bytes)", file_path.c_str(), file_size);
            return std::nullopt;
        }
        
        // Read file content
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Could not open file: %s", file_path.c_str());
            return std::nullopt;
        }
        
        std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        
        // Validate JSON structure
        try {
            nlohmann::json::parse(content);
        } catch (const nlohmann::json::exception& e) {
            RCLCPP_ERROR(get_logger(), "Invalid JSON in file %s: %s", file_path.c_str(), e.what());
            return std::nullopt;
        }
        
        return content;
    }
    
    /**
     * Create action goal from JSON content
     */
    MTCExecution::Goal create_goal(const std::string& json_content, const std::string& robot_ip) {
        MTCExecution::Goal goal;
        goal.task_script_json = json_content;
        goal.robot_ip = robot_ip;
        return goal;
    }
    
    /**
     * Send goal to action server
     */
    std::shared_ptr<GoalHandleMTCExecution> send_goal(const MTCExecution::Goal& goal) {
        RCLCPP_INFO(get_logger(), "Sending task execution goal...");
        
        auto send_goal_options = rclcpp_action::Client<MTCExecution>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&MTCActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&MTCActionClient::result_callback, this, std::placeholders::_1);
        
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
     * Wait for task completion
     */
    bool wait_for_completion(std::shared_ptr<GoalHandleMTCExecution> goal_handle) {
        auto result_future = action_client_->async_get_result(goal_handle);
        
        auto status = rclcpp::spin_until_future_complete(shared_from_this(), result_future, timeout_);
        if (status != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Failed to get result (timeout or error)");
            return false;
        }
        
        auto result = result_future.get();
        return result.code == rclcpp_action::ResultCode::SUCCEEDED;
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
        return 1;
    }
    
    std::string json_file = argv[1];
    std::string robot_ip = (argc > 2) ? argv[2] : "192.168.1.101";
    int timeout_seconds = (argc > 3) ? std::stoi(argv[3]) : 300;
    
    // Create the action client with timeout
    auto client = std::make_shared<MTCActionClient>(std::chrono::seconds(timeout_seconds));
    
    // Wait for the action server to become available
    RCLCPP_INFO(client->get_logger(), "Waiting for action server...");
    if (!client->get_action_client()->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(client->get_logger(), "Action server not available after 10 seconds");
        rclcpp::shutdown();
        return 1;
    }
    
    // Execute the task
    bool success = client->execute_task(json_file, robot_ip);
    
    rclcpp::shutdown();
    return success ? 0 : 1;
}
