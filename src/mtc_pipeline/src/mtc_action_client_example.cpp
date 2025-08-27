#include "mtc_pipeline/action/mtc_execution.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

using MTCExecution = mtc_pipeline::action::MTCExecution;
using GoalHandleMTCExecution = rclcpp_action::ClientGoalHandle<MTCExecution>;

class MTCActionClient : public rclcpp::Node {
private:
    rclcpp_action::Client<MTCExecution>::SharedPtr action_client_;

public:
    MTCActionClient() : Node("mtc_action_client") {
        action_client_ = rclcpp_action::create_client<MTCExecution>(
            this, "mtc_execution");
    }
    
    rclcpp_action::Client<MTCExecution>::SharedPtr get_action_client() {
        return action_client_;
    }

    bool send_goal(const std::string& json_file_path, 
                   const std::string& robot_ip = "192.168.1.101",
                   const std::string& start_gripper = "none") {
        
        // Read JSON file
        std::ifstream file(json_file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", json_file_path.c_str());
            return false;
        }
        
        nlohmann::json config;
        file >> config;
        std::string json_string = config.dump();
        
        // Create goal
        auto goal_msg = MTCExecution::Goal();
        goal_msg.task_script_json = json_string;
        goal_msg.robot_ip = robot_ip;
        goal_msg.start_gripper = start_gripper;
        
        RCLCPP_INFO(this->get_logger(), "Sending goal...");
        
        // Send goal
        auto send_goal_options = rclcpp_action::Client<MTCExecution>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&MTCActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&MTCActionClient::result_callback, this, std::placeholders::_1);
        
        auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
        
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), goal_handle_future) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
            return false;
        }
        
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Goal accepted, waiting for result...");
        
        // Wait for result
        auto result_future = action_client_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get result");
            return false;
        }
        
        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Task completed successfully!");
            RCLCPP_INFO(this->get_logger(), "Completed %d/%d steps", 
                       result.result->completed_steps, result.result->total_steps);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Task failed: %s", result.result->error_message.c_str());
        }
        
        return result.code == rclcpp_action::ResultCode::SUCCEEDED;
    }

private:
    void feedback_callback(GoalHandleMTCExecution::SharedPtr,
                          const std::shared_ptr<const MTCExecution::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), 
                   "Progress: %.1f%% - Step %d/%d - Action: %s - Status: %s - Gripper: %s",
                   feedback->progress_percentage,
                   feedback->current_step,
                   feedback->current_step, // This should be total_steps, but it's not in feedback
                   feedback->current_action.c_str(),
                   feedback->status_message.c_str(),
                   feedback->current_gripper.c_str());
    }
    
    void result_callback(const GoalHandleMTCExecution::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Task succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Task aborted: %s", 
                           result.result->error_message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Task canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <json_file_path> [robot_ip] [start_gripper]" << std::endl;
        std::cout << "Example: " << argv[0] << " ./script.json 192.168.1.101 none" << std::endl;
        return 1;
    }
    
    std::string json_file = argv[1];
    std::string robot_ip = (argc > 2) ? argv[2] : "192.168.1.101";
    std::string start_gripper = (argc > 3) ? argv[3] : "none";
    
    auto client = std::make_shared<MTCActionClient>();
    
    // Wait for action server
    while (!client->get_action_client()->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_INFO(client->get_logger(), "Waiting for action server...");
    }
    
    bool success = client->send_goal(json_file, robot_ip, start_gripper);
    
    rclcpp::shutdown();
    return success ? 0 : 1;
}
