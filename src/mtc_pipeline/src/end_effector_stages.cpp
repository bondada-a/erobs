#include "mtc_pipeline/end_effector_stages.hpp"
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <future>
#include <chrono>

namespace mtc = moveit::task_constructor;

EndEffectorStages::EndEffectorStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
    : node_(node), config_(config) {
    loadEndEffectorConfig();
}

void EndEffectorStages::loadEndEffectorConfig() {
    // Hardcoded end effector configuration for the specific hardware setup
    end_effector_config_["type"] = "hande";
    end_effector_config_["gripper_topic"] = "/gripper_action_controller/gripper_cmd";
    end_effector_config_["gripper_open_position"] = "0.025";
    end_effector_config_["gripper_close_position"] = "0.0";
    end_effector_config_["gripper_force"] = "100.0";
    end_effector_config_["vacuum_topic"] = "/vacuum_control";
    end_effector_config_["vacuum_pressure"] = "0.8";
}

std::string EndEffectorStages::getEndEffectorType() {
    return end_effector_config_["type"];
}

std::string EndEffectorStages::getGripperActionTopic() {
    return end_effector_config_["gripper_topic"];
}

std::string EndEffectorStages::getVacuumActionTopic() {
    return end_effector_config_["vacuum_topic"];
}

bool EndEffectorStages::waitForService(const std::string& service_name, std::chrono::seconds timeout) {
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        if (node_->get_service_names_and_types().find(service_name) != node_->get_service_names_and_types().end()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

bool EndEffectorStages::controlGripper(const std::string& action, double position, double force) {
    std::string end_effector_type = getEndEffectorType();
    RCLCPP_INFO(node_->get_logger(), "Controlling gripper: %s, action: %s", end_effector_type.c_str(), action.c_str());
    
    if (end_effector_type == "hande") {
        // For hande gripper, use action client
        std::string topic = getGripperActionTopic();
        RCLCPP_INFO(node_->get_logger(), "Connecting to gripper action server at: %s", topic.c_str());
        
        // Create action client
        auto gripper_action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
            node_, topic);
        
        // Wait for action server with longer timeout
        RCLCPP_INFO(node_->get_logger(), "Waiting for gripper action server...");
        if (!gripper_action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available at %s", topic.c_str());
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "Gripper action server is available!");
        
        // Set position based on action
        double target_position;
        if (action == "open") {
            target_position = std::stod(end_effector_config_["gripper_open_position"]);
        } else if (action == "close") {
            target_position = std::stod(end_effector_config_["gripper_close_position"]);
        } else {
            target_position = position; // Use provided position
        }
        
        // Set force
        double target_force = (force > 0.0) ? force : std::stod(end_effector_config_["gripper_force"]);
        
        // Create goal
        auto goal = control_msgs::action::GripperCommand::Goal();
        goal.command.position = target_position;
        goal.command.max_effort = target_force;
        
        // Send goal
        auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::WrappedResult& result) {
            if (result.result->reached_goal) {
                RCLCPP_INFO(node_->get_logger(), "Gripper action completed successfully");
            } else {
                RCLCPP_WARN(node_->get_logger(), "Gripper action did not reach goal");
            }
        };
        
        auto goal_handle_future = gripper_action_client->async_send_goal(goal, send_goal_options);
        
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to send gripper goal");
            return false;
        }
        
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Gripper goal was rejected");
            return false;
        }
        
        // Wait for result
        auto result_future = gripper_action_client->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Gripper action timed out");
            return false;
        }
        
        auto result = result_future.get();
        return result.result->reached_goal;
        
    } else if (end_effector_type == "epick") {
        // For epick, we might need different control method
        RCLCPP_WARN(node_->get_logger(), "Epick gripper control not yet implemented");
        return false;
        
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Unknown end effector type: %s", end_effector_type.c_str());
        return false;
    }
}

bool EndEffectorStages::controlVacuum(const std::string& action, double pressure) {
    std::string end_effector_type = getEndEffectorType();
    RCLCPP_INFO(node_->get_logger(), "Controlling vacuum: %s, action: %s", end_effector_type.c_str(), action.c_str());
    
    if (end_effector_type == "epick") {
        // For epick vacuum system
        std::string topic = getVacuumActionTopic();
        
        // Create service client if not already created
        static auto vacuum_service_client = node_->create_client<std_srvs::srv::SetBool>(topic);
        
        if (!vacuum_service_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Vacuum service not available at %s", topic.c_str());
            return false;
        }
        
        // Create request
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        if (action == "on") {
            request->data = true;
        } else if (action == "off") {
            request->data = false;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Invalid vacuum action: %s", action.c_str());
            return false;
        }
        
        // Send request
        auto future = vacuum_service_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Vacuum service call failed");
            return false;
        }
        
        auto response = future.get();
        return response->success;
        
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Vacuum control not supported for end effector type: %s", end_effector_type.c_str());
        return false;
    }
}

bool EndEffectorStages::controlCustom(const std::string& end_effector_type, const std::string& action, const nlohmann::json& params) {
    RCLCPP_INFO(node_->get_logger(), "Custom end effector control: %s, action: %s", end_effector_type.c_str(), action.c_str());
    
    // This is a placeholder for custom end effector control
    // Users can extend this method to handle their specific end effectors
    RCLCPP_WARN(node_->get_logger(), "Custom end effector control not implemented for type: %s", end_effector_type.c_str());
    return false;
}

bool EndEffectorStages::run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node) {
    // Parse the step configuration
    std::string end_effector_type = step.value("end_effector_type", getEndEffectorType());
    std::string action = step.value("end_effector_action", "");
    
    RCLCPP_INFO(node->get_logger(), "End effector control: type=%s, action=%s", end_effector_type.c_str(), action.c_str());
    
    // Handle different end effector types
    if (end_effector_type == "hande" || end_effector_type == "gripper") {
        // Create MTC task for gripper control
        moveit::task_constructor::Task task;
        task.stages()->setName("Gripper Control Task");
        task.loadRobotModel(node);
        
        const std::string hand_group_name = "hande_gripper";
        
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        interpolation_planner->setMaxVelocityScalingFactor(0.2);
        interpolation_planner->setMaxAccelerationScalingFactor(0.2);
        
        // Add current state
        task.add(std::make_unique<mtc::stages::CurrentState>("current"));
        
        // Create gripper stage using MTC
        std::string goal_state;
        if (action == "open") {
            goal_state = "hande_open";
        } else if (action == "close") {
            goal_state = "hande_closed";
        } else {
            RCLCPP_ERROR(node->get_logger(), "Unknown gripper action: %s", action.c_str());
            return false;
        }
        
        auto gripper_stage = std::make_unique<mtc::stages::MoveTo>("gripper_control", interpolation_planner);
        gripper_stage->setGroup(hand_group_name);
        gripper_stage->setGoal(goal_state);
        task.add(std::move(gripper_stage));
        
        try {
            task.init();
        } catch (const moveit::task_constructor::InitStageException& e) {
            RCLCPP_ERROR(node->get_logger(), "Stage initialization failed: %s", e.what());
            return false;
        }
        
        if (!task.plan(5)) {
            RCLCPP_ERROR(node->get_logger(), "Task planning failed");
            return false;
        }
        
        if (task.solutions().empty()) {
            RCLCPP_ERROR(node->get_logger(), "No solutions found to execute");
            return false;
        }
        
        auto result = task.execute(*task.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Task execution failed");
            return false;
        }
        
        RCLCPP_INFO(node->get_logger(), "End effector control successful: %s %s", end_effector_type.c_str(), action.c_str());
        return true;
        
    } else if (end_effector_type == "epick" || end_effector_type == "vacuum") {
        // For vacuum, we still need to use service calls since MTC doesn't have vacuum stages
        double pressure = step.value("pressure", 0.0);
        return controlVacuum(action, pressure);
        
    } else {
        // Handle custom end effectors
        nlohmann::json params = step.value("params", nlohmann::json::object());
        return controlCustom(end_effector_type, action, params);
    }
}
