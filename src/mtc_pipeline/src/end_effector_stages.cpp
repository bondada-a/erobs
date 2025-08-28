#include "mtc_pipeline/end_effector_stages.hpp"
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <memory>
#include <string>

namespace mtc = moveit::task_constructor;

EndEffectorStages::EndEffectorStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
    : node_(node), config_(config) {
    loadEndEffectorConfig();
}

// Load hardcoded end effector configuration
void EndEffectorStages::loadEndEffectorConfig() {
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

// Control gripper using action client
bool EndEffectorStages::controlGripper(const std::string& action, double position, double force) {
    std::string end_effector_type = getEndEffectorType();
    
    if (end_effector_type == "hande") {
        auto gripper_action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
            node_, getGripperActionTopic());
        
        if (!gripper_action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available");
            return false;
        }
        
        // Set target position based on action
        double target_position;
        if (action == "open") {
            target_position = std::stod(end_effector_config_["gripper_open_position"]);
        } else if (action == "close") {
            target_position = std::stod(end_effector_config_["gripper_close_position"]);
        } else {
            target_position = position;
        }
        
        double target_force = (force > 0.0) ? force : std::stod(end_effector_config_["gripper_force"]);
        
        // Create and send goal
        auto goal = control_msgs::action::GripperCommand::Goal();
        goal.command.position = target_position;
        goal.command.max_effort = target_force;
        
        auto goal_handle_future = gripper_action_client->async_send_goal(goal);
        
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
        
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Unknown end effector type: %s", end_effector_type.c_str());
        return false;
    }
}

// Control vacuum using service
bool EndEffectorStages::controlVacuum(const std::string& action, double /* pressure */) {
    std::string end_effector_type = getEndEffectorType();
    
    if (end_effector_type == "epick") {
        static auto vacuum_service_client = node_->create_client<std_srvs::srv::SetBool>(getVacuumActionTopic());
        
        if (!vacuum_service_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Vacuum service not available");
            return false;
        }
        
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = (action == "on");
        
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

// Execute end effector control task
bool EndEffectorStages::run(const nlohmann::json& step, const nlohmann::json& /* poses */, rclcpp::Node::SharedPtr node) {
    std::string end_effector_type = step.value("end_effector_type", getEndEffectorType());
    std::string action = step.value("end_effector_action", "");
    
    RCLCPP_INFO(node->get_logger(), "End effector control: type=%s, action=%s", end_effector_type.c_str(), action.c_str());
    
    // Handle gripper control
    if (end_effector_type == "hande" || end_effector_type == "gripper") {
        // Create MTC task for gripper control
        moveit::task_constructor::Task task;
        task.stages()->setName("Gripper Control Task");
        
        const std::string hand_group_name = "hande_gripper";
        
        // Setup planner
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        interpolation_planner->setMaxVelocityScalingFactor(0.2);
        interpolation_planner->setMaxAccelerationScalingFactor(0.2);
        
        // Add current state
        task.add(std::make_unique<mtc::stages::CurrentState>("current"));
        
        // Set goal state based on action
        std::string goal_state;
        if (action == "open") {
            goal_state = "hande_open";
        } else if (action == "close") {
            goal_state = "hande_closed";
        } else {
            RCLCPP_ERROR(node->get_logger(), "Unknown gripper action: %s", action.c_str());
            return false;
        }
        
        // Create and add gripper stage
        auto gripper_stage = std::make_unique<mtc::stages::MoveTo>("gripper_control", interpolation_planner);
        gripper_stage->setGroup(hand_group_name);
        gripper_stage->setGoal(goal_state);
        task.add(std::move(gripper_stage));
        
        // Initialize and execute task
        try {
            task.loadRobotModel(node);
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
        // Handle vacuum control
        double pressure = step.value("pressure", 0.0);
        return controlVacuum(action, pressure);
        
    } else {
        // Handle custom end effectors
        nlohmann::json params = step.value("params", nlohmann::json::object());
        RCLCPP_WARN(node->get_logger(), "Custom end effector control not implemented for type: %s", end_effector_type.c_str());
        return false;
    }
}
