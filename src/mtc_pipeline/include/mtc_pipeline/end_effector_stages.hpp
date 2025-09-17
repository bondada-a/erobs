#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/planner_interface.h>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <vector>
#include <map>

namespace mtc = moveit::task_constructor;

class EndEffectorStages {
public:
    EndEffectorStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config);
    
    bool run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node);
    bool run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node, 
             std::function<bool()> should_cancel);
    
    // End effector control methods
    bool controlGripper(const std::string& action, double position = 0.0, double force = 0.0);
    bool controlVacuum(const std::string& action, const std::string& end_effector_type, double pressure = 0.0);
    bool controlCustom(const std::string& end_effector_type, const std::string& action, const nlohmann::json& params);

private:
    rclcpp::Node::SharedPtr node_;
    nlohmann::json config_;
    
    // Service clients for different end effectors (created on-demand)
    // rclcpp::Client<control_msgs::msg::GripperCommand>::SharedPtr gripper_client_;
    // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr vacuum_client_;
    
    // Helper methods
    bool waitForService(const std::string& service_name, std::chrono::seconds timeout = std::chrono::seconds(5));
    std::string getEndEffectorType();
    std::string getGripperActionTopic();
    std::string getVacuumActionTopic();
    
    // Configuration
    void loadEndEffectorConfig();
    std::map<std::string, std::string> end_effector_config_;
};
