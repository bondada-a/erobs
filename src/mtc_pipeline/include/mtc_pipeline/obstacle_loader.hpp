// Planning scene obstacle loader from YAML configuration.

#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace mtc_pipeline {

/// @brief Load obstacles from YAML into MoveIt planning scene
bool loadPlanningSceneObstacles(const rclcpp::Logger& logger, const std::string& yaml_path);

/// @brief Convert roll-pitch-yaw angles to quaternion
geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw);

}
