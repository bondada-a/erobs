// Planning scene obstacle loader from YAML configuration.

#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace mtc_pipeline {

// Load obstacles from YAML into MoveIt planning scene.
// Supports: box, cylinder, sphere primitives with position + RPY orientation.
bool loadPlanningSceneObstacles(const rclcpp::Logger& logger, const std::string& yaml_path);

// Convert roll-pitch-yaw (radians) to quaternion.
geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw);

}
