// Loads gripper configurations from YAML file.
// Maps gripper types to their MoveIt packages and tool voltages.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace mtc_pipeline {

class GripperConfigRegistry {
public:
    struct GripperConfig {
        std::string moveit_package;  // ROS 2 package with MoveIt config
        int tool_voltage;            // Voltage: 0, 12, or 24V
    };

    // Loads configuration from YAML file (path relative to package share dir)
    GripperConfigRegistry(rclcpp::Node* node, const std::string& config_file);

    // Query interface
    std::optional<GripperConfig> get_config(const std::string& gripper_type) const;
    bool has_config(const std::string& gripper_type) const;
    std::vector<std::string> available_grippers() const;
    size_t size() const { return configs_.size(); }

private:
    rclcpp::Node* node_;
    std::unordered_map<std::string, GripperConfig> configs_;

    // YAML parsing and path resolution
    void load_from_yaml(const std::string& config_file);
    std::string resolve_config_path(const std::string& relative_path) const;
};

}
