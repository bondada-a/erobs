// Loads gripper configurations from YAML file.
// Maps gripper types to their MoveIt packages and tool voltages.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace mtc_pipeline {

// Forward declaration
struct BeamlineConfig;

class GripperConfigRegistry {
public:
    struct GripperConfig {
        std::string moveit_package;  // ROS 2 package with MoveIt config
        int tool_voltage;            // Voltage: 0, 12, or 24V
    };

    /// @brief Load gripper configurations from YAML file
    GripperConfigRegistry(rclcpp::Node* node, const std::string& config_file);

    /// @brief Load gripper configurations from BeamlineConfig
    GripperConfigRegistry(rclcpp::Node* node, const struct BeamlineConfig& beamline_config);

    /// @brief Get configuration for specified gripper type
    std::optional<GripperConfig> get_config(const std::string& gripper_type) const;

    /// @brief Check if configuration exists for gripper type
    bool has_config(const std::string& gripper_type) const;

    /// @brief Get list of all configured gripper types
    std::vector<std::string> available_grippers() const;

    /// @brief Get number of loaded gripper configurations
    size_t size() const { return configs_.size(); }

private:
    rclcpp::Node* node_;
    std::unordered_map<std::string, GripperConfig> configs_;

    /// @brief Parse YAML file and populate configuration map
    void load_from_yaml(const std::string& config_file);

    /// @brief Resolve relative path to absolute package share directory path
    std::string resolve_config_path(const std::string& relative_path) const;
};

}
