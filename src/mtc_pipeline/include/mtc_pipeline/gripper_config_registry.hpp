// Gripper Configuration Registry
// Loads and provides access to gripper configurations from YAML file
// Eliminates hardcoded gripper-to-package mappings

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace mtc_pipeline {

/**
 * @brief Registry for gripper configurations loaded from YAML
 *
 * Manages gripper-specific settings including MoveIt packages and tool voltages.
 * Configurations are loaded from a YAML file at initialization, allowing new
 * grippers to be added without code changes.
 *
 * Example usage:
 * @code
 * GripperConfigRegistry registry(node, "config/grippers.yaml");
 * auto config = registry.get_config("hande");
 * if (config) {
 *     launch_moveit(config->moveit_package);
 *     set_voltage(config->tool_voltage);
 * }
 * @endcode
 */
class GripperConfigRegistry {
public:
    /**
     * @brief Configuration data for a single gripper type
     */
    struct GripperConfig {
        std::string moveit_package;  ///< ROS 2 package containing MoveIt config
        int tool_voltage;            ///< Tool voltage in volts (0, 12, or 24)
    };

    /**
     * @brief Construct registry and load configuration from file
     *
     * @param node ROS 2 node for logging (raw pointer)
     * @param config_file Path to YAML config file (relative to package share dir)
     * @throws std::runtime_error if file cannot be loaded or parsed
     */
    GripperConfigRegistry(rclcpp::Node* node,
                          const std::string& config_file);

    /**
     * @brief Get configuration for a specific gripper type
     *
     * @param gripper_type Gripper identifier (e.g., "hande", "epick")
     * @return Configuration if found, std::nullopt otherwise
     */
    std::optional<GripperConfig> get_config(const std::string& gripper_type) const;

    /**
     * @brief Check if a gripper type is registered
     *
     * @param gripper_type Gripper identifier to check
     * @return true if configuration exists, false otherwise
     */
    bool has_config(const std::string& gripper_type) const;

    /**
     * @brief Get list of all registered gripper types
     *
     * @return Vector of gripper type identifiers
     */
    std::vector<std::string> available_grippers() const;

    /**
     * @brief Get number of registered grippers
     *
     * @return Count of gripper configurations
     */
    size_t size() const { return configs_.size(); }

private:
    rclcpp::Node* node_;
    std::unordered_map<std::string, GripperConfig> configs_;

    /**
     * @brief Load configurations from YAML file
     *
     * @param config_file Path to YAML file
     * @throws std::runtime_error on parse errors
     */
    void load_from_yaml(const std::string& config_file);

    /**
     * @brief Resolve package-relative path to absolute path
     *
     * @param relative_path Path relative to package share directory
     * @return Absolute filesystem path
     */
    std::string resolve_config_path(const std::string& relative_path) const;
};

}  // namespace mtc_pipeline
