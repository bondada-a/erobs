#include "mtc_pipeline/gripper_config_registry.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdexcept>

namespace mtc_pipeline {

GripperConfigRegistry::GripperConfigRegistry(
    rclcpp::Node* node,
    const std::string& config_file)
    : node_(node)
{
    try {
        std::string resolved_path = resolve_config_path(config_file);
        load_from_yaml(resolved_path);

        RCLCPP_INFO(node_->get_logger(),
                    "Loaded %zu gripper configuration(s) from %s",
                    configs_.size(), resolved_path.c_str());

        // Log available grippers
        auto grippers = available_grippers();
        if (!grippers.empty()) {
            std::string gripper_list;
            for (size_t i = 0; i < grippers.size(); ++i) {
                gripper_list += grippers[i];
                if (i < grippers.size() - 1) gripper_list += ", ";
            }
            RCLCPP_INFO(node_->get_logger(), "Available grippers: %s", gripper_list.c_str());
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to load gripper configuration: %s", e.what());
        throw;
    }
}

std::optional<GripperConfigRegistry::GripperConfig>
GripperConfigRegistry::get_config(const std::string& gripper_type) const
{
    auto it = configs_.find(gripper_type);
    if (it != configs_.end()) {
        return it->second;
    }
    return std::nullopt;
}

bool GripperConfigRegistry::has_config(const std::string& gripper_type) const
{
    return configs_.find(gripper_type) != configs_.end();
}

std::vector<std::string> GripperConfigRegistry::available_grippers() const
{
    std::vector<std::string> grippers;
    grippers.reserve(configs_.size());

    for (const auto& [name, config] : configs_) {
        grippers.push_back(name);
    }

    // Sort for consistent output
    std::sort(grippers.begin(), grippers.end());
    return grippers;
}

void GripperConfigRegistry::load_from_yaml(const std::string& config_file)
{
    // Verify file exists
    std::ifstream file_check(config_file);
    if (!file_check.good()) {
        throw std::runtime_error("Config file not found: " + config_file);
    }
    file_check.close();

    // Parse YAML
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_file);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML parse error: " + std::string(e.what()));
    }

    // Validate top-level structure
    if (!yaml["grippers"] || !yaml["grippers"].IsMap()) {
        throw std::runtime_error("Config file must contain 'grippers' map");
    }

    // Load each gripper configuration
    const YAML::Node& grippers_node = yaml["grippers"];
    for (const auto& entry : grippers_node) {
        std::string gripper_name = entry.first.as<std::string>();
        const YAML::Node& gripper_data = entry.second;

        // Validate required fields
        if (!gripper_data["moveit_package"]) {
            RCLCPP_WARN(node_->get_logger(),
                       "Skipping gripper '%s': missing 'moveit_package'",
                       gripper_name.c_str());
            continue;
        }

        if (!gripper_data["tool_voltage"]) {
            RCLCPP_WARN(node_->get_logger(),
                       "Skipping gripper '%s': missing 'tool_voltage'",
                       gripper_name.c_str());
            continue;
        }

        // Parse configuration
        GripperConfig config;
        config.moveit_package = gripper_data["moveit_package"].as<std::string>();
        config.tool_voltage = gripper_data["tool_voltage"].as<int>();

        // Validate tool voltage
        if (config.tool_voltage != 0 && config.tool_voltage != 12 && config.tool_voltage != 24) {
            RCLCPP_WARN(node_->get_logger(),
                       "Gripper '%s' has unusual tool_voltage %dV (expected 0, 12, or 24)",
                       gripper_name.c_str(), config.tool_voltage);
        }

        // Store configuration
        configs_[gripper_name] = config;

        RCLCPP_DEBUG(node_->get_logger(),
                    "Loaded gripper '%s': package=%s, voltage=%dV",
                    gripper_name.c_str(),
                    config.moveit_package.c_str(),
                    config.tool_voltage);
    }

    if (configs_.empty()) {
        throw std::runtime_error("No valid gripper configurations found in file");
    }
}

std::string GripperConfigRegistry::resolve_config_path(const std::string& relative_path) const
{
    // If already absolute path, return as-is
    if (!relative_path.empty() && relative_path[0] == '/') {
        return relative_path;
    }

    // Resolve relative to package share directory
    try {
        std::string package_share = ament_index_cpp::get_package_share_directory("mtc_pipeline");
        return package_share + "/" + relative_path;
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to resolve package path: " + std::string(e.what()));
    }
}

}  // namespace mtc_pipeline
