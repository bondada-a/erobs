// Beamline configuration loader implementation

#include "mtc_pipeline/beamline_config.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace mtc_pipeline {

BeamlineConfig load_beamline_config(const std::string& yaml_file) {
    // Resolve path - if relative, use package share directory
    std::string resolved_path = yaml_file;
    if (!yaml_file.empty() && yaml_file[0] != '/') {
        // Relative path - resolve to package share directory
        try {
            std::string package_share = ament_index_cpp::get_package_share_directory("mtc_pipeline");
            resolved_path = package_share + "/" + yaml_file;
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to resolve package path for: " + yaml_file);
        }
    }

    // Check if file exists
    std::ifstream file(resolved_path);
    if (!file.good()) {
        throw std::runtime_error("Beamline config file not found: " + resolved_path);
    }

    YAML::Node config;
    try {
        config = YAML::LoadFile(resolved_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to parse beamline config: " + std::string(e.what()));
    }

    BeamlineConfig beamline;

    // Load beamline name
    if (config["beamline"]) {
        beamline.name = config["beamline"].as<std::string>();
    } else {
        throw std::runtime_error("Missing 'beamline' field in config");
    }

    // Load robot configuration
    if (!config["robot"]) {
        throw std::runtime_error("Missing 'robot' section in config");
    }

    auto robot = config["robot"];
    beamline.robot.model = robot["model"].as<std::string>();
    beamline.robot.ip = robot["ip"].as<std::string>();
    beamline.robot.arm_group = robot["arm_group"].as<std::string>("ur_arm");  // Default
    beamline.robot.ik_frame = robot["ik_frame"].as<std::string>("tool0");     // Default

    // Load gripper configurations
    if (!config["grippers"]) {
        throw std::runtime_error("Missing 'grippers' section in config");
    }

    for (const auto& gripper_node : config["grippers"]) {
        std::string name = gripper_node.first.as<std::string>();

        BeamlineConfig::GripperEntry entry;
        auto gripper_config = gripper_node.second;

        if (!gripper_config["moveit_package"]) {
            throw std::runtime_error("Missing 'moveit_package' for gripper: " + name);
        }
        entry.moveit_package = gripper_config["moveit_package"].as<std::string>();

        if (!gripper_config["tool_voltage"]) {
            throw std::runtime_error("Missing 'tool_voltage' for gripper: " + name);
        }
        entry.tool_voltage = gripper_config["tool_voltage"].as<int>();

        // group_name is optional - will fall back to computed name if not provided
        entry.group_name = gripper_config["group_name"].as<std::string>("");

        beamline.grippers[name] = entry;
    }

    // Load available grippers list (optional - defaults to all grippers)
    if (config["available_grippers"]) {
        beamline.available_grippers = config["available_grippers"].as<std::vector<std::string>>();
    } else {
        // Default: all configured grippers are available
        for (const auto& [name, _] : beamline.grippers) {
            beamline.available_grippers.push_back(name);
        }
    }

    // Load workspace configuration
    if (config["workspace"]) {
        auto workspace = config["workspace"];
        beamline.workspace.obstacle_config =
            workspace["obstacle_config"].as<std::string>("config/beamline_scene.yaml");  // Default
    }

    // Load planning parameters (optional)
    if (config["planning"]) {
        auto planning = config["planning"];
        beamline.planning.velocity_scaling =
            planning["velocity_scaling"].as<double>(0.2);
        beamline.planning.acceleration_scaling =
            planning["acceleration_scaling"].as<double>(0.2);
    }

    return beamline;
}

}  // namespace mtc_pipeline
