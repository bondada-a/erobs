// Beamline configuration data structure and loader
// Defines deployment-specific settings for each beamline (CMS, LIX, PDF, etc.)

#pragma once

#include <string>
#include <map>
#include <vector>

namespace mtc_pipeline {

/// @brief Complete beamline deployment configuration
struct BeamlineConfig {
    // Beamline identity
    std::string name;              // e.g., "cms", "lix", "pdf"

    // Robot configuration
    struct Robot {
        std::string model;         // e.g., "ur5e", "ur3e"
        std::string ip;            // e.g., "192.168.1.100"
        std::string arm_group;     // MoveIt planning group, e.g., "ur_arm"
        std::string ik_frame;      // IK reference frame, e.g., "tool0"
    } robot;

    // Gripper configurations
    struct GripperEntry {
        std::string moveit_package;    // MoveIt config package name
        int tool_voltage;              // Tool voltage (0, 12, or 24V)
        std::string group_name;        // MoveIt gripper group name (optional)
    };
    std::map<std::string, GripperEntry> grippers;  // gripper_name -> config

    // Available grippers at this beamline
    std::vector<std::string> available_grippers;

    // Workspace configuration
    struct Workspace {
        std::string obstacle_config;   // Path to obstacle scene YAML
    } workspace;

    // Planning parameters (optional)
    struct Planning {
        double velocity_scaling = 0.2;
        double acceleration_scaling = 0.2;
    } planning;
};

/// @brief Load beamline configuration from YAML file
/// @param yaml_file Path to beamline configuration YAML
/// @return Populated BeamlineConfig structure
/// @throws std::runtime_error if file not found or invalid format
BeamlineConfig load_beamline_config(const std::string& yaml_file);

}  // namespace mtc_pipeline
