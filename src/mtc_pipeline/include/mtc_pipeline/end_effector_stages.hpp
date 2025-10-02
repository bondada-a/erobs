#pragma once

#include "mtc_pipeline/base_stages.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <map>

class EndEffectorStages : public BaseStages {
public:
    EndEffectorStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config);

    bool run(const nlohmann::json& step, const nlohmann::json& poses);

private:
    // Gripper configuration
    struct GripperConfig {
        std::string group_name;
        std::map<std::string, std::string> action_to_state;
    };

    // Initialize gripper configurations from SRDF definitions
    void initializeGripperConfigs();

    // Gripper configurations - populated in constructor
    std::map<std::string, GripperConfig> gripper_configs_;
};
