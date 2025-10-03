#pragma once

#include "mtc_pipeline/base_stages.hpp"

#include <nlohmann/json.hpp>

class EndEffectorStages : public BaseStages {
public:
    EndEffectorStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config);

    bool run(const nlohmann::json& step, const nlohmann::json& poses);
};
