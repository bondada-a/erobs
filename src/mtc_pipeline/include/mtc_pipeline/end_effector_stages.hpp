#pragma once

#include "mtc_pipeline/base_stages.hpp"

#include <nlohmann/json.hpp>
#include <functional>
#include <string>

class EndEffectorStages : public BaseStages {
public:
    EndEffectorStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config);

    bool run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node);
    bool run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node,
             std::function<bool()> should_cancel);
};
