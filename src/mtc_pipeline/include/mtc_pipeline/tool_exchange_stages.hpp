// tool_exchange_stages.hpp
#pragma once
#include "mtc_pipeline/base_stages.hpp"

#include <nlohmann/json.hpp>

class ToolExchangeStages : public BaseStages {
public:
  ToolExchangeStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config);
  bool run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node);
};
