// tool_exchange_stages.hpp
#pragma once
#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/action/tool_exchange_action.hpp"

#include <nlohmann/json.hpp>

class ToolExchangeStages : public BaseStages {
public:
  ToolExchangeStages(const rclcpp::Node::SharedPtr& node);
  bool run(const mtc_pipeline::action::ToolExchangeAction::Goal& goal, const nlohmann::json& poses);
};
