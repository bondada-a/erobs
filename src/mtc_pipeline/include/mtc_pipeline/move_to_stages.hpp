#pragma once

#include "mtc_pipeline/base_stages.hpp"

#include <nlohmann/json.hpp>

class MoveToStages : public BaseStages {
public:
  // Constructor
  MoveToStages(const rclcpp::Node::SharedPtr& node);

  // Main orchestrator step runner
  bool run(const nlohmann::json& step, const nlohmann::json& poses);
};