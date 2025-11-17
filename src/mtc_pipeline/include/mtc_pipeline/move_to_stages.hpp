#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/action/move_to_action.hpp"

#include <nlohmann/json.hpp>

class MoveToStages : public BaseStages {
public:
  // Constructor
  MoveToStages(const rclcpp::Node::SharedPtr& node);

  // Main orchestrator step runner
  bool run(const mtc_pipeline::action::MoveToAction::Goal& goal, const nlohmann::json& poses);
};
