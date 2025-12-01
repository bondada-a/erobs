// Tool exchange: load/dock grippers at magnetic holder stations.

#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/action/tool_exchange_action.hpp"
#include <nlohmann/json.hpp>

class ToolExchangeStages : public BaseStages {
public:
    /// @brief Construct ToolExchange stages with ROS 2 node
    ToolExchangeStages(const rclcpp::Node::SharedPtr& node);

    /// @brief Execute tool exchange operation from goal specification
    bool run(const mtc_pipeline::action::ToolExchangeAction::Goal& goal);
};
