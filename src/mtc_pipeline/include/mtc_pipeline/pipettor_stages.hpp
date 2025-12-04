// Pipettor operations: SUCK, EXPEL, SET_LED via custom MTC stage.

#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/action/pipettor_action.hpp"

class PipettorStages : public BaseStages {
public:
    /// @brief Construct Pipettor stages with ROS 2 node
    PipettorStages(const rclcpp::Node::SharedPtr& node);

    /// @brief Execute pipettor operation from goal specification
    bool run(const mtc_pipeline::action::PipettorAction::Goal& goal);
};
