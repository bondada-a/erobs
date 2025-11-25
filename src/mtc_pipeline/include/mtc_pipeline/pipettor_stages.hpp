// Pipettor operations: SUCK, EXPEL, SET_LED via custom MTC stage.

#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/action/pipettor_action.hpp"

class PipettorStages : public BaseStages {
public:
    PipettorStages(const rclcpp::Node::SharedPtr& node);
    bool run(const mtc_pipeline::action::PipettorAction::Goal& goal);
};
