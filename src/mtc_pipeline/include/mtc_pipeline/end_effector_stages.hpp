// End effector control: open/close grippers via SRDF states.

#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/action/end_effector_action.hpp"

class EndEffectorStages : public BaseStages {
public:
    EndEffectorStages(const rclcpp::Node::SharedPtr& node);
    bool run(const mtc_pipeline::action::EndEffectorAction::Goal& goal);
};
