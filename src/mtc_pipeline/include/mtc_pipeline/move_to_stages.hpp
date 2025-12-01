// MoveTo stage: handles relative moves, joint poses, and named SRDF states.

#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/action/move_to_action.hpp"

class MoveToStages : public BaseStages {
public:
    MoveToStages(const rclcpp::Node::SharedPtr& node);
    bool run(const mtc_pipeline::action::MoveToAction::Goal& goal);
};
