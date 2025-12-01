// MoveTo stage: handles relative moves, joint poses, and named SRDF states.

#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/action/move_to_action.hpp"

class MoveToStages : public BaseStages {
public:
    /// @brief Construct MoveTo stages with ROS 2 node
    MoveToStages(const rclcpp::Node::SharedPtr& node);

    /// @brief Execute move-to task from goal specification
    bool run(const mtc_pipeline::action::MoveToAction::Goal& goal);
};
