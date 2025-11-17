#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/action/end_effector_action.hpp"

#include <nlohmann/json.hpp>
#include <string>

class EndEffectorStages : public BaseStages {
public:
    EndEffectorStages(const rclcpp::Node::SharedPtr& node);

    bool run(const mtc_pipeline::action::EndEffectorAction::Goal& goal);

private:
    std::string get_gripper_group_name(const std::string& end_effector_type);
    std::string get_goal_state_name(const std::string& end_effector_type, const std::string& action);
};
