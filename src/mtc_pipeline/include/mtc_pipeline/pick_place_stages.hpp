// Pick and place sequence: approach → grip → retreat → approach → release → retreat

#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/action/pick_place_action.hpp"
#include <nlohmann/json.hpp>

class PickPlaceStages : public BaseStages {
public:
    PickPlaceStages(const rclcpp::Node::SharedPtr& node);
    bool run(const mtc_pipeline::action::PickPlaceAction::Goal& goal);

private:
    std::unique_ptr<mtc::Stage> make_move_to_named_stage(
        const std::string& label,
        const std::string& pose_key,
        const nlohmann::json& poses,
        const mtc::solvers::PlannerInterfacePtr& planner);

    std::unique_ptr<mtc::Stage> make_gripper_stage(
        const std::string& label,
        const mtc::solvers::PlannerInterfacePtr& planner,
        bool open,
        const std::string& gripper_type);
};
