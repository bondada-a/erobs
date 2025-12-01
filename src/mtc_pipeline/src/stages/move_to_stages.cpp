#include "mtc_pipeline/move_to_stages.hpp"
#include <moveit/task_constructor/stages/move_to.h>
#include <nlohmann/json.hpp>

MoveToStages::MoveToStages(const rclcpp::Node::SharedPtr& node)
    : BaseStages(node) {}

bool MoveToStages::run(const mtc_pipeline::action::MoveToAction::Goal& goal) {
    auto task = create_task_template("MoveTo Task");
    auto planner = (goal.planning_type == "cartesian")
        ? make_cartesian_planner() : make_pipeline_planner();

    // Relative move (direction + distance)
    if (!goal.direction.empty() && goal.distance != 0.0) {
        auto label = "move_" + goal.direction + "_" + std::to_string(goal.distance) + "m";
        auto stage = create_relative_move_stage(label, goal.direction, goal.distance, planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
        task.add(std::move(stage));
    }
    // Target-based move (joint pose or named state)
    else if (!goal.target.empty()) {
        nlohmann::json poses;
        try {
            poses = nlohmann::json::parse(goal.poses_json);
        } catch (const nlohmann::json::exception& e) {
            RCLCPP_ERROR(node()->get_logger(), "Failed to parse poses_json: %s", e.what());
            return false;
        }

        auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_" + goal.target, planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
        stage->setGroup(default_arm_group_name());

        if (poses.contains(goal.target)) {
            // Joint pose from JSON
            const auto& joint_pose = poses.at(goal.target);
            if (!joint_pose.is_array() || joint_pose.size() != 6) {
                RCLCPP_ERROR(node()->get_logger(), "'%s' must be array of 6 joint angles", goal.target.c_str());
                return false;
            }
            stage->setGoal(joints_from_degrees(joint_pose.get<std::vector<double>>()));
        } else {
            // Named SRDF state
            stage->setGoal(goal.target);
        }
        task.add(std::move(stage));
    }
    else {
        RCLCPP_ERROR(node()->get_logger(), "MoveTo: missing 'target' or 'direction'/'distance'");
        return false;
    }

    return load_plan_execute(task);
}
