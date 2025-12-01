#include "mtc_pipeline/tool_exchange_stages.hpp"
#include <moveit/task_constructor/stages/move_to.h>
#include <cmath>

namespace {
constexpr double DOCK_SPACING_METERS = 0.1524;  // 6 inches between dock stations
}

ToolExchangeStages::ToolExchangeStages(const rclcpp::Node::SharedPtr& node)
    : BaseStages(node) {}

bool ToolExchangeStages::run(const mtc_pipeline::action::ToolExchangeAction::Goal& goal)
{
    nlohmann::json poses;
    try {
        poses = nlohmann::json::parse(goal.poses_json);
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(node()->get_logger(), "Failed to parse poses_json: %s", e.what());
        return false;
    }

    // Validate state transitions
    if (goal.operation == "load" && goal.current_attached_gripper != "none") {
        RCLCPP_ERROR(node()->get_logger(), "Cannot load %s: %s already attached",
            goal.gripper.c_str(), goal.current_attached_gripper.c_str());
        return false;
    }
    if (goal.operation == "dock" && goal.current_attached_gripper != goal.gripper) {
        RCLCPP_ERROR(node()->get_logger(), "Cannot dock %s: %s is attached",
            goal.gripper.c_str(), goal.current_attached_gripper.c_str());
        return false;
    }

    auto task = create_task_template(goal.operation == "load" ? "Load Tool" : "Dock Tool");
    auto sampling = make_pipeline_planner();
    auto cartesian = make_cartesian_planner();

    // Helper to add relative move stages
    auto add_move = [&](const std::string& label, const std::string& dir, double dist) {
        auto stage = create_relative_move_stage(label, dir, dist, cartesian);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
        task.add(std::move(stage));
    };

    // Move to approach pose
    const auto& joint_pose = poses.at(goal.approach_pose);
    if (!joint_pose.is_array() || joint_pose.size() != 6) {
        RCLCPP_ERROR(node()->get_logger(), "'%s' must be array of 6 joint angles", goal.approach_pose.c_str());
        return false;
    }

    auto approach = std::make_unique<mtc::stages::MoveTo>("approach", sampling);
    approach->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    approach->setGroup(default_arm_group_name());
    approach->setGoal(joints_from_degrees(joint_pose.get<std::vector<double>>()));
    task.add(std::move(approach));

    // Lateral shift to align with dock (reference is dock 3)
    double offset_y = DOCK_SPACING_METERS * (3 - goal.dock_number);
    if (std::abs(offset_y) >= 1e-4) {
        add_move("shift to dock", offset_y >= 0 ? "right" : "left", std::abs(offset_y));
    }

    if (goal.operation == "load") {
        add_move("attach tool", "forward", 0.2);
        add_move("detach holder", "up", 0.15);
        add_move("retreat", "backward", 0.2);
    } else if (goal.operation == "dock") {
        add_move("align holder", "forward", 0.2);
        add_move("detach tool", "down", 0.15);
        add_move("retreat", "backward", 0.2);
    } else {
        RCLCPP_ERROR(node()->get_logger(), "Unknown operation: %s", goal.operation.c_str());
        return false;
    }

    return load_plan_execute(task);
}
