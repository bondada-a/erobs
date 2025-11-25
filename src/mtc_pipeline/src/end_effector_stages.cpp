#include "mtc_pipeline/end_effector_stages.hpp"
#include <moveit/task_constructor/stages/move_to.h>

namespace {
// Derives MoveIt group name from gripper type
std::string get_gripper_group(const std::string& type) {
    if (type == "pipettor") return "";  // No movable joints
    return type + "_gripper";  // hande -> hande_gripper, epick -> epick_gripper
}
}  // namespace

EndEffectorStages::EndEffectorStages(const rclcpp::Node::SharedPtr& node)
    : BaseStages(node) {}

bool EndEffectorStages::run(const mtc_pipeline::action::EndEffectorAction::Goal& goal)
{
    std::string group = get_gripper_group(goal.end_effector_type);
    if (group.empty()) {
        RCLCPP_ERROR(node()->get_logger(), "Unknown end effector: %s", goal.end_effector_type.c_str());
        return false;
    }

    RCLCPP_INFO(node()->get_logger(), "End effector: %s -> %s",
        goal.end_effector_type.c_str(), goal.end_effector_action.c_str());

    auto task = create_task_template(goal.end_effector_action, group);
    auto stage = std::make_unique<mtc::stages::MoveTo>(goal.end_effector_action, make_joint_interpolation_planner());
    stage->setGroup(group);
    stage->setGoal(goal.end_effector_action);  // Action name is the SRDF state
    task.add(std::move(stage));

    return load_plan_execute(task);
}
