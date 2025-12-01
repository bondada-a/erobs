#include "mtc_pipeline/pick_place_stages.hpp"
#include "mtc_pipeline/gripper_utils.hpp"

#include <moveit/task_constructor/stages/move_to.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

// Note: Helper functions moved to eliminate duplication:
//   - Gripper utilities -> gripper_utils.hpp
//   - Wrist constraint -> BaseStages::create_wrist3_level_constraint()

PickPlaceStages::PickPlaceStages(const rclcpp::Node::SharedPtr& node)
    : BaseStages(node) {}

std::unique_ptr<mtc::Stage> PickPlaceStages::make_move_to_named_stage(
    const std::string& label,
    const std::string& pose_key,
    const nlohmann::json& poses,
    const mtc::solvers::PlannerInterfacePtr& planner)
{
    const auto& joint_pose = poses.at(pose_key);
    if (!joint_pose.is_array() || joint_pose.size() != 6) {
        RCLCPP_ERROR(node()->get_logger(), "'%s' must be array of 6 joint angles", pose_key.c_str());
        return nullptr;
    }

    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(default_arm_group_name());
    stage->setGoal(joints_from_degrees(joint_pose.get<std::vector<double>>()));
    return stage;
}

std::unique_ptr<mtc::Stage> PickPlaceStages::make_gripper_stage(
    const std::string& label,
    const mtc::solvers::PlannerInterfacePtr& planner,
    bool open,
    const std::string& gripper_type)
{
    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->setGroup(mtc_pipeline::gripper_utils::get_group_name(gripper_type));
    stage->setGoal(mtc_pipeline::gripper_utils::get_state_name(gripper_type, open));
    return stage;
}

bool PickPlaceStages::run(const mtc_pipeline::action::PickPlaceAction::Goal& goal)
{
    nlohmann::json poses;
    try {
        poses = nlohmann::json::parse(goal.poses_json);
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(node()->get_logger(), "Failed to parse poses_json: %s", e.what());
        return false;
    }

    auto task = create_task_template("Pick and Place");
    auto pipeline = make_pipeline_planner();
    auto gripper_planner = make_joint_interpolation_planner();

    // Helper to add constrained move stage
    auto add_constrained_move = [&](const std::string& label, const std::string& pose_key) {
        auto stage = make_move_to_named_stage(label, pose_key, poses, pipeline);
        if (!stage) return false;
        if (auto* move = dynamic_cast<mtc::stages::MoveTo*>(stage.get())) {
            move->setPathConstraints(create_wrist3_level_constraint());
        }
        task.add(std::move(stage));
        return true;
    };

    // Pick sequence
    task.add(make_gripper_stage("open gripper", gripper_planner, true, goal.gripper));
    if (!add_constrained_move("pickup approach", goal.pick_approach)) return false;
    if (!add_constrained_move("pickup", goal.pick_target)) return false;
    task.add(make_gripper_stage("close gripper", gripper_planner, false, goal.gripper));
    if (!add_constrained_move("pickup retreat", goal.pick_approach)) return false;

    // Place sequence (no wrist constraint - more flexibility needed)
    auto add_move = [&](const std::string& label, const std::string& pose_key) {
        auto stage = make_move_to_named_stage(label, pose_key, poses, pipeline);
        if (!stage) return false;
        task.add(std::move(stage));
        return true;
    };

    if (!add_move("place approach", goal.place_approach)) return false;
    if (!add_move("place", goal.place_target)) return false;
    task.add(make_gripper_stage("release", gripper_planner, true, goal.gripper));
    if (!add_move("place retreat", goal.place_approach)) return false;

    // Return home
    auto home = std::make_unique<mtc::stages::MoveTo>("return home", pipeline);
    home->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    home->setGroup(default_arm_group_name());
    home->setGoal("moveit_home");
    task.add(std::move(home));

    return load_plan_execute(task);
}
