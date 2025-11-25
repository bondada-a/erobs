#include "mtc_pipeline/vision_pick_place_stages.hpp"
#include "mtc_pipeline/config/gripper_config.hpp"

#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace {

moveit_msgs::msg::Constraints createWrist3Constraint() {
    moveit_msgs::msg::Constraints c;
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = "wrist_3_joint";
    jc.position = 0.0;
    jc.tolerance_above = 0.01;
    jc.tolerance_below = 0.01;
    jc.weight = 1.0;
    c.joint_constraints.push_back(jc);
    return c;
}

}  // namespace

VisionPickPlaceStages::VisionPickPlaceStages(const rclcpp::Node::SharedPtr& node)
    : BaseStages(node)
{
    vision_ = std::make_shared<VisionStages>(node);
}

bool VisionPickPlaceStages::run(const mtc_pipeline::action::VisionPickPlaceAction::Goal& goal)
{
    // Parse grasp offset (default: 5cm above, 180° rotation)
    nlohmann::json grasp_offset;
    if (!goal.grasp_offset_json.empty()) {
        try {
            grasp_offset = nlohmann::json::parse(goal.grasp_offset_json);
        } catch (const nlohmann::json::exception& e) {
            RCLCPP_ERROR(node()->get_logger(), "Invalid grasp_offset_json: %s", e.what());
            return false;
        }
    } else {
        grasp_offset = nlohmann::json::parse(R"({"x":0,"y":0,"z":0.05,"rpy":[0,3.14159,0]})");
    }

    RCLCPP_INFO(node()->get_logger(), "Vision pick/place: pick_tag=%d, place_tag=%d, gripper=%s",
        goal.pick_tag_id, goal.place_tag_id, goal.gripper.c_str());

    // Detect pick target
    auto pick_tag_pose = vision_->detect_and_transform_tag(goal.pick_tag_id, 10.0);
    if (!pick_tag_pose) {
        RCLCPP_ERROR(node()->get_logger(), "Failed to detect pick tag %d", goal.pick_tag_id);
        return false;
    }

    // Compute pick poses (minimal offset approach like vision_moveto)
    geometry_msgs::msg::PoseStamped grasp_pose = *pick_tag_pose;
    grasp_pose.pose.position.z += 0.02;  // 2cm above tag
    grasp_pose.pose.orientation.x = 0.0;
    grasp_pose.pose.orientation.y = 1.0;  // Point down
    grasp_pose.pose.orientation.z = 0.0;
    grasp_pose.pose.orientation.w = 0.0;

    auto pick_approach = grasp_pose;
    pick_approach.pose.position.z += goal.approach_offset;

    auto pick_retreat = grasp_pose;
    pick_retreat.pose.position.z += goal.retreat_offset;

    RCLCPP_INFO(node()->get_logger(), "Pick: grasp=[%.3f,%.3f,%.3f], approach=+%.2fm, retreat=+%.2fm",
        grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z,
        goal.approach_offset, goal.retreat_offset);

    // Compute place poses
    geometry_msgs::msg::PoseStamped place_pose, place_approach, place_retreat;

    if (goal.place_tag_id >= 0) {
        auto place_tag_pose = vision_->detect_and_transform_tag(goal.place_tag_id, 10.0);
        if (!place_tag_pose) {
            RCLCPP_ERROR(node()->get_logger(), "Failed to detect place tag %d", goal.place_tag_id);
            return false;
        }
        place_pose = compute_grasp_pose(*place_tag_pose, grasp_offset);
        place_approach = compute_offset_pose(place_pose, goal.approach_offset);
        place_retreat = compute_offset_pose(place_pose, goal.retreat_offset);
    } else {
        // Default place position
        place_pose.header.frame_id = "base_link";
        place_pose.pose.position.x = 0.4;
        place_pose.pose.position.y = 0.3;
        place_pose.pose.position.z = 0.15;
        place_pose.pose.orientation.x = 0.0;
        place_pose.pose.orientation.y = 1.0;
        place_pose.pose.orientation.z = 0.0;
        place_pose.pose.orientation.w = 0.0;
        place_approach = compute_offset_pose(place_pose, goal.approach_offset);
        place_retreat = compute_offset_pose(place_pose, goal.retreat_offset);
    }

    // Build MTC task
    auto task = create_task_template("Vision Pick and Place");
    auto gripper_planner = make_joint_interpolation_planner();
    auto pipeline = make_pipeline_planner();

    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian->setMaxVelocityScalingFactor(0.2);
    cartesian->setMaxAccelerationScalingFactor(0.2);
    cartesian->setStepSize(0.005);
    cartesian->setMinFraction(0.5);

    // Pick sequence
    task.add(make_gripper_stage("open gripper", gripper_planner, true, goal.gripper));
    task.add(make_cartesian_move_stage("pick approach", pick_approach, pipeline, false));
    task.add(make_cartesian_move_stage("grasp", grasp_pose, cartesian, true));
    task.add(make_gripper_stage("close gripper", gripper_planner, false, goal.gripper));
    task.add(make_cartesian_move_stage("pick retreat", pick_retreat, cartesian, true));

    // Place sequence disabled for testing
    RCLCPP_WARN(node()->get_logger(), "Place sequence disabled - pick only");

    return load_plan_execute(task);
}

geometry_msgs::msg::PoseStamped VisionPickPlaceStages::compute_grasp_pose(
    const geometry_msgs::msg::PoseStamped& tag_pose,
    const nlohmann::json& offset)
{
    geometry_msgs::msg::PoseStamped result = tag_pose;
    result.header.frame_id = "base_link";

    tf2::Transform tag_tf;
    tf2::fromMsg(tag_pose.pose, tag_tf);

    tf2::Vector3 offset_vec(offset.value("x", 0.0), offset.value("y", 0.0), offset.value("z", 0.0));
    tf2::Transform offset_tf(tf2::Quaternion::getIdentity(), offset_vec);

    if (offset.contains("rpy")) {
        auto rpy = offset["rpy"].get<std::vector<double>>();
        if (rpy.size() == 3) {
            tf2::Quaternion q;
            q.setRPY(rpy[0], rpy[1], rpy[2]);
            offset_tf.setRotation(q);
        }
    }

    tf2::toMsg(tag_tf * offset_tf, result.pose);
    return result;
}

geometry_msgs::msg::PoseStamped VisionPickPlaceStages::compute_offset_pose(
    const geometry_msgs::msg::PoseStamped& base_pose,
    double z_offset)
{
    auto result = base_pose;
    result.pose.position.z += z_offset;
    return result;
}

std::unique_ptr<mtc::Stage> VisionPickPlaceStages::make_gripper_stage(
    const std::string& label,
    const mtc::solvers::PlannerInterfacePtr& planner,
    bool open,
    const std::string& gripper_type)
{
    auto config = gripper_config::get_gripper_config(gripper_type);
    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->setGroup(config.group);
    stage->setGoal(open ? config.release_state : config.grasp_state);
    return stage;
}

std::unique_ptr<mtc::Stage> VisionPickPlaceStages::make_cartesian_move_stage(
    const std::string& label,
    const geometry_msgs::msg::PoseStamped& target,
    const mtc::solvers::PlannerInterfacePtr& planner,
    bool apply_wrist_constraint)
{
    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(default_arm_group_name());
    stage->setGoal(target);

    if (apply_wrist_constraint) {
        stage->setPathConstraints(createWrist3Constraint());
    }
    return stage;
}
