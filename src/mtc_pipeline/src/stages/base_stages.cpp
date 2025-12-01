#include "mtc_pipeline/base_stages.hpp"

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <algorithm>
#include <array>
#include <map>

namespace {

// Direction vectors in flange frame. Z is inverted (flange Z points into tool).
const std::map<std::string, std::array<double, 3>> DIRECTION_VECTORS = {
    {"forward",  { 1.0,  0.0,  0.0}}, {"x",  { 1.0,  0.0,  0.0}},
    {"backward", {-1.0,  0.0,  0.0}}, {"-x", {-1.0,  0.0,  0.0}},
    {"right",    { 0.0,  1.0,  0.0}}, {"y",  { 0.0,  1.0,  0.0}},
    {"left",     { 0.0, -1.0,  0.0}}, {"-y", { 0.0, -1.0,  0.0}},
    {"up",       { 0.0,  0.0, -1.0}}, {"z",  { 0.0,  0.0, -1.0}},
    {"down",     { 0.0,  0.0,  1.0}}, {"-z", { 0.0,  0.0,  1.0}}
};

}  // namespace

// Static defaults for UR5e
const std::vector<std::string>& BaseStages::default_joint_names() {
    static const std::vector<std::string> names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    return names;
}

const std::string& BaseStages::default_arm_group_name() {
    static const std::string name = "ur_arm";
    return name;
}

const std::string& BaseStages::default_ik_frame() {
    static const std::string frame = "flange";
    return frame;
}

BaseStages::BaseStages(const rclcpp::Node::SharedPtr& node)
    : node_(node) {
    node_->declare_parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner");
    node_->declare_parameter("ompl.request_adapters",
        "default_planner_request_adapters/AddTimeOptimalParameterization");
}

rclcpp::Node::SharedPtr BaseStages::node() const {
    return node_;
}

mtc::Task BaseStages::create_task_template(const std::string& name,
                                            const std::string& arm_group,
                                            const std::string& ik_frame) const {
    mtc::Task task;
    task.stages()->setName(name);
    task.setProperty("group", arm_group.empty() ? default_arm_group_name() : arm_group);

    geometry_msgs::msg::PoseStamped ik_frame_pose;
    ik_frame_pose.header.frame_id = ik_frame.empty() ? default_ik_frame() : ik_frame;
    task.setProperty("ik_frame", ik_frame_pose);

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    return task;
}

bool BaseStages::load_plan_execute(mtc::Task& task) const {
    try {
        if (!task.getRobotModel()) {
            task.loadRobotModel(node_);
        }
        task.init();
    } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR(node_->get_logger(), "Task init failed: %s", e.what());
        return false;
    }

    if (!task.plan()) {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Found %zu solution(s)", task.solutions().size());

    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Execution failed: %d", result.val);
        return false;
    }

    return true;
}

std::map<std::string, double> BaseStages::joints_from_degrees(
    const std::vector<double>& angles_deg) const {
    const auto& names = default_joint_names();
    std::map<std::string, double> joint_goal;

    const size_t count = std::min(angles_deg.size(), names.size());
    for (size_t i = 0; i < count; ++i) {
        joint_goal[names[i]] = deg_to_rad(angles_deg[i]);
    }
    return joint_goal;
}

mtc::solvers::PlannerInterfacePtr BaseStages::make_pipeline_planner() const {
    auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
    planner->setMaxVelocityScalingFactor(0.2);
    planner->setMaxAccelerationScalingFactor(0.2);
    return planner;
}

mtc::solvers::PlannerInterfacePtr BaseStages::make_cartesian_planner() const {
    auto planner = std::make_shared<mtc::solvers::CartesianPath>();
    planner->setMaxVelocityScalingFactor(0.2);
    planner->setMaxAccelerationScalingFactor(0.2);
    planner->setStepSize(0.001);
    planner->setMinFraction(0.6);
    return planner;
}

mtc::solvers::PlannerInterfacePtr BaseStages::make_joint_interpolation_planner() const {
    auto planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    planner->setMaxVelocityScalingFactor(0.2);
    planner->setMaxAccelerationScalingFactor(0.2);
    return planner;
}

std::unique_ptr<mtc::Stage> BaseStages::create_relative_move_stage(
    const std::string& label,
    const std::string& direction,
    double distance,
    const mtc::solvers::PlannerInterfacePtr& planner) const {

    const auto& [x, y, z] = DIRECTION_VECTORS.at(direction);

    auto stage = std::make_unique<mtc::stages::MoveRelative>(label, planner);
    stage->setGroup(default_arm_group_name());
    stage->setMinMaxDistance(distance, distance);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = default_ik_frame();
    vec.vector.x = x;
    vec.vector.y = y;
    vec.vector.z = z;

    stage->setDirection(vec);
    return stage;
}

moveit_msgs::msg::Constraints BaseStages::create_wrist3_level_constraint() const {
    moveit_msgs::msg::Constraints constraint;
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = "wrist_3_joint";
    jc.position = 0.0;
    jc.tolerance_above = 0.01;
    jc.tolerance_below = 0.01;
    jc.weight = 1.0;
    constraint.joint_constraints.push_back(jc);
    return constraint;
}
