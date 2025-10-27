#include "mtc_pipeline/base_stages.hpp"

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <algorithm>
#include <array>
#include <map>

// ============================================================================
// Constants and Static Defaults
// ============================================================================

namespace {
  // Direction vectors for relative movements
  // Note: Z-axis is inverted for flange (up = -Z, down = +Z)
  const std::map<std::string, std::array<double, 3>> DIRECTION_VECTORS = {
    {"forward",  { 1.0,  0.0,  0.0}}, {"x",  { 1.0,  0.0,  0.0}},
    {"backward", {-1.0,  0.0,  0.0}}, {"-x", {-1.0,  0.0,  0.0}},
    {"right",    { 0.0,  1.0,  0.0}}, {"y",  { 0.0,  1.0,  0.0}},
    {"left",     { 0.0, -1.0,  0.0}}, {"-y", { 0.0, -1.0,  0.0}},
    {"up",       { 0.0,  0.0, -1.0}}, {"z",  { 0.0,  0.0, -1.0}},  // Physical up = -Z in flange
    {"down",     { 0.0,  0.0,  1.0}}, {"-z", { 0.0,  0.0,  1.0}}   // Physical down = +Z in flange
  };
}

// Static joint and arm defaults
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

// ============================================================================
// Class Implementation
// ============================================================================

BaseStages::BaseStages(const rclcpp::Node::SharedPtr& node)
  : node_(node) {
  // Configure OMPL parameters for pipeline planner
  // Check if parameters are already declared to avoid conflicts
  if (!node_->has_parameter("ompl.planning_plugin")) {
    node_->declare_parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner");
  }
  if (!node_->has_parameter("ompl.request_adapters")) {
    node_->declare_parameter("ompl.request_adapters", "default_planner_request_adapters/AddTimeOptimalParameterization");
  }
}

rclcpp::Node::SharedPtr BaseStages::node() const {
  return node_;
}

// ============================================================================
// Task Template Creation
// ============================================================================

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

// ============================================================================
// Task Execution
// ============================================================================

bool BaseStages::load_plan_execute(mtc::Task& task) const {
  // Initialize
  try {
    if (!task.getRobotModel()) {
      task.loadRobotModel(node_);
    }
    task.init();
  } catch (const mtc::InitStageException&) {
    return false;
  }

  // Plan
  if (!task.plan()) {
    RCLCPP_ERROR(node_->get_logger(), "MTC task planning failed");
    return false;
  }

  // Check if we actually got any solutions
  if (task.solutions().empty()) {
    RCLCPP_ERROR(node_->get_logger(), "MTC planning succeeded but found no solutions");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Found %zu solution(s)", task.solutions().size());

  // Execute
  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "MTC execution failed with error code: %d", result.val);
  }
  return result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

// ============================================================================
// Joint Conversion Utilities
// ============================================================================

std::map<std::string, double> BaseStages::joints_from_degrees(const std::vector<double>& angles_deg) const {
  const auto& names = default_joint_names();
  std::map<std::string, double> joint_goal;

  const size_t count = std::min(angles_deg.size(), names.size());
  for (size_t i = 0; i < count; ++i) {
    joint_goal[names[i]] = deg_to_rad(angles_deg[i]);
  }

  return joint_goal;
}

// ============================================================================
// Planner Configuration & Factories
// ============================================================================

mtc::solvers::PlannerInterfacePtr BaseStages::make_pipeline_planner() const {
  auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  planner->setMaxVelocityScalingFactor(0.2);  // 20% of max velocity
  planner->setMaxAccelerationScalingFactor(0.2);  // 20% of max acceleration
  return planner;
}

mtc::solvers::PlannerInterfacePtr BaseStages::make_cartesian_planner() const {
  auto planner = std::make_shared<mtc::solvers::CartesianPath>();
  planner->setMaxVelocityScalingFactor(0.2);  // 20% of max velocity
  planner->setMaxAccelerationScalingFactor(0.2);  // 20% of max acceleration
  planner->setStepSize(0.001);  // 1mm step size
  planner->setMinFraction(0.6);  // 60% of path must be valid
  return planner;
}

mtc::solvers::PlannerInterfacePtr BaseStages::make_joint_interpolation_planner() const {
  auto planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  planner->setMaxVelocityScalingFactor(0.2);  // 20% of max velocity
  planner->setMaxAccelerationScalingFactor(0.2);  // 20% of max acceleration
  return planner;
}

// ============================================================================
// Movement Stage Creation Methods
// ============================================================================

// Create relative move stage using direction string
std::unique_ptr<mtc::Stage> BaseStages::create_relative_move_stage(
  const std::string& label,
  const std::string& direction,
  double distance,
  const mtc::solvers::PlannerInterfacePtr& planner) const {

  auto it = DIRECTION_VECTORS.find(direction);
  if (it == DIRECTION_VECTORS.end()) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid direction: '%s'", direction.c_str());
    return nullptr;
  }

  const auto& [x, y, z] = it->second;

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
