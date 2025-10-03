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
const std::vector<std::string>& BaseStages::defaultJointNames() {
  static const std::vector<std::string> names = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  };
  return names;
}

const std::string& BaseStages::defaultArmGroupName() {
  static const std::string name = "ur_arm";
  return name;
}

const std::string& BaseStages::defaultIkFrame() {
  static const std::string frame = "flange";
  return frame;
}

// Planner default constants
const double BaseStages::PipelinePlannerDefaults::vel_scale = 0.2;
const double BaseStages::PipelinePlannerDefaults::acc_scale = 0.2;
const char* const BaseStages::PipelinePlannerDefaults::pipeline_id = "ompl";

const double BaseStages::CartesianPlannerDefaults::vel_scale = 0.2;
const double BaseStages::CartesianPlannerDefaults::acc_scale = 0.2;
const double BaseStages::CartesianPlannerDefaults::step = 0.001;
const double BaseStages::CartesianPlannerDefaults::min_fraction = 0.6; // 60% of the path should be valid

const double BaseStages::JointInterpolationPlannerDefaults::vel_scale = 0.2;
const double BaseStages::JointInterpolationPlannerDefaults::acc_scale = 0.2;

// ============================================================================
// Class Implementation
// ============================================================================

BaseStages::BaseStages(const rclcpp::Node::SharedPtr& node)
  : node_(node) {
  // Configure OMPL parameters for pipeline planner
  node_->declare_parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner");
  node_->declare_parameter("ompl.request_adapters", "default_planner_request_adapters/AddTimeOptimalParameterization");
}

rclcpp::Node::SharedPtr BaseStages::node() const {
  return node_;
}

// ============================================================================
// Task Template Creation
// ============================================================================

mtc::Task BaseStages::createTaskTemplate(const std::string& name,
                                         const std::string& arm_group,
                                         const std::string& ik_frame) const {
  mtc::Task task;
  task.stages()->setName(name);
  task.setProperty("group", arm_group.empty() ? defaultArmGroupName() : arm_group);

  geometry_msgs::msg::PoseStamped ik_frame_pose;
  ik_frame_pose.header.frame_id = ik_frame.empty() ? defaultIkFrame() : ik_frame;
  task.setProperty("ik_frame", ik_frame_pose);

  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  return task;
}

// ============================================================================
// Task Execution
// ============================================================================

bool BaseStages::loadPlanExecute(mtc::Task& task) const {
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
    return false;
  }

  // Execute
  auto result = task.execute(*task.solutions().front());
  return result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

// ============================================================================
// Joint Conversion Utilities
// ============================================================================

std::map<std::string, double> BaseStages::jointsFromDegrees(const std::vector<double>& angles_deg) const {
  const auto& names = defaultJointNames();
  std::map<std::string, double> joint_goal;

  const size_t count = std::min(angles_deg.size(), names.size());
  for (size_t i = 0; i < count; ++i) {
    joint_goal[names[i]] = degToRad(angles_deg[i]);
  }

  return joint_goal;
}

// ============================================================================
// Planner Configuration & Factories
// ============================================================================

mtc::solvers::PlannerInterfacePtr BaseStages::makePipelinePlanner(const std::string& pipeline_id,
                                                                  double vel_scale,
                                                                  double acc_scale) const {
  auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, pipeline_id);
  planner->setMaxVelocityScalingFactor(vel_scale);
  planner->setMaxAccelerationScalingFactor(acc_scale);
  return planner;
}

mtc::solvers::PlannerInterfacePtr BaseStages::makeCartesianPlanner(double vel_scale,
                                                                    double acc_scale,
                                                                    double step,
                                                                    double min_fraction) const {
  auto planner = std::make_shared<mtc::solvers::CartesianPath>();
  planner->setMaxVelocityScalingFactor(vel_scale);
  planner->setMaxAccelerationScalingFactor(acc_scale);
  planner->setStepSize(step);
  planner->setMinFraction(min_fraction);
  return planner;
}

mtc::solvers::PlannerInterfacePtr BaseStages::makeJointInterpolationPlanner(double vel_scale,
                                                                             double acc_scale) const {
  auto planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  planner->setMaxVelocityScalingFactor(vel_scale);
  planner->setMaxAccelerationScalingFactor(acc_scale);
  return planner;
}

// ============================================================================
// Movement Stage Creation Methods
// ============================================================================

// Create relative move stage using direction string
std::unique_ptr<mtc::Stage> BaseStages::createRelativeMoveStage(
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
  stage->setGroup(defaultArmGroupName());
  stage->setMinMaxDistance(distance, distance);

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = defaultIkFrame();
  vec.vector.x = x;
  vec.vector.y = y;
  vec.vector.z = z;

  stage->setDirection(vec);
  return stage;
}
