#include "mtc_pipeline/base_stages.hpp"

#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <array>
#include <cmath>
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

BaseStages::BaseStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : node_(node), config_(config) {}

rclcpp::Node::SharedPtr BaseStages::node() const {
  return node_;
}

nlohmann::json& BaseStages::config() {
  return config_;
}

const nlohmann::json& BaseStages::config() const {
  return config_;
}

// ============================================================================
// Task Template Creation
// ============================================================================

mtc::Task BaseStages::createTaskTemplate(const std::string& name,
                                         const std::string& arm_group,
                                         const std::string& ik_frame,
                                         bool add_current_state) const {
  mtc::Task task;
  task.stages()->setName(name);
  const std::string& group = arm_group.empty() ? defaultArmGroupName() : arm_group;
  task.setProperty("group", group);

  geometry_msgs::msg::PoseStamped ik_frame_pose;
  const std::string& frame = ik_frame.empty() ? defaultIkFrame() : ik_frame;
  ik_frame_pose.header.frame_id = frame;
  ik_frame_pose.pose.orientation.w = 1.0;
  task.setProperty("ik_frame", ik_frame_pose);

  if (add_current_state) {
    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
  }

  return task;
}

// ============================================================================
// Task Execution
// ============================================================================

bool BaseStages::loadPlanExecute(mtc::Task& task,
                                 int plan_attempts,
                                 const ShouldCancelFn& should_cancel) const {
  RCLCPP_DEBUG(node_->get_logger(), "loadPlanExecute: Starting");

  try {
    RCLCPP_DEBUG(node_->get_logger(), "loadPlanExecute: Loading robot model");
    if (!task.getRobotModel()) {
      task.loadRobotModel(node_);
    }
    RCLCPP_DEBUG(node_->get_logger(), "loadPlanExecute: Robot model loaded, initializing task");
    task.init();
    RCLCPP_DEBUG(node_->get_logger(), "loadPlanExecute: Task initialized successfully");
  } catch (const mtc::InitStageException& e) {
    RCLCPP_ERROR(node_->get_logger(), "Stage initialization failed: %s", e.what());
    return false;
  }

  if (should_cancel && should_cancel()) {
    RCLCPP_WARN(node_->get_logger(), "Task cancelled before planning");
    return false;
  }

  RCLCPP_DEBUG(node_->get_logger(), "loadPlanExecute: Starting planning with %d attempts", plan_attempts);
  if (!task.plan(plan_attempts)) {
    RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
    return false;
  }
  RCLCPP_DEBUG(node_->get_logger(), "loadPlanExecute: Planning completed successfully");

  if (task.solutions().empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No solutions found to execute");
    return false;
  }
  RCLCPP_DEBUG(node_->get_logger(), "loadPlanExecute: Found %zu solution(s)", task.solutions().size());

  if (should_cancel && should_cancel()) {
    RCLCPP_WARN(node_->get_logger(), "Task cancelled before execution");
    return false;
  }

  RCLCPP_DEBUG(node_->get_logger(), "loadPlanExecute: Starting execution");
  auto result = task.execute(*task.solutions().front());
  RCLCPP_DEBUG(node_->get_logger(), "loadPlanExecute: Execution completed with code: %d", result.val);

  if (should_cancel && should_cancel()) {
    RCLCPP_WARN(node_->get_logger(), "Task cancelled after execution");
    return false;
  }

  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Task execution failed with code: %d", result.val);
    return false;
  }

  RCLCPP_DEBUG(node_->get_logger(), "loadPlanExecute: Completed successfully");
  return true;
}

// ============================================================================
// Joint Conversion Utilities
// ============================================================================

std::map<std::string, double> BaseStages::jointsFromDegrees(const std::vector<double>& angles_deg) const {
  const auto& names = defaultJointNames();
  std::map<std::string, double> joint_goal;

  for (size_t i = 0; i < std::min(angles_deg.size(), names.size()); ++i) {
    joint_goal[names[i]] = degToRad(angles_deg[i]);
  }

  return joint_goal;
}

std::map<std::string, double> BaseStages::jointsFromRadians(const std::vector<double>& angles_rad) const {
  const auto& names = defaultJointNames();
  std::map<std::string, double> joint_goal;

  for (size_t i = 0; i < std::min(angles_rad.size(), names.size()); ++i) {
    joint_goal[names[i]] = angles_rad[i];
  }

  return joint_goal;
}

// ============================================================================
// Planner Configuration & Factories
// ============================================================================

void BaseStages::configureOmplParameters() const {
  if (!node_) {
    return;
  }

  const auto declare_if_needed = [this](const std::string& name, const rclcpp::ParameterValue& value) {
    try {
      node_->declare_parameter(name, value);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
      // Already declared, safe to ignore
    }
  };

  declare_if_needed("ompl.planning_plugin", rclcpp::ParameterValue(std::string("ompl_interface/OMPLPlanner")));
  declare_if_needed("ompl.request_adapters", rclcpp::ParameterValue(std::string("default_planner_request_adapters/AddTimeOptimalParameterization")));

  try {
    node_->set_parameter(rclcpp::Parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner"));
    node_->set_parameter(rclcpp::Parameter("ompl.request_adapters", "default_planner_request_adapters/AddTimeOptimalParameterization"));
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to set OMPL parameters: %s", e.what());
  }
}

mtc::solvers::PlannerInterfacePtr BaseStages::makePipelinePlanner(const std::string& pipeline_id,
                                                                  double vel_scale,
                                                                  double acc_scale) const {
  if (pipeline_id == "ompl") {
    configureOmplParameters();
  }

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

// Create joint move stage from degrees
std::unique_ptr<mtc::Stage> BaseStages::createJointMoveStage(
  const std::string& label,
  const std::vector<double>& joint_angles_deg,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group) const {

  const std::string& group = arm_group.empty() ? defaultArmGroupName() : arm_group;
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(group);
  stage->setGoal(jointsFromDegrees(joint_angles_deg));
  return stage;
}

// Create joint move stage from pre-converted joint goals
std::unique_ptr<mtc::Stage> BaseStages::createJointMoveStage(
  const std::string& label,
  const std::map<std::string, double>& joint_goals,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group) const {

  const std::string& group = arm_group.empty() ? defaultArmGroupName() : arm_group;
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(group);
  stage->setGoal(joint_goals);
  return stage;
}

// Create relative move stage using direction string
std::unique_ptr<mtc::Stage> BaseStages::createRelativeMoveStage(
  const std::string& label,
  const std::string& direction,
  double distance,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group) const {

  auto it = DIRECTION_VECTORS.find(direction);
  if (it == DIRECTION_VECTORS.end()) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid direction: '%s'", direction.c_str());
    return nullptr;
  }

  const auto& [x, y, z] = it->second;
  const std::string& group = arm_group.empty() ? defaultArmGroupName() : arm_group;

  auto stage = std::make_unique<mtc::stages::MoveRelative>(label, planner);
  stage->setGroup(group);
  stage->setMinMaxDistance(distance, distance);

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = defaultIkFrame();  // Direction expressed in flange frame
  vec.vector.x = x;
  vec.vector.y = y;
  vec.vector.z = z;

  stage->setDirection(vec);
  return stage;
}

// Create named state move stage
std::unique_ptr<mtc::Stage> BaseStages::createNamedStateMoveStage(
  const std::string& label,
  const std::string& named_state,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group) const {

  const std::string& group = arm_group.empty() ? defaultArmGroupName() : arm_group;
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(group);
  stage->setGoal(named_state);
  return stage;
}

// Create cartesian move stage from joint angles (uses FK to convert to pose)
std::unique_ptr<mtc::Stage> BaseStages::createCartesianMoveStageFromJoints(
  const std::string& label,
  const std::vector<double>& joint_angles_deg,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group,
  moveit::core::RobotState& robot_state) const {

  // Convert joints to Cartesian pose using robot state
  const auto& robot_model = robot_state.getRobotModel();
  const auto* group = robot_model->getJointModelGroup(arm_group);

  // Convert degrees to radians
  std::vector<double> joint_angles_rad(joint_angles_deg.size());
  std::transform(joint_angles_deg.begin(), joint_angles_deg.end(),
                 joint_angles_rad.begin(),
                 [this](double deg) { return degToRad(deg); });
  robot_state.setJointGroupPositions(group, joint_angles_rad);

  // Get target pose in Cartesian space using flange frame
  const Eigen::Isometry3d& target_pose_eigen = robot_state.getGlobalLinkTransform(defaultIkFrame());

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = robot_model->getModelFrame();
  target_pose_msg.pose = tf2::toMsg(target_pose_eigen);

  // Create and configure stage
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(arm_group);
  stage->setGoal(target_pose_msg);

  return stage;
}
