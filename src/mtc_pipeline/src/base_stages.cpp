#include "mtc_pipeline/base_stages.hpp"

#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/exceptions/exceptions.hpp>

#include <algorithm>
#include <cmath>

namespace {
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
}

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

void BaseStages::refreshPoses(const nlohmann::json& poses) {
  config_["poses"] = poses;
}

mtc::Task BaseStages::createTaskTemplate(const std::string& name,
                                         const std::string& arm_group,
                                         const std::string& ik_frame,
                                         bool add_current_state) const {
  mtc::Task task;
  task.stages()->setName(name);
  task.setProperty("group", arm_group);

  geometry_msgs::msg::PoseStamped ik_frame_pose;
  ik_frame_pose.header.frame_id = ik_frame;
  ik_frame_pose.pose.orientation.w = 1.0;
  task.setProperty("ik_frame", ik_frame_pose);

  if (add_current_state) {
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));
  }

  return task;
}

bool BaseStages::loadPlanExecute(mtc::Task& task,
                                 int plan_attempts,
                                 const ShouldCancelFn& should_cancel) const {
  try {
    if (!task.getRobotModel()) {
      task.loadRobotModel(node_);
    }
    task.init();
  } catch (const mtc::InitStageException& e) {
    RCLCPP_ERROR(node_->get_logger(), "Stage initialization failed: %s", e.what());
    return false;
  }

  if (should_cancel && should_cancel()) {
    RCLCPP_WARN(node_->get_logger(), "Task cancelled before planning");
    return false;
  }

  if (!task.plan(plan_attempts)) {
    RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
    return false;
  }

  if (task.solutions().empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No solutions found to execute");
    return false;
  }

  if (should_cancel && should_cancel()) {
    RCLCPP_WARN(node_->get_logger(), "Task cancelled before execution");
    return false;
  }

  auto result = task.execute(*task.solutions().front());

  if (should_cancel && should_cancel()) {
    RCLCPP_WARN(node_->get_logger(), "Task cancelled after execution");
    return false;
  }

  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Task execution failed with code: %d", result.val);
    return false;
  }

  return true;
}

std::map<std::string, double> BaseStages::jointsFromDegrees(const std::vector<double>& angles_deg) const {
  const auto& names = defaultJointNames();
  std::map<std::string, double> joint_goal;

  for (size_t i = 0; i < std::min(angles_deg.size(), names.size()); ++i) {
    joint_goal[names[i]] = angles_deg[i] * DEG_TO_RAD;
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

const double BaseStages::PipelinePlannerDefaults::vel_scale = 0.2;
const double BaseStages::PipelinePlannerDefaults::acc_scale = 0.2;
const char* const BaseStages::PipelinePlannerDefaults::pipeline_id = "ompl";

const double BaseStages::CartesianPlannerDefaults::vel_scale = 0.2;
const double BaseStages::CartesianPlannerDefaults::acc_scale = 0.2;
const double BaseStages::CartesianPlannerDefaults::step = 0.001;
const double BaseStages::CartesianPlannerDefaults::min_fraction = 0.6; // 60% of the path should be valid

const double BaseStages::JointInterpolationPlannerDefaults::vel_scale = 0.2;
const double BaseStages::JointInterpolationPlannerDefaults::acc_scale = 0.2;

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

std::shared_ptr<mtc::solvers::PipelinePlanner> BaseStages::makePipelinePlanner(const std::string& pipeline_id,
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

std::shared_ptr<mtc::solvers::CartesianPath> BaseStages::makeCartesianPlanner(double vel_scale,
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

std::shared_ptr<mtc::solvers::JointInterpolationPlanner> BaseStages::makeJointInterpolationPlanner(double vel_scale,
                                                                                                    double acc_scale) const {
  auto planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  planner->setMaxVelocityScalingFactor(vel_scale);
  planner->setMaxAccelerationScalingFactor(acc_scale);
  return planner;
}
