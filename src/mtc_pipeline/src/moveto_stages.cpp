#include "mtc_pipeline/moveto_stages.hpp"

#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cmath>
#include <stdexcept>
#include <array>
#include <map>

namespace mtc = moveit::task_constructor;

MoveToStages::MoveToStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config) {}

// Create a move to joint goal (handles both named poses and direct joint values)
std::unique_ptr<mtc::Stage> MoveToStages::moveToJointGoal(
  const std::string& label,
  const std::vector<double>& joint_angles,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group_name) const
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(arm_group_name);
  stage->setGoal(jointsFromDegrees(joint_angles));
  return stage;
}


// Create a relative movement stage
std::unique_ptr<mtc::Stage> MoveToStages::moveToRelative(
  const std::string& label,
  const std::string& direction,
  double distance,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group_name) const
{
  // Direction vectors: {x, y, z}
  // Note: Z-axis is inverted for flange (up = -Z, down = +Z)
  static const std::map<std::string, std::array<double, 3>> DIRECTION_VECTORS = {
    {"forward",  { 1.0,  0.0,  0.0}}, {"x",  { 1.0,  0.0,  0.0}},
    {"backward", {-1.0,  0.0,  0.0}}, {"-x", {-1.0,  0.0,  0.0}},
    {"right",    { 0.0,  1.0,  0.0}}, {"y",  { 0.0,  1.0,  0.0}},
    {"left",     { 0.0, -1.0,  0.0}}, {"-y", { 0.0, -1.0,  0.0}},
    {"up",       { 0.0,  0.0, -1.0}}, {"z",  { 0.0,  0.0, -1.0}},  // Inverted Z
    {"down",     { 0.0,  0.0,  1.0}}, {"-z", { 0.0,  0.0,  1.0}}   // Inverted Z
  };

  auto stage = std::make_unique<mtc::stages::MoveRelative>(label, planner);
  stage->setGroup(arm_group_name);
  stage->setMinMaxDistance(distance, distance);

  const auto& [x, y, z] = DIRECTION_VECTORS.at(direction);

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "flange";
  vec.vector.x = x;
  vec.vector.y = y;
  vec.vector.z = z;

  stage->setDirection(vec);
  return stage;
}

// Create a move to Cartesian pose stage
std::unique_ptr<mtc::Stage> MoveToStages::moveToCartesianPose(
  const std::string& label,
  const std::vector<double>& joint_angles_deg,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group_name,
  moveit::core::RobotState& robot_state) const //needed for forward kinematics
{
  // Convert joints to Cartesian pose using robot state
  const auto& robot_model = robot_state.getRobotModel();
  const auto* group = robot_model->getJointModelGroup(arm_group_name);

  // Convert degrees to radians
  std::vector<double> joint_angles_rad;
  joint_angles_rad.reserve(joint_angles_deg.size());
  for (const auto& angle_deg : joint_angles_deg) {
    joint_angles_rad.push_back(degToRad(angle_deg));
  }
  robot_state.setJointGroupPositions(group, joint_angles_rad);

  // Get target pose in Cartesian space using "flange" as ik_frame
  const std::string ik_frame = "flange";
  const Eigen::Isometry3d& target_pose_eigen = robot_state.getGlobalLinkTransform(ik_frame);

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = robot_model->getModelFrame();
  target_pose_msg.pose = tf2::toMsg(target_pose_eigen);

  // Create and configure stage
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(arm_group_name);
  stage->setGoal(target_pose_msg);

  return stage;
}


// Handle named state : predefined states from the SRDF (moveit_home)
bool MoveToStages::handleNamedState(const nlohmann::json& step, mtc::Task& task,
                                   const mtc::solvers::PlannerInterfacePtr& planner,
                                   const std::string& arm_group_name,
                                   const moveit::core::RobotModelConstPtr&,
                                   const moveit::core::JointModelGroup* group,
                                   moveit::core::RobotState& robot_state) const
{
  const std::string named_state = step.at("target");

  if (!robot_state.setToDefaultValues(group, named_state)) {
    RCLCPP_ERROR(node()->get_logger(), "Named state '%s' not found", named_state.c_str());
    return false;
  }

  std::vector<double> joint_angles_rad;
  robot_state.copyJointGroupPositions(group, joint_angles_rad);

  auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_" + named_state, planner);
  stage->setGroup(arm_group_name);
  stage->setGoal(jointsFromRadians(joint_angles_rad));
  task.add(std::move(stage));
  return true;
}

// Handle joint angles : direct joint angles or pose from json
bool MoveToStages::handleJoints(const nlohmann::json& step, mtc::Task& task,
                               const mtc::solvers::PlannerInterfacePtr& planner,
                               const std::string& arm_group_name,
                               const std::string& planning_type,
                               const moveit::core::RobotModelConstPtr& robot_model,
                               const moveit::core::JointModelGroup* group,
                               moveit::core::RobotState& robot_state) const
{
  if (step.contains("target") && step["target"].is_array()) {
    // Direct joint angles
    const auto joint_angles = step.at("target").get<std::vector<double>>();
    task.add(moveToJointGoal("move_to_joints", joint_angles, planner, arm_group_name));
  }
  else {
    // Pose defined in json
    const std::string pose_key = step.at("target");
    const auto& poses_config = config().at("poses");
    const auto& joint_pose_json = poses_config.at(pose_key);
    if (!joint_pose_json.is_array() || joint_pose_json.size() != 6) {
      RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 joint angles", pose_key.c_str());
      return false;
    }
    auto joint_angles_deg = joint_pose_json.get<std::vector<double>>();

    if (planning_type == "cartesian") {
      // Cartesian planning - convert joints to pose
      task.add(moveToCartesianPose("move_to_cartesian_" + pose_key, joint_angles_deg, planner, arm_group_name, robot_state));
    } else {
      // Joint planning
      task.add(moveToJointGoal("move_to_" + pose_key, joint_angles_deg, planner, arm_group_name));
    }
  }
  return true;
}

// Handle relative movement : move relative to the current position
bool MoveToStages::handleRelative(const nlohmann::json& step, mtc::Task& task,
                                 const mtc::solvers::PlannerInterfacePtr& planner,
                                 const std::string& arm_group_name) const
{
  const std::string direction = step.at("direction");
  const double distance = step.at("distance").get<double>();

  const std::string label = "move_" + direction + "_" + std::to_string(distance) + "m";
  auto stage = moveToRelative(label, direction, distance, planner, arm_group_name);
  if (!stage) {
    return false;  // Error already logged in moveToRelative
  }
  task.add(std::move(stage));
  return true;
}

// =================================================================================
// Main Orchestration
// =================================================================================

bool MoveToStages::run(const nlohmann::json& step,
                       const nlohmann::json& poses)
{
  refreshPoses(poses); // Update internal pose config with new pose data

  const std::string target_type = step.value("target_type", "pose");
  const std::string planning_type = step.value("planning_type", "joint");
  const std::string arm_group_name = step.value("arm_group", defaultArmGroupName());

  RCLCPP_DEBUG(node()->get_logger(), "MoveTo: target_type='%s', planning_type='%s', arm_group='%s'",
              target_type.c_str(), planning_type.c_str(), arm_group_name.c_str());

  // Scope block to control destruction order and avoid class_loader warnings
  bool success;
  {
    auto task = createTaskTemplate("MoveTo Task", arm_group_name);

    // Load robot model and get joint group (needed for named_state and cartesian planning)
    task.loadRobotModel(node());
    const auto& robot_model = task.getRobotModel();
    const auto* group = robot_model->getJointModelGroup(arm_group_name);
    moveit::core::RobotState robot_state(robot_model);

    // Create planners after loading robot model to ensure correct model reference
    mtc::solvers::PlannerInterfacePtr planner;
    if (planning_type == "cartesian") {
      planner = makeCartesianPlanner();
    } else {
      planner = makePipelinePlanner();
    }

    if (target_type == "named_state") {
      if (!handleNamedState(step, task, planner, arm_group_name, robot_model, group, robot_state)) {
        return false;
      }
    } else if (target_type == "joints" || target_type == "pose") {
      if (!handleJoints(step, task, planner, arm_group_name, planning_type, robot_model, group, robot_state)) {
        return false;
      }
    } else if (target_type == "relative") {
      if (!handleRelative(step, task, planner, arm_group_name)) {
        return false;
      }
    } else {
      RCLCPP_ERROR(node()->get_logger(), "Unsupported target_type '%s'", target_type.c_str());
      return false;
    }

    success = loadPlanExecute(task, 5);
    if (success) {
      RCLCPP_INFO(node()->get_logger(), "MoveTo task completed successfully");
    }

    // Explicit cleanup in correct order: planner before task
    planner.reset();
  } // task and any remaining references destroyed here

  return success;
}
