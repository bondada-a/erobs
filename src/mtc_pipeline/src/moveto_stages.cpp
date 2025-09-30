#include "mtc_pipeline/moveto_stages.hpp"

#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
}

namespace mtc = moveit::task_constructor;

MoveToStages::MoveToStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config) {}

// =================================================================================
// Stage Factory Functions
// =================================================================================

// Create a move to joint goal (handles both named poses and direct joint values)
std::unique_ptr<mtc::Stage> MoveToStages::moveToJointGoal(
  const std::string& label,
  const std::vector<double>& joint_angles,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group_name)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(arm_group_name);
  stage->setGoal(jointsFromDegrees(joint_angles));
  return stage;
}

// TODO: Feature : Add moveToPosition(x,y,z,r,p,y) for moving directly to a position wrt base frame/world frame

// Create a relative movement stage
std::unique_ptr<mtc::Stage> MoveToStages::moveToRelative(
  const std::string& label,
  const std::string& direction,
  double distance,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group_name)
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>(label, planner);
  stage->setGroup(arm_group_name);
  stage->setMinMaxDistance(std::abs(distance), std::abs(distance));

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "flange";

  if (direction == "forward" || direction == "x") {
    vec.vector.x = (distance >= 0.0) ? 1.0 : -1.0;
  } else if (direction == "right" || direction == "y") {
    vec.vector.y = (distance >= 0.0) ? 1.0 : -1.0;
  } else if (direction == "up" || direction == "z") {
    vec.vector.z = (distance >= 0.0) ? -1.0 : 1.0;
  } else if (direction == "backward" || direction == "-x") {
    vec.vector.x = (distance >= 0.0) ? -1.0 : 1.0;
  } else if (direction == "left" || direction == "-y") {
    vec.vector.y = (distance >= 0.0) ? -1.0 : 1.0;
  } else if (direction == "down" || direction == "-z") {
    vec.vector.z = (distance >= 0.0) ? 1.0 : -1.0;
  } else {
    throw std::runtime_error("Invalid direction: " + direction +
                             ". Use: forward/x, right/y, up/z, backward/-x, left/-y, down/-z");
  }

  stage->setDirection(vec);
  return stage;
}

// =================================================================================
// Main Orchestration
// =================================================================================

bool MoveToStages::run(const nlohmann::json& step,
                       const nlohmann::json& poses,
                       rclcpp::Node::SharedPtr)
{
  refreshPoses(poses); // Update internal pose config with new pose data

  const std::string target_type = step.value("target_type", "pose");
  const std::string planning_type = step.value("planning_type", "joint");
  const std::string arm_group_name = step.value("arm_group", defaultArmGroupName());

  mtc::solvers::PlannerInterfacePtr planner;
  if (planning_type == "cartesian") {
    planner = makeCartesianPlanner(0.1, 0.1, 0.01, 0.8);
  } else {
    planner = makePipelinePlanner("ompl", 0.2, 0.2);
  }

  auto task = createTaskTemplate("MoveTo Task", arm_group_name);

  if (target_type == "named_state") {
    const std::string named_state = step.at("target");

    try {
      if (!task.getRobotModel()) {
        task.loadRobotModel(node());
      }
      const auto& robot_model = task.getRobotModel();
      const auto* group = robot_model->getJointModelGroup(arm_group_name);
      
      moveit::core::RobotState robot_state(robot_model);
      if (!robot_state.setToDefaultValues(group, named_state)) {
        RCLCPP_ERROR(node()->get_logger(), "Named state '%s' failed", named_state.c_str());
        return false;
      }

      std::vector<double> joint_angles_rad;
      robot_state.copyJointGroupPositions(group, joint_angles_rad);

      auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_" + named_state, planner);
      stage->setGroup(arm_group_name);
      stage->setGoal(jointsFromRadians(joint_angles_rad));
      task.add(std::move(stage));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node()->get_logger(), "Named state '%s' failed", named_state.c_str());
      return false;
    }
  } else if (target_type == "joints") { // Direct joint angles : might be useful when using aruco tags to get joint angles directly
    const auto joint_angles = step.at("target").get<std::vector<double>>();
    task.add(moveToJointGoal("move_to_joints", joint_angles, planner, arm_group_name));
  } else if (target_type == "pose") {
    const std::string pose_key = step.at("target");
    const auto& poses_config = config().at("poses");
    const auto& joint_pose_json = poses_config.at(pose_key);
    if (!joint_pose_json.is_array() || joint_pose_json.size() != 6) {
      throw std::runtime_error(pose_key + " must be an array of 6 joint angles");
    }
    auto joint_angles_deg = joint_pose_json.get<std::vector<double>>();

    if (planning_type == "cartesian") {
      // For Cartesian planning to a pose defined by joints, we need to perform Forward Kinematics
      try {
        if (!task.getRobotModel()) {
          task.loadRobotModel(node());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node()->get_logger(), "Failed to load robot model: %s", e.what());
        return false;
      }

      const auto& robot_model = task.getRobotModel();
      moveit::core::RobotState robot_state(robot_model);
      const auto* group = robot_model->getJointModelGroup(arm_group_name);
      
      // Convert degrees to radians for the joint group
      std::vector<double> joint_angles_rad;
      joint_angles_rad.reserve(joint_angles_deg.size());
      for (const auto& angle_deg : joint_angles_deg) {
        joint_angles_rad.push_back(angle_deg * DEG_TO_RAD);
      }
      robot_state.setJointGroupPositions(group, joint_angles_rad);

      // Get the Cartesian pose of the end-effector
      const std::string& ik_frame = task.properties().get<geometry_msgs::msg::PoseStamped>("ik_frame").header.frame_id;
      const Eigen::Isometry3d& target_pose_eigen = robot_state.getGlobalLinkTransform(ik_frame);

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = robot_model->getModelFrame();
      target_pose_msg.pose = tf2::toMsg(target_pose_eigen);

      auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_cartesian_" + pose_key, planner);
      stage->setGroup(arm_group_name);
      stage->setGoal(target_pose_msg);
      task.add(std::move(stage));

    } else {
      // Original behavior for joint-space planning
      task.add(moveToJointGoal("move_to_" + pose_key, joint_angles_deg, planner, arm_group_name));
    }
  } else if (target_type == "relative") {
    const std::string direction = step.at("direction");
    const double distance = step.at("distance").get<double>();
    
    // Generate descriptive label based on direction and distance
    const std::string label = "move_" + direction + "_" + std::to_string(distance) + "m";
    
    task.add(moveToRelative(label, direction, distance, planner, arm_group_name));
  } else {
    RCLCPP_ERROR(node()->get_logger(), "Unsupported target_type '%s'", target_type.c_str());
    return false;
  }

  const bool success = loadPlanExecute(task, 5);
  if (success) {
    RCLCPP_INFO(node()->get_logger(), "MoveTo task completed successfully");
  }
  return success;
}
