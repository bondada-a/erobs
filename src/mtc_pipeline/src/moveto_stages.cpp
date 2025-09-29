#include "mtc_pipeline/moveto_stages.hpp"

#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
}

namespace mtc = moveit::task_constructor;

MoveToStages::MoveToStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config) {}

std::unique_ptr<mtc::Stage> MoveToStages::makeMoveToNamedStage(
  const std::string& label,
  const std::string& pose_key,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group_name,
  bool is_named_state)
{
  if (is_named_state) {
    throw std::runtime_error("Named states should be handled in the run function, not in makeMoveToNamedStage");
  }

  const auto& poses = config().at("poses");
  const auto& joint_pose = poses.at(pose_key);
  if (!joint_pose.is_array() || joint_pose.size() != BaseStages::defaultJointNames().size()) {
    throw std::runtime_error(pose_key + " must be an array of 6 numbers");
  }

  std::vector<double> joint_angles_deg;
  joint_angles_deg.reserve(joint_pose.size());
  for (const auto& angle : joint_pose) {
    joint_angles_deg.push_back(angle.get<double>());
  }

  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(arm_group_name);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
  stage->setIKFrame("flange");
  stage->setGoal(jointsFromDegrees(joint_angles_deg));
  return stage;
}

std::unique_ptr<mtc::Stage> MoveToStages::makeMoveToJointStage(
  const std::string& label,
  const std::vector<double>& joint_angles,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group_name)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(arm_group_name);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
  stage->setIKFrame("flange");
  stage->setGoal(jointsFromDegrees(joint_angles));
  return stage;
}

std::unique_ptr<mtc::Stage> MoveToStages::makeMoveToPoseStage(
  const std::string& label,
  const geometry_msgs::msg::PoseStamped& pose,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group_name)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(arm_group_name);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
  stage->setIKFrame("flange");
  stage->setGoal(pose);
  return stage;
}

std::unique_ptr<mtc::Stage> MoveToStages::makeMoveRelativeStage(
  const std::string& label,
  const std::string& direction,
  double distance,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group_name)
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>(label, planner);
  stage->properties().set("marker_ns", "relative_move");
  stage->properties().set("link", "flange");
  stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
  stage->setGroup(arm_group_name);
  stage->setMinMaxDistance(std::abs(distance), std::abs(distance));

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "flange";

  if (direction == "forward" || direction == "x") {
    vec.vector.x = (distance >= 0.0) ? 1.0 : -1.0;
  } else if (direction == "right" || direction == "y") {
    vec.vector.y = (distance >= 0.0) ? 1.0 : -1.0;
  } else if (direction == "up" || direction == "z") {
    vec.vector.z = (distance >= 0.0) ? 1.0 : -1.0;
  } else if (direction == "backward" || direction == "-x") {
    vec.vector.x = (distance >= 0.0) ? -1.0 : 1.0;
  } else if (direction == "left" || direction == "-y") {
    vec.vector.y = (distance >= 0.0) ? -1.0 : 1.0;
  } else if (direction == "down" || direction == "-z") {
    vec.vector.z = (distance >= 0.0) ? -1.0 : 1.0;
  } else {
    throw std::runtime_error("Invalid direction: " + direction +
                             ". Use: forward/x, right/y, up/z, backward/-x, left/-y, down/-z");
  }

  stage->setDirection(vec);
  return stage;
}

bool MoveToStages::run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node_ptr)
{
  return run(step, poses, node_ptr, nullptr);
}

bool MoveToStages::run(const nlohmann::json& step,
                       const nlohmann::json& poses,
                       rclcpp::Node::SharedPtr /*node_ptr*/,
                       std::function<bool()> should_cancel)
{
  refreshPoses(poses);

  const std::string target_type = step.value("target_type", "pose");
  const std::string planning_type = step.value("planning_type", "joint");
  const std::string arm_group_name = step.value("arm_group", "ur_arm");

  mtc::solvers::PlannerInterfacePtr planner;
  if (planning_type == "cartesian") {
    planner = makeCartesianPlanner(0.2, 0.2, 0.001, 0.8);
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
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node()->get_logger(), "Failed to load robot model: %s", e.what());
      return false;
    }

    const auto& robot_model = task.getRobotModel();
    if (!robot_model) {
      RCLCPP_ERROR(node()->get_logger(), "Robot model unavailable for named state lookup");
      return false;
    }

    const auto* group = robot_model->getJointModelGroup(arm_group_name);
    if (!group) {
      RCLCPP_ERROR(node()->get_logger(), "Group '%s' not found in robot model", arm_group_name.c_str());
      return false;
    }

    moveit::core::RobotState robot_state(robot_model);
    if (!robot_state.setToDefaultValues(group, named_state)) {
      RCLCPP_ERROR(node()->get_logger(), "Named state '%s' not found for group '%s'",
                   named_state.c_str(), arm_group_name.c_str());
      return false;
    }

    std::vector<double> joint_angles_rad;
    robot_state.copyJointGroupPositions(group, joint_angles_rad);

    std::vector<double> joint_angles_deg(joint_angles_rad.size());
    std::transform(joint_angles_rad.begin(), joint_angles_rad.end(), joint_angles_deg.begin(),
                   [](double value) { return value * RAD_TO_DEG; });

    task.add(makeMoveToJointStage("move_to_" + named_state, joint_angles_deg, planner, arm_group_name));
  } else if (target_type == "joints") {
    const auto joint_angles = step.at("target").get<std::vector<double>>();
    task.add(makeMoveToJointStage("move_to_joints", joint_angles, planner, arm_group_name));
  } else if (target_type == "relative") {
    const std::string direction = step.at("direction");
    const double distance = step.at("distance").get<double>();
    task.add(makeMoveRelativeStage("move_relative", direction, distance, planner, arm_group_name));
  } else if (target_type == "pose") {
    const std::string pose_key = step.at("target");
    task.add(makeMoveToNamedStage("move_to_" + pose_key, pose_key, planner, arm_group_name, false));
  } else {
    RCLCPP_ERROR(node()->get_logger(), "Unsupported target_type '%s'", target_type.c_str());
    return false;
  }

  const bool success = loadPlanExecute(task, 5, should_cancel);
  if (success) {
    RCLCPP_INFO(node()->get_logger(), "MoveTo task completed successfully");
  }
  return success;
}
