#include "mtc_pipeline/pick_place_stages.hpp"

#include <moveit/task_constructor/stages/move_to.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace mtc = moveit::task_constructor;

PickPlaceStages::PickPlaceStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config) {}

std::unique_ptr<mtc::Stage> PickPlaceStages::makeMoveToNamedStage(
  const std::string& label,
  const std::string& pose_key,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group_name)
{
  const auto& poses = config().at("poses");
  const auto& joint_pose = poses.at(pose_key);
  if (!joint_pose.is_array() || joint_pose.size() != BaseStages::defaultJointNames().size()) {
    throw std::runtime_error(pose_key + " must be an array of 6 numbers");
  }

  std::vector<double> joint_angles_deg;
  joint_angles_deg.reserve(joint_pose.size());
  for (const auto& value : joint_pose) {
    joint_angles_deg.push_back(value.get<double>());
  }

  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(arm_group_name);
  stage->setGoal(jointsFromDegrees(joint_angles_deg));
  return stage;
}

std::unique_ptr<mtc::Stage> PickPlaceStages::makeGripperStage(
  const std::string& label,
  const std::string& hand_group_name,
  const std::string& goal_state,
  const mtc::solvers::PlannerInterfacePtr& planner)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(hand_group_name);
  stage->setGoal(goal_state);
  return stage;
}

bool PickPlaceStages::run(const nlohmann::json& step,
                          const nlohmann::json& poses,
                          rclcpp::Node::SharedPtr /*node_ptr*/)
{
  if (!step.contains("pick_poses") || !step.contains("place_poses")) {
    RCLCPP_ERROR(node()->get_logger(), "Step must contain pick_poses and place_poses");
    return false;
  }

  refreshPoses(poses);

  const std::vector<std::string> pick_poses = step["pick_poses"].get<std::vector<std::string>>();
  const std::vector<std::string> place_poses = step["place_poses"].get<std::vector<std::string>>();

  if (pick_poses.size() < 2 || place_poses.size() < 2) {
    RCLCPP_ERROR(node()->get_logger(), "pick_poses/place_poses must each contain at least two entries");
    return false;
  }

  const std::string& arm_group_name = defaultArmGroupName();
  constexpr const char* hand_group_name = "hande_gripper";

  auto task = createTaskTemplate("Pick and Place Modular Task", arm_group_name);

  auto sampling_planner = makePipelinePlanner();
  auto interpolation_planner = makeJointInterpolationPlanner();
  auto cartesian_planner = makeCartesianPlanner();

  auto constrain_stage_path = [](std::unique_ptr<mtc::Stage> stage,
                                 const moveit_msgs::msg::Constraints& constraints) {
    if (auto* move_to_stage = dynamic_cast<mtc::stages::MoveTo*>(stage.get())) {
      move_to_stage->setPathConstraints(constraints);
    }
    return stage;
  };

  // Pick sequence
  task.add(makeGripperStage("open gripper", hand_group_name, "hande_open", interpolation_planner));
  task.add(makeMoveToNamedStage("move to pickup approach", pick_poses.at(0), sampling_planner, arm_group_name));
  task.add(makeMoveToNamedStage("move to pickup", pick_poses.at(1), cartesian_planner, arm_group_name));
  task.add(makeGripperStage("close gripper", hand_group_name, "hande_closed", interpolation_planner));
  task.add(makeMoveToNamedStage("pickup retreat", pick_poses.at(0), cartesian_planner, arm_group_name));

  moveit_msgs::msg::Constraints wrist3_constraint;
  moveit_msgs::msg::JointConstraint jc;
  jc.joint_name = "wrist_3_joint";
  jc.position = 0.0;
  jc.tolerance_above = 0.01;
  jc.tolerance_below = 0.01;
  jc.weight = 1.0;
  wrist3_constraint.joint_constraints.push_back(jc);

  task.add(constrain_stage_path(
    makeMoveToNamedStage("move to place", place_poses.at(0), sampling_planner, arm_group_name),
    wrist3_constraint));

  task.add(makeMoveToNamedStage("place", place_poses.at(1), sampling_planner, arm_group_name));
  task.add(makeGripperStage("open gripper", hand_group_name, "hande_open", interpolation_planner));
  task.add(makeMoveToNamedStage("place retreat", place_poses.at(0), cartesian_planner, arm_group_name));

  auto home_stage = std::make_unique<mtc::stages::MoveTo>("move home", sampling_planner);
  home_stage->setGroup(arm_group_name);
  home_stage->setGoal("moveit_home");
  task.add(std::move(home_stage));

  const bool success = loadPlanExecute(task);
  if (!success) {
    RCLCPP_ERROR(node()->get_logger(), "Pick and place task failed");
  }
  return success;
}
