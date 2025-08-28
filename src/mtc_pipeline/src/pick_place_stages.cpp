#include "mtc_pipeline/pick_place_stages.hpp"
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <cmath>
#include <stdexcept>

namespace mtc = moveit::task_constructor;

PickPlaceStages::PickPlaceStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : node_(node), config_(config) {}

// Create move stage to named pose
std::unique_ptr<mtc::Stage> PickPlaceStages::makeMoveToNamedStage(
  const std::string& label,
  const std::string& pose_key,
  const mtc::solvers::PlannerInterfacePtr& planner,
  const std::string& arm_group_name
) {
  auto& angles_deg = config_["poses"][pose_key];
  if (!angles_deg.is_array() || angles_deg.size() != 6)
    throw std::runtime_error(pose_key + " must be an array of 6 numbers");

  const std::vector<std::string> joint_names = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint",      "wrist_2_joint",      "wrist_3_joint"
  };
  
  std::map<std::string, double> joint_goal;
  for (size_t i = 0; i < 6; ++i)
    joint_goal[joint_names[i]] = angles_deg[i].get<double>() * M_PI / 180.0;

  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(arm_group_name);
  stage->setGoal(joint_goal);
  return stage;
}

// Create gripper control stage
std::unique_ptr<mtc::Stage> PickPlaceStages::makeGripperStage(
  const std::string& label,
  const std::string& hand_group_name,
  const std::string& goal_state,
  const mtc::solvers::PlannerInterfacePtr& planner
) {
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(hand_group_name);
  stage->setGoal(goal_state);
  return stage;
}

// Execute pick and place task
bool PickPlaceStages::run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node)
{
  if (!step.contains("pick_poses") || !step.contains("place_poses")) {
    RCLCPP_ERROR(node->get_logger(), "Step must contain pick_poses and place_poses");
    return false;
  }
  std::vector<std::string> pick_poses = step["pick_poses"].get<std::vector<std::string>>();
  std::vector<std::string> place_poses = step["place_poses"].get<std::vector<std::string>>();

  // Update config with poses
  config_["poses"] = poses;

  moveit::task_constructor::Task task;
  task.stages()->setName("Pick and Place Modular Task");

  const std::string arm_group_name = "ur_arm";
  const std::string hand_group_name = "hande_gripper";

  // Helper function to configure planners
  auto configurePlanner = [](auto planner, double vel_scale = 0.2, double acc_scale = 0.2) {
    planner->setMaxVelocityScalingFactor(vel_scale);
    planner->setMaxAccelerationScalingFactor(acc_scale);
    return planner;
  };

  // Setup planners
  auto sampling_planner = configurePlanner(std::make_shared<mtc::solvers::PipelinePlanner>(node));
  auto interpolation_planner = configurePlanner(std::make_shared<mtc::solvers::JointInterpolationPlanner>());
  auto cartesian_planner = configurePlanner(std::make_shared<mtc::solvers::CartesianPath>());
  cartesian_planner->setStepSize(0.001);
  cartesian_planner->setMinFraction(0.95);

  // Pick sequence
  task.add(std::make_unique<mtc::stages::CurrentState>("current"));
  task.add(makeGripperStage("Open Gripper", hand_group_name, "hande_open", interpolation_planner));
  task.add(makeMoveToNamedStage("move to pickup approach", pick_poses[0], sampling_planner, arm_group_name));
  task.add(makeMoveToNamedStage("move to pickup", pick_poses[1], cartesian_planner, arm_group_name));
  task.add(makeGripperStage("close gripper", hand_group_name, "hande_closed", interpolation_planner));
  task.add(makeMoveToNamedStage("pickup retreat", pick_poses[0], cartesian_planner, arm_group_name));

  // Place sequence with wrist constraint
  moveit_msgs::msg::Constraints wrist3_constraint;
  {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name      = "wrist_3_joint";
    jc.position        = 0.0;
    jc.tolerance_above = 0.01;
    jc.tolerance_below = 0.01;
    jc.weight          = 1.0;
    wrist3_constraint.joint_constraints.push_back(jc);
  }
  {
    auto stage = makeMoveToNamedStage("move to place", place_poses[0], sampling_planner, arm_group_name);
    auto* move_to_stage = dynamic_cast<mtc::stages::MoveTo*>(stage.get());
    if (move_to_stage) {
      move_to_stage->setPathConstraints(wrist3_constraint);
    }
    task.add(std::move(stage));
  }
  task.add(makeMoveToNamedStage("place", place_poses[1], sampling_planner, arm_group_name));
  task.add(makeGripperStage("open gripper", hand_group_name, "hande_open", interpolation_planner));
  task.add(makeMoveToNamedStage("place retreat", place_poses[0], cartesian_planner, arm_group_name));
  
  // Move to home
  auto home_stage = std::make_unique<mtc::stages::MoveTo>("move home", sampling_planner);
  home_stage->setGroup(arm_group_name);
  home_stage->setGoal("moveit_home");
  task.add(std::move(home_stage));

  // Execute task with error handling
  try {
    task.loadRobotModel(node);
    task.init();
  } catch (const moveit::task_constructor::InitStageException& e) {
    RCLCPP_ERROR(node->get_logger(), "Stage initialization failed: %s", e.what());
    return false;
  }

  if (!task.plan(5)) {
    RCLCPP_ERROR(node->get_logger(), "Task planning failed");
    return false;
  }

  if (task.solutions().empty()) {
    RCLCPP_ERROR(node->get_logger(), "No solutions found to execute");
    return false;
  }

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Task execution failed");
    return false;
  }

  return true;
}
