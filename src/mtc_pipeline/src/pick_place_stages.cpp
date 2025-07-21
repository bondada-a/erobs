#include "mtc_pipeline/pick_place_stages.hpp"
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>
#include <string>

namespace mtc = moveit::task_constructor;

PickPlaceStages::PickPlaceStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : node_(node), config_(config) {}

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

std::vector<std::unique_ptr<mtc::Stage>> PickPlaceStages::makePickStages() {
  std::vector<std::unique_ptr<mtc::Stage>> stages;

  const std::string arm_group_name = "ur_arm";
  const std::string hand_group_name = "hande_gripper";
  const std::string hand_frame = "flange";

  // Planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setMaxVelocityScalingFactor(0.2);
  sampling_planner->setMaxAccelerationScalingFactor(0.2);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(0.2);
  interpolation_planner->setMaxAccelerationScalingFactor(0.2);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setStepSize(0.001);
  cartesian_planner->setMinFraction(0.95);

  // 1. Current State
  stages.emplace_back(std::make_unique<mtc::stages::CurrentState>("current"));

  // 2. Open Gripper
  stages.emplace_back(makeGripperStage("Open Gripper", hand_group_name, "hande_open", interpolation_planner));

  // 3a. Move to pickup_approach from JSON
  stages.emplace_back(makeMoveToNamedStage("move to pickup approach", "pickup_approach", sampling_planner, arm_group_name));
    
  // 3b. Move to actual pickup pose
  stages.emplace_back(makeMoveToNamedStage("move to pickup", "pickup", cartesian_planner, arm_group_name));

  // 4. Allow collision between gripper and object
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
    stage->allowCollisions(
      std::vector<std::string>{ "sample_holder" },
      std::vector<std::string>{ "hand_ee_link", "hand_tool0", "hande_finger_left_link", "hande_finger_right_link" },
      true
    );
    stages.emplace_back(std::move(stage));
  }

  // 5. Close Gripper
  stages.emplace_back(makeGripperStage("close gripper", hand_group_name, "hande_closed", interpolation_planner));

  // 6. Attach Object
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject("sample_holder", hand_frame);
    stages.emplace_back(std::move(stage));
  }

  // 7. Retreat
  stages.emplace_back(makeMoveToNamedStage("pickup retreat", "pickup_approach", cartesian_planner, arm_group_name));

  return stages;
}

std::vector<std::unique_ptr<mtc::Stage>> PickPlaceStages::makePlaceStages() {
  std::vector<std::unique_ptr<mtc::Stage>> stages;

  const std::string arm_group_name = "ur_arm";
  const std::string hand_group_name = "hande_gripper";
  const std::string hand_frame = "flange";

  // Planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setMaxVelocityScalingFactor(0.2);
  sampling_planner->setMaxAccelerationScalingFactor(0.2);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(0.2);
  interpolation_planner->setMaxAccelerationScalingFactor(0.2);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setStepSize(0.001);
  cartesian_planner->setMinFraction(0.95);

  // 1. Move to place approach (with joint constraint)
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
  auto stage = makeMoveToNamedStage("move to place", "place_approach", sampling_planner, arm_group_name);
  auto* move_to_stage = dynamic_cast<mtc::stages::MoveTo*>(stage.get());
  if (move_to_stage) {
      move_to_stage->setPathConstraints(wrist3_constraint);
  }
  stages.emplace_back(std::move(stage));
}


  // 2. Move to place
  stages.emplace_back(makeMoveToNamedStage("place", "place", sampling_planner, arm_group_name));

  // 3. Open Gripper
  stages.emplace_back(makeGripperStage("open gripper", hand_group_name, "hande_open", interpolation_planner));

  // 4. Detach Object
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject("sample_holder", hand_frame);
    stages.emplace_back(std::move(stage));
  }

  // 5. Retreat from place
  stages.emplace_back(makeMoveToNamedStage("place retreat", "place_approach", cartesian_planner, arm_group_name));

  // 6. Move arm to home ("moveit_home" named target)
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move home", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("moveit_home");  // assumes this is a named target in SRDF
    stages.emplace_back(std::move(stage));
  }


  return stages;
}
