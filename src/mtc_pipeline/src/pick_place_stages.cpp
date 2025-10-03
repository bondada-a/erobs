#include "mtc_pipeline/pick_place_stages.hpp"
#include "mtc_pipeline/moveto_stages.hpp"

#include <moveit/task_constructor/stages/move_to.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace mtc = moveit::task_constructor;

namespace {
constexpr const char* GRIPPER_GROUP = "hande_gripper";
constexpr const char* GRIPPER_OPEN_STATE = "hande_open";
constexpr const char* GRIPPER_CLOSED_STATE = "hande_closed";
constexpr const char* WRIST3_JOINT_NAME = "wrist_3_joint";
constexpr double WRIST3_POSITION = 0.0;
constexpr double WRIST3_TOLERANCE = 0.01;
constexpr double WRIST3_WEIGHT = 1.0;

moveit_msgs::msg::Constraints createWrist3Constraint() {
  moveit_msgs::msg::Constraints constraint;
  moveit_msgs::msg::JointConstraint jc;
  jc.joint_name = WRIST3_JOINT_NAME;
  jc.position = WRIST3_POSITION;
  jc.tolerance_above = WRIST3_TOLERANCE;
  jc.tolerance_below = WRIST3_TOLERANCE;
  jc.weight = WRIST3_WEIGHT;
  constraint.joint_constraints.push_back(jc);
  return constraint;
}
}

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
    RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 numbers", pose_key.c_str());
    return nullptr;
  }

  auto joint_angles_deg = joint_pose.get<std::vector<double>>();

  // TODO: Fix this to use proper MoveToStages API
  // MoveToStages moveto_helper(node(), config());
  // return moveto_helper.moveToJointGoal(label, joint_angles_deg, planner, arm_group_name);

  RCLCPP_ERROR(node()->get_logger(), "makeMoveToNamedStage not implemented - needs refactoring");
  return nullptr;
}

std::unique_ptr<mtc::Stage> PickPlaceStages::makeGripperStage(
  const std::string& label,
  const mtc::solvers::PlannerInterfacePtr& planner,
  bool open)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(GRIPPER_GROUP);
  stage->setGoal(open ? GRIPPER_OPEN_STATE : GRIPPER_CLOSED_STATE);
  return stage;
}

bool PickPlaceStages::run(const nlohmann::json& step,
                          const nlohmann::json& poses)
{
  // Basic validation
  if (!step.contains("pick_poses") || !step.contains("place_poses")) {
    RCLCPP_ERROR(node()->get_logger(), "Step must contain pick_poses and place_poses");
    return false;
  }

  refreshPoses(poses);

  // Debug: Print the entire step to see what we're receiving
  RCLCPP_INFO(node()->get_logger(), "Received step: %s", step.dump().c_str());

  const std::vector<std::string> pick_poses = step["pick_poses"].get<std::vector<std::string>>();
  const std::vector<std::string> place_poses = step["place_poses"].get<std::vector<std::string>>();

  // Debug: Print the parsed poses
  RCLCPP_INFO(node()->get_logger(), "Parsed pick_poses size: %zu", pick_poses.size());
  RCLCPP_INFO(node()->get_logger(), "Parsed place_poses size: %zu", place_poses.size());
  for (size_t i = 0; i < pick_poses.size(); ++i) {
    RCLCPP_INFO(node()->get_logger(), "pick_poses[%zu]: '%s'", i, pick_poses[i].c_str());
  }
  for (size_t i = 0; i < place_poses.size(); ++i) {
    RCLCPP_INFO(node()->get_logger(), "place_poses[%zu]: '%s'", i, place_poses[i].c_str());
  }

  if (pick_poses.size() < 2 || place_poses.size() < 2) {
    RCLCPP_ERROR(node()->get_logger(), "Need at least 2 poses for pick and place");
    return false;
  }

  const std::string arm_group = defaultArmGroupName();
  auto task = createTaskTemplate("Pick and Place", arm_group);

  // Create planners for this task
  auto pipeline_planner = makePipelinePlanner();
  auto cartesian_planner = makeCartesianPlanner();
  auto gripper_planner = makeJointInterpolationPlanner();

  RCLCPP_INFO(node()->get_logger(), "Pick poses: [%s, %s], Place poses: [%s, %s]",
              pick_poses[0].c_str(), pick_poses[1].c_str(),
              place_poses[0].c_str(), place_poses[1].c_str());

  // === PICK SEQUENCE ===
  RCLCPP_INFO(node()->get_logger(), "Building pick sequence...");

  // 1. Open gripper
  task.add(makeGripperStage("open gripper", gripper_planner, true));

  // 2. Move to pickup approach
  {
    auto stage = makeMoveToNamedStage("pickup approach", pick_poses[0], pipeline_planner, arm_group);
    if (!stage) return false;
    task.add(std::move(stage));
  }

  // 3. Move to pickup position
  {
    auto stage = makeMoveToNamedStage("pickup", pick_poses[1], cartesian_planner, arm_group);
    if (!stage) return false;
    task.add(std::move(stage));
  }

  // 4. Close gripper
  task.add(makeGripperStage("close gripper", gripper_planner, false));

  // 5. Pickup retreat (with wrist constraint) - use cartesian for smooth retreat
  {
    auto stage = makeMoveToNamedStage("pickup retreat", pick_poses[0], cartesian_planner, arm_group);
    if (!stage) return false;
    if (auto* move_to_stage = dynamic_cast<mtc::stages::MoveTo*>(stage.get())) {
      move_to_stage->setPathConstraints(createWrist3Constraint());
    }
    task.add(std::move(stage));
  }

  // === PLACE SEQUENCE ===
  RCLCPP_INFO(node()->get_logger(), "Building place sequence...");

  // 6. Move to place approach (with wrist constraint)
  {
    auto stage = makeMoveToNamedStage("place approach", place_poses[0], pipeline_planner, arm_group);
    if (!stage) return false;
    if (auto* move_to_stage = dynamic_cast<mtc::stages::MoveTo*>(stage.get())) {
      move_to_stage->setPathConstraints(createWrist3Constraint());
    }
    task.add(std::move(stage));
  }

  // 7. Move to place position (with wrist constraint)
  {
    auto stage = makeMoveToNamedStage("place", place_poses[1], cartesian_planner, arm_group);
    if (!stage) return false;
    if (auto* move_to_stage = dynamic_cast<mtc::stages::MoveTo*>(stage.get())) {
      move_to_stage->setPathConstraints(createWrist3Constraint());
    }
    task.add(std::move(stage));
  }

  // 8. Open gripper
  task.add(makeGripperStage("open gripper", gripper_planner, true));

  // 9. Place retreat
  {
    auto stage = makeMoveToNamedStage("place retreat", place_poses[0], cartesian_planner, arm_group);
    if (!stage) return false;
    task.add(std::move(stage));
  }

  // 10. Return home (optional)
  if (step.value("return_home", true)) {
    auto home_stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner);
    home_stage->setGroup(arm_group);
    home_stage->setGoal("moveit_home");
    task.add(std::move(home_stage));
  }

  // Execute the complete sequence
  RCLCPP_INFO(node()->get_logger(), "Executing pick and place sequence...");
  const bool success = loadPlanExecute(task);

  if (success) {
    RCLCPP_INFO(node()->get_logger(), "Pick and place sequence completed successfully");
  } else {
    RCLCPP_ERROR(node()->get_logger(), "Pick and place sequence failed");
  }

  return success;
}