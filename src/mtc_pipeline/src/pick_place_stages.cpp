#include "mtc_pipeline/pick_place_stages.hpp"

#include <moveit/task_constructor/stages/move_to.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

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

PickPlaceStages::PickPlaceStages(const rclcpp::Node::SharedPtr& node)
  : BaseStages(node) {}

std::unique_ptr<mtc::Stage> PickPlaceStages::makeMoveToNamedStage(
  const std::string& label,
  const std::string& pose_key,
  const nlohmann::json& poses,
  const mtc::solvers::PlannerInterfacePtr& planner)
{
  const auto& joint_pose = poses.at(pose_key);
  if (!joint_pose.is_array() || joint_pose.size() != BaseStages::defaultJointNames().size()) {
    RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 numbers", pose_key.c_str());
    return nullptr;
  }

  auto joint_angles_deg = joint_pose.get<std::vector<double>>();
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
  stage->setGroup(defaultArmGroupName());
  stage->setGoal(jointsFromDegrees(joint_angles_deg));
  return stage;
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
  // Validation
  if (!step.contains("pick_poses") || !step.contains("place_poses")) {
    RCLCPP_ERROR(node()->get_logger(), "Step must contain pick_poses and place_poses");
    return false;
  }

  const std::vector<std::string> pick_poses = step["pick_poses"].get<std::vector<std::string>>();
  const std::vector<std::string> place_poses = step["place_poses"].get<std::vector<std::string>>();

  if (pick_poses.size() < 2 || place_poses.size() < 2) {
    RCLCPP_ERROR(node()->get_logger(), "Need at least 2 poses for pick and place");
    return false;
  }

  auto task = createTaskTemplate("Pick and Place");
  auto pipeline_planner = makePipelinePlanner();
  auto cartesian_planner = makeCartesianPlanner();
  auto gripper_planner = makeJointInterpolationPlanner();

  // ============================================================================
  // PICK SEQUENCE
  // ============================================================================

  // 1. Open gripper
  task.add(makeGripperStage("open gripper", gripper_planner, true));

  // 2. Move to pickup approach
  {
    auto stage = makeMoveToNamedStage("pickup approach", pick_poses[0], poses, pipeline_planner);
    if (!stage) return false;
    task.add(std::move(stage));
  }

  // 3. Move to pickup position
  {
    auto stage = makeMoveToNamedStage("pickup", pick_poses[1], poses, cartesian_planner);
    if (!stage) return false;
    task.add(std::move(stage));
  }

  // 4. Close gripper
  task.add(makeGripperStage("close gripper", gripper_planner, false));

  // 5. Pickup retreat (with wrist constraint to keep wrist orientation)
  {
    auto stage = makeMoveToNamedStage("pickup retreat", pick_poses[0], poses, cartesian_planner);
    if (!stage) return false;
    if (auto* move_to_stage = dynamic_cast<mtc::stages::MoveTo*>(stage.get())) {
      move_to_stage->setPathConstraints(createWrist3Constraint());
    }
    task.add(std::move(stage));
  }

  // ============================================================================
  // PLACE SEQUENCE
  // ============================================================================

  // 6. Move to place approach (with wrist constraint)
  {
    auto stage = makeMoveToNamedStage("place approach", place_poses[0], poses, pipeline_planner);
    if (!stage) return false;
    if (auto* move_to_stage = dynamic_cast<mtc::stages::MoveTo*>(stage.get())) {
      move_to_stage->setPathConstraints(createWrist3Constraint());
    }
    task.add(std::move(stage));
  }

  // 7. Move to place position (with wrist constraint)
  {
    auto stage = makeMoveToNamedStage("place", place_poses[1], poses, cartesian_planner);
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
    auto stage = makeMoveToNamedStage("place retreat", place_poses[0], poses, cartesian_planner);
    if (!stage) return false;
    task.add(std::move(stage));
  }

  // ============================================================================
  // RETURN HOME (optional)
  // ============================================================================
  if (step.value("return_home", true)) {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(defaultArmGroupName());
    stage->setGoal("moveit_home");
    task.add(std::move(stage));
  }

  return loadPlanExecute(task);
}