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

std::unique_ptr<mtc::Stage> PickPlaceStages::make_move_to_named_stage(
  const std::string& label,
  const std::string& pose_key,
  const nlohmann::json& poses,
  const mtc::solvers::PlannerInterfacePtr& planner)
{
  const auto& joint_pose = poses.at(pose_key);
  if (!joint_pose.is_array() || joint_pose.size() != BaseStages::default_joint_names().size()) {
    RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 numbers", pose_key.c_str());
    return nullptr;
  }

  auto joint_angles_deg = joint_pose.get<std::vector<double>>();
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
  stage->setGroup(default_arm_group_name());
  stage->setGoal(joints_from_degrees(joint_angles_deg));
  return stage;
}

std::unique_ptr<mtc::Stage> PickPlaceStages::make_gripper_stage(
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
  // Validation - check for individual pose fields
  if (!step.contains("pick_approach") || !step.contains("pick_target") ||
      !step.contains("place_approach") || !step.contains("place_target")) {
    RCLCPP_ERROR(node()->get_logger(),
                "Step must contain pick_approach, pick_target, place_approach, and place_target");
    return false;
  }

  // Extract individual pose names
  const std::string pick_approach = step["pick_approach"].get<std::string>();
  const std::string pick_target = step["pick_target"].get<std::string>();
  const std::string place_approach = step["place_approach"].get<std::string>();
  const std::string place_target = step["place_target"].get<std::string>();

  auto task = create_task_template("Pick and Place");
  auto pipeline_planner = make_pipeline_planner();
  auto cartesian_planner = make_cartesian_planner();
  auto gripper_planner = make_joint_interpolation_planner();

  // ============================================================================
  // PICK SEQUENCE
  // ============================================================================

  // 1. Open gripper
  task.add(make_gripper_stage("open gripper", gripper_planner, true));

  // 2. Move to pickup approach
  {
    auto stage = make_move_to_named_stage("pickup approach", pick_approach, poses, pipeline_planner);
    if (!stage) return false;
    task.add(std::move(stage));
  }

  // 3. Move to pickup position
  {
    auto stage = make_move_to_named_stage("pickup", pick_target, poses, cartesian_planner);
    if (!stage) return false;
    task.add(std::move(stage));
  }

  // 4. Close gripper
  task.add(make_gripper_stage("close gripper", gripper_planner, false));

  // 5. Pickup retreat (with wrist constraint to keep wrist orientation)
  {
    auto stage = make_move_to_named_stage("pickup retreat", pick_approach, poses, cartesian_planner);
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
    auto stage = make_move_to_named_stage("place approach", place_approach, poses, pipeline_planner);
    if (!stage) return false;
    if (auto* move_to_stage = dynamic_cast<mtc::stages::MoveTo*>(stage.get())) {
      move_to_stage->setPathConstraints(createWrist3Constraint());
    }
    task.add(std::move(stage));
  }

  // 7. Move to place position (with wrist constraint)
  {
    auto stage = make_move_to_named_stage("place", place_target, poses, cartesian_planner);
    if (!stage) return false;
    if (auto* move_to_stage = dynamic_cast<mtc::stages::MoveTo*>(stage.get())) {
      move_to_stage->setPathConstraints(createWrist3Constraint());
    }
    task.add(std::move(stage));
  }

  // 8. Open gripper
  task.add(make_gripper_stage("open gripper", gripper_planner, true));

  // 9. Place retreat
  {
    auto stage = make_move_to_named_stage("place retreat", place_approach, poses, cartesian_planner);
    if (!stage) return false;
    task.add(std::move(stage));
  }

  // ============================================================================
  // RETURN HOME (optional)
  // ============================================================================
  if (step.value("return_home", true)) {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(default_arm_group_name());
    stage->setGoal("moveit_home");
    task.add(std::move(stage));
  }

  return load_plan_execute(task);
}