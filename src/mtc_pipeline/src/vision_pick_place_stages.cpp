#include "mtc_pipeline/vision_pick_place_stages.hpp"
#include "../../end_effectors/gripper_config.hpp"

#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace {
// Wrist constraint to prevent tilting during pick/place
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
}  // namespace

VisionPickPlaceStages::VisionPickPlaceStages(const rclcpp::Node::SharedPtr& node)
  : BaseStages(node)
{
  vision_ = std::make_shared<VisionStages>(node);
  RCLCPP_INFO(node->get_logger(), "VisionPickPlaceStages initialized");
}

bool VisionPickPlaceStages::run(const mtc_pipeline::action::VisionPickPlaceAction::Goal& goal)
{
  // Extract parameters from goal
  const int pick_tag_id = goal.pick_tag_id;
  const int place_tag_id = goal.place_tag_id;
  const std::string& gripper = goal.gripper;
  const double approach_offset = goal.approach_offset;
  const double retreat_offset = goal.retreat_offset;

  // Parse grasp offset configuration
  nlohmann::json grasp_offset;
  if (!goal.grasp_offset_json.empty()) {
    try {
      grasp_offset = nlohmann::json::parse(goal.grasp_offset_json);
    } catch (const nlohmann::json::exception& e) {
      RCLCPP_ERROR(node()->get_logger(), "Failed to parse grasp_offset_json: %s", e.what());
      return false;
    }
  } else {
    // Default offset: 5cm above tag, rotated 180 degrees around roll
    grasp_offset = nlohmann::json::parse(R"({"x":0,"y":0,"z":0.05,"rpy":[0,3.14159,0]})");
  }

  RCLCPP_INFO(node()->get_logger(),
    "Starting vision pick and place: pick_tag=%d, place_tag=%d, gripper=%s",
    pick_tag_id, place_tag_id, gripper.c_str());

  // STEP 1: Detect pick target
  RCLCPP_INFO(node()->get_logger(), "Detecting pick tag %d...", pick_tag_id);
  auto pick_tag_pose = vision_->detect_and_transform_tag(pick_tag_id, 10.0);
  if (!pick_tag_pose) {
    RCLCPP_ERROR(node()->get_logger(), "Failed to detect pick tag %d", pick_tag_id);
    return false;
  }

  // STEP 2: Compute pick poses
  // For debugging: First use the raw tag pose without offsets
  RCLCPP_INFO(node()->get_logger(),
    "Raw tag pose detected: [%.3f, %.3f, %.3f]",
    pick_tag_pose->pose.position.x,
    pick_tag_pose->pose.position.y,
    pick_tag_pose->pose.position.z);

  // Start with simpler approach - use tag pose directly as grasp (like vision_moveto)
  // Then add small offsets for approach/retreat
  geometry_msgs::msg::PoseStamped grasp_pose = *pick_tag_pose;  // Start with raw tag pose

  // Add a small Z offset to account for gripper length
  grasp_pose.pose.position.z += 0.02;  // 2cm above tag (minimal offset)

  // For approach, go higher above the grasp position
  auto pick_approach = grasp_pose;
  pick_approach.pose.position.z += approach_offset;  // Add approach offset

  // For retreat, go even higher
  auto pick_retreat = grasp_pose;
  pick_retreat.pose.position.z += retreat_offset;  // Add retreat offset

  // Keep orientation pointing down (like vision_moveto does)
  // This orientation works for vision_moveto, so use the same
  grasp_pose.pose.orientation.x = 0.0;
  grasp_pose.pose.orientation.y = 1.0;  // 180 degree rotation around Y (pointing down)
  grasp_pose.pose.orientation.z = 0.0;
  grasp_pose.pose.orientation.w = 0.0;

  pick_approach.pose.orientation = grasp_pose.pose.orientation;
  pick_retreat.pose.orientation = grasp_pose.pose.orientation;

  RCLCPP_INFO(node()->get_logger(),
    "Pick poses computed:");
  RCLCPP_INFO(node()->get_logger(),
    "  Grasp:    [%.3f, %.3f, %.3f]",
    grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z);
  RCLCPP_INFO(node()->get_logger(),
    "  Approach: [%.3f, %.3f, %.3f] (%.3fm above)",
    pick_approach.pose.position.x, pick_approach.pose.position.y, pick_approach.pose.position.z,
    approach_offset);
  RCLCPP_INFO(node()->get_logger(),
    "  Retreat:  [%.3f, %.3f, %.3f] (%.3fm above)",
    pick_retreat.pose.position.x, pick_retreat.pose.position.y, pick_retreat.pose.position.z,
    retreat_offset);

  // STEP 3: Compute place poses (vision or named)
  geometry_msgs::msg::PoseStamped place_pose, place_approach, place_retreat;

  if (place_tag_id >= 0) {
    // Vision-based place
    RCLCPP_INFO(node()->get_logger(), "Detecting place tag %d...", place_tag_id);
    auto place_tag_pose = vision_->detect_and_transform_tag(place_tag_id, 10.0);
    if (!place_tag_pose) {
      RCLCPP_ERROR(node()->get_logger(), "Failed to detect place tag %d", place_tag_id);
      return false;
    }

    // Use same offset for place as for pick (can be customized later)
    place_pose = compute_grasp_pose(*place_tag_pose, grasp_offset);
    place_approach = compute_offset_pose(place_pose, approach_offset);
    place_retreat = compute_offset_pose(place_pose, retreat_offset);

    RCLCPP_INFO(node()->get_logger(),
      "Place poses computed (vision-based):");
    RCLCPP_INFO(node()->get_logger(),
      "  Place:    [%.3f, %.3f, %.3f]",
      place_pose.pose.position.x, place_pose.pose.position.y, place_pose.pose.position.z);
    RCLCPP_INFO(node()->get_logger(),
      "  Approach: [%.3f, %.3f, %.3f]",
      place_approach.pose.position.x, place_approach.pose.position.y, place_approach.pose.position.z);
    RCLCPP_INFO(node()->get_logger(),
      "  Retreat:  [%.3f, %.3f, %.3f]",
      place_retreat.pose.position.x, place_retreat.pose.position.y, place_retreat.pose.position.z);
  } else {
    // Use predefined place poses when place_tag_id is -1
    RCLCPP_INFO(node()->get_logger(), "Using predefined place poses");

    // Parse place poses from JSON if provided
    if (!goal.place_poses_json.empty()) {
      nlohmann::json place_poses;
      try {
        place_poses = nlohmann::json::parse(goal.place_poses_json);
      } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(node()->get_logger(), "Failed to parse place_poses_json: %s", e.what());
        return false;
      }

      // We need to use a hybrid approach - use predefined joint positions
      // but we'll create dummy Cartesian poses for consistency
      // The actual implementation will use joint-space moves for place

      // Get current end-effector pose as a starting point
      auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node(), default_arm_group_name());
      geometry_msgs::msg::PoseStamped current_pose = move_group->getCurrentPose();

      // Use the current pose with modified Z for approach/place/retreat
      place_approach = current_pose;
      place_approach.pose.position.z = 0.3;  // Default safe height

      place_pose = current_pose;
      place_pose.pose.position.z = 0.15;  // Default place height

      place_retreat = current_pose;
      place_retreat.pose.position.z = 0.35;  // Default retreat height

      // If specific positions are provided in JSON, we can override
      if (place_poses.contains("place_position")) {
        auto pos = place_poses["place_position"];
        if (pos.is_array() && pos.size() == 3) {
          place_pose.pose.position.x = pos[0];
          place_pose.pose.position.y = pos[1];
          place_pose.pose.position.z = pos[2];

          place_approach = place_pose;
          place_approach.pose.position.z += approach_offset;

          place_retreat = place_pose;
          place_retreat.pose.position.z += retreat_offset;
        }
      }

      RCLCPP_INFO(node()->get_logger(),
        "Place poses computed (predefined custom):");
      RCLCPP_INFO(node()->get_logger(),
        "  Place:    [%.3f, %.3f, %.3f]",
        place_pose.pose.position.x, place_pose.pose.position.y, place_pose.pose.position.z);
      RCLCPP_INFO(node()->get_logger(),
        "  Approach: [%.3f, %.3f, %.3f]",
        place_approach.pose.position.x, place_approach.pose.position.y, place_approach.pose.position.z);
      RCLCPP_INFO(node()->get_logger(),
        "  Retreat:  [%.3f, %.3f, %.3f]",
        place_retreat.pose.position.x, place_retreat.pose.position.y, place_retreat.pose.position.z);
    } else {
      // Use default place position if no JSON provided
      RCLCPP_INFO(node()->get_logger(), "Using default place position");

      // Create default place poses at a known safe location
      place_pose.header.frame_id = "base_link";
      place_pose.pose.position.x = 0.4;   // 40cm forward
      place_pose.pose.position.y = 0.3;   // 30cm to the left
      place_pose.pose.position.z = 0.15;  // 15cm height

      // Keep gripper pointing down
      place_pose.pose.orientation.x = 0.0;
      place_pose.pose.orientation.y = 1.0;
      place_pose.pose.orientation.z = 0.0;
      place_pose.pose.orientation.w = 0.0;

      place_approach = place_pose;
      place_approach.pose.position.z += approach_offset;

      place_retreat = place_pose;
      place_retreat.pose.position.z += retreat_offset;

      RCLCPP_INFO(node()->get_logger(),
        "Place poses computed (predefined default):");
      RCLCPP_INFO(node()->get_logger(),
        "  Place:    [%.3f, %.3f, %.3f]",
        place_pose.pose.position.x, place_pose.pose.position.y, place_pose.pose.position.z);
      RCLCPP_INFO(node()->get_logger(),
        "  Approach: [%.3f, %.3f, %.3f]",
        place_approach.pose.position.x, place_approach.pose.position.y, place_approach.pose.position.z);
      RCLCPP_INFO(node()->get_logger(),
        "  Retreat:  [%.3f, %.3f, %.3f]",
        place_retreat.pose.position.x, place_retreat.pose.position.y, place_retreat.pose.position.z);
    }
  }

  // STEP 4: Build MTC task with all computed poses
  auto task = create_task_template("Vision Pick and Place");

  // Create planners
  auto gripper_planner = make_joint_interpolation_planner();
  auto pipeline_planner = make_pipeline_planner();  // For long-distance moves

  // Custom Cartesian planner with relaxed constraints for vision-based tasks
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setStepSize(0.005);  // 5mm step size (larger for better success)
  cartesian_planner->setMinFraction(0.5);  // 50% of path must be valid (relaxed)

  RCLCPP_INFO(node()->get_logger(),
    "Building MTC task with PICK-ONLY sequence (5 stages)");

  // ============================================================================
  // PICK SEQUENCE
  // ============================================================================

  // 1. Open gripper
  task.add(make_gripper_stage("open gripper", gripper_planner, true, gripper));

  // 2. Move to pick approach (use pipeline planner for flexibility)
  task.add(make_cartesian_move_stage("pick approach", pick_approach, pipeline_planner, false));

  // 3. Move to grasp pose (short Cartesian move from approach to grasp)
  task.add(make_cartesian_move_stage("grasp", grasp_pose, cartesian_planner, true));

  // 4. Close gripper
  task.add(make_gripper_stage("close gripper", gripper_planner, false, gripper));

  // 5. Pick retreat (short Cartesian move up with object)
  task.add(make_cartesian_move_stage("pick retreat", pick_retreat, cartesian_planner, true));

  // ============================================================================
  // PLACE SEQUENCE - COMMENTED OUT FOR TESTING
  // ============================================================================

  RCLCPP_WARN(node()->get_logger(),
    "PLACE SEQUENCE TEMPORARILY DISABLED FOR TESTING - Only performing pick operation");

  // TODO: Uncomment after pick testing is complete
  /*
  // 6. Move to place approach (use pipeline planner for free-space motion)
  task.add(make_cartesian_move_stage("place approach", place_approach, pipeline_planner, false));

  // 7. Move to place position (short Cartesian move down)
  task.add(make_cartesian_move_stage("place", place_pose, cartesian_planner, true));

  // 8. Open gripper
  task.add(make_gripper_stage("release gripper", gripper_planner, true, gripper));

  // 9. Place retreat (short Cartesian move up)
  task.add(make_cartesian_move_stage("place retreat", place_retreat, cartesian_planner, false));

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
  */

  // Execute the task
  return load_plan_execute(task);
}

geometry_msgs::msg::PoseStamped VisionPickPlaceStages::compute_grasp_pose(
  const geometry_msgs::msg::PoseStamped& tag_pose,
  const nlohmann::json& offset)
{
  // Transform tag pose by offset
  geometry_msgs::msg::PoseStamped grasp_pose = tag_pose;
  grasp_pose.header.frame_id = "base_link";  // Ensure we're in base_link frame

  // Convert tag pose to tf2 transform
  tf2::Transform tag_tf;
  tf2::fromMsg(tag_pose.pose, tag_tf);

  // Extract translation offset
  double x = offset.value("x", 0.0);
  double y = offset.value("y", 0.0);
  double z = offset.value("z", 0.0);

  // Create offset transform
  tf2::Vector3 offset_vec(x, y, z);
  tf2::Transform offset_tf(tf2::Quaternion::getIdentity(), offset_vec);

  // Apply rotation if specified (roll, pitch, yaw)
  if (offset.contains("rpy")) {
    auto rpy = offset["rpy"].get<std::vector<double>>();
    if (rpy.size() == 3) {
      tf2::Quaternion rot_quat;
      rot_quat.setRPY(rpy[0], rpy[1], rpy[2]);
      offset_tf.setRotation(rot_quat);
    }
  }

  // Apply offset in tag's local frame
  tf2::Transform grasp_tf = tag_tf * offset_tf;

  // Convert back to geometry_msgs
  tf2::toMsg(grasp_tf, grasp_pose.pose);

  RCLCPP_DEBUG(node()->get_logger(),
    "Computed grasp pose: offset=[%.3f, %.3f, %.3f], result=[%.3f, %.3f, %.3f]",
    x, y, z,
    grasp_pose.pose.position.x,
    grasp_pose.pose.position.y,
    grasp_pose.pose.position.z);

  return grasp_pose;
}

geometry_msgs::msg::PoseStamped VisionPickPlaceStages::compute_offset_pose(
  const geometry_msgs::msg::PoseStamped& base_pose,
  double z_offset)
{
  auto offset_pose = base_pose;
  offset_pose.pose.position.z += z_offset;  // Simple vertical offset in world frame

  RCLCPP_DEBUG(node()->get_logger(),
    "Computed offset pose: z_offset=%.3f, result=[%.3f, %.3f, %.3f]",
    z_offset,
    offset_pose.pose.position.x,
    offset_pose.pose.position.y,
    offset_pose.pose.position.z);

  return offset_pose;
}

std::unique_ptr<mtc::Stage> VisionPickPlaceStages::make_gripper_stage(
  const std::string& label,
  const mtc::solvers::PlannerInterfacePtr& planner,
  bool open,
  const std::string& gripper_type)
{
  auto config = gripper_config::get_gripper_config(gripper_type);
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->setGroup(config.group);
  stage->setGoal(open ? config.release_state : config.grasp_state);

  RCLCPP_DEBUG(node()->get_logger(),
    "Created gripper stage: %s, gripper=%s, state=%s",
    label.c_str(), config.group.c_str(),
    open ? config.release_state.c_str() : config.grasp_state.c_str());

  return stage;
}

std::unique_ptr<mtc::Stage> VisionPickPlaceStages::make_cartesian_move_stage(
  const std::string& label,
  const geometry_msgs::msg::PoseStamped& target_pose,
  const mtc::solvers::PlannerInterfacePtr& planner,
  bool apply_wrist_constraint)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
  stage->setGroup(default_arm_group_name());
  stage->setGoal(target_pose);

  // Apply wrist constraint to prevent tilting while carrying object
  if (apply_wrist_constraint) {
    if (auto* move_stage = dynamic_cast<mtc::stages::MoveTo*>(stage.get())) {
      move_stage->setPathConstraints(createWrist3Constraint());
      RCLCPP_DEBUG(node()->get_logger(),
        "Applied wrist constraint to stage: %s", label.c_str());
    }
  }

  RCLCPP_DEBUG(node()->get_logger(),
    "Created cartesian move stage: %s, target=[%.3f, %.3f, %.3f]",
    label.c_str(),
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    target_pose.pose.position.z);

  return stage;
}