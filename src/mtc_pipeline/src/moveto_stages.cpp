#include "mtc_pipeline/moveto_stages.hpp"
#include <moveit/robot_state/robot_state.h>

namespace mtc = moveit::task_constructor;

MoveToStages::MoveToStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config) {}


bool MoveToStages::run(const nlohmann::json& step, const nlohmann::json& poses) {
  const std::string target_type = step.at("target_type");
  const std::string planning_type = step.value("planning_type", "joint");

  auto task = createTaskTemplate("MoveTo Task");
  auto planner = (planning_type == "cartesian") ? makeCartesianPlanner() : makePipelinePlanner();

  if (target_type == "named_state") {
    const std::string named_state = step.at("target");
    task.add(createNamedStateMoveStage("move_to_" + named_state, named_state, planner));
  } else if (target_type == "pose") {
    // Pose defined in json config
    const std::string pose_key = step.at("target");
    const auto& joint_pose_json = poses.at(pose_key);

    if (!joint_pose_json.is_array() || joint_pose_json.size() != 6) {
      RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 joint angles", pose_key.c_str());
      return false;
    }

    const auto joint_angles_deg = joint_pose_json.get<std::vector<double>>();
    const std::string label = planning_type == "cartesian" ? "move_to_cartesian_" + pose_key : "move_to_" + pose_key;

    // Apply planning type
    if (planning_type == "cartesian") {
      // Load robot model and create robot state for FK conversion (needed only for cartesian)
      task.loadRobotModel(node());
      const auto& robot_model = task.getRobotModel();
      moveit::core::RobotState robot_state(robot_model);

      // Cartesian planning - convert joints to pose via FK
      const std::string arm_group = defaultArmGroupName();
      task.add(createCartesianMoveStageFromJoints(label, joint_angles_deg, planner, arm_group, robot_state));
    } else {
      // Joint planning
      task.add(createJointMoveStage(label, joint_angles_deg, planner));
    }
  } else if (target_type == "relative") {
    const std::string direction = step.at("direction");
    const double distance = step.at("distance").get<double>();

    const std::string label = "move_" + direction + "_" + std::to_string(distance) + "m";
    task.add(createRelativeMoveStage(label, direction, distance, planner));
  } else {
    RCLCPP_ERROR(node()->get_logger(), "Unsupported target_type '%s'", target_type.c_str());
    return false;
  }

  const bool success = loadPlanExecute(task);
  if (success) {
    RCLCPP_INFO(node()->get_logger(), "MoveTo task completed successfully");
  }
  return success;
}
