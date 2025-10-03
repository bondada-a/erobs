#include "mtc_pipeline/moveto_stages.hpp"

namespace mtc = moveit::task_constructor;

MoveToStages::MoveToStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
  : BaseStages(node, config) {}


bool MoveToStages::run(const nlohmann::json& step, const nlohmann::json& poses) {
  const std::string target_type = step.at("target_type");
  const std::string planning_type = step.value("planning_type", "joint");

  auto task = createTaskTemplate("MoveTo Task");
  auto planner = (planning_type == "cartesian") ? makeCartesianPlanner() : makePipelinePlanner();

  // 1. NAMED STATE: Move to predefined SRDF state (e.g., "moveit_home")
  if (target_type == "named_state") {
    const std::string named_state = step.at("target");
    task.add(createNamedStateMoveStage("move_to_" + named_state, named_state, planner));
  }

  // 2. POSE: Move to joint configuration from JSON config
  else if (target_type == "pose") {
    const std::string pose_key = step.at("target");
    const auto& joint_pose_json = poses.at(pose_key);

    if (!joint_pose_json.is_array() || joint_pose_json.size() != 6) {
      RCLCPP_ERROR(node()->get_logger(), "'%s' must be an array of 6 joint angles", pose_key.c_str());
      return false;
    }

    const auto joint_angles_deg = joint_pose_json.get<std::vector<double>>();
    const std::string label = planning_type == "cartesian" ? "move_to_cartesian_" + pose_key : "move_to_" + pose_key;

    if (planning_type == "cartesian") {  // Cartesian path
      task.add(createCartesianMoveStageFromJoints(label, joint_angles_deg, planner));
    } else {    // Joint: Plan in joint space (default)
      task.add(createJointMoveStage(label, joint_angles_deg, planner));
    }
  }

  // 3. RELATIVE: Move relative to current position (e.g., "forward", "up")
  else if (target_type == "relative") {
    const std::string direction = step.at("direction");
    const double distance = step.at("distance").get<double>();
    const std::string label = "move_" + direction + "_" + std::to_string(distance) + "m";
    // TODO: Check if relative movements should always use CartesianPlanner instead of respecting planning_type
    task.add(createRelativeMoveStage(label, direction, distance, planner));
  }

  else {
    RCLCPP_ERROR(node()->get_logger(), "Unsupported target_type '%s'", target_type.c_str());
    return false;
  }

  return loadPlanExecute(task);
}
