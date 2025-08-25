#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/planner_interface.h>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <vector>
#include <map>

namespace mtc = moveit::task_constructor;

class MoveToStages {
public:
  // Constructor
  MoveToStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config);

  // Main orchestrator step runner
  bool run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node);

  // Create a move to named pose (from JSON poses or SRDF named states)
  std::unique_ptr<mtc::Stage> makeMoveToNamedStage(
    const std::string& label,
    const std::string& pose_key,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name,
    bool is_named_state = false
  );

  // Create a move to joint angles (direct joint values)
  std::unique_ptr<mtc::Stage> makeMoveToJointStage(
    const std::string& label,
    const std::vector<double>& joint_angles,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name
  );

  // Create a Cartesian move to pose
  std::unique_ptr<mtc::Stage> makeMoveToPoseStage(
    const std::string& label,
    const geometry_msgs::msg::PoseStamped& pose,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name
  );

  // Create a relative movement stage
  std::unique_ptr<mtc::Stage> makeMoveRelativeStage(
    const std::string& label,
    const std::string& direction,
    double distance,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name
  );

private:
  rclcpp::Node::SharedPtr node_;
  nlohmann::json config_;
  
  // Helper methods
  std::map<std::string, double> convertDegreesToRadians(const std::vector<double>& angles_deg);
  std::vector<std::string> getJointNames();
};
