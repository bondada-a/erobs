#pragma once

#include "mtc_pipeline/base_stages.hpp"

#include <moveit/task_constructor/solvers/planner_interface.h>
#include <nlohmann/json.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace mtc = moveit::task_constructor;

class MoveToStages : public BaseStages {
public:
  // Constructor
  MoveToStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config);

  // Main orchestrator step runner
  bool run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node);

  // Create a move to joint goal (handles both named poses and direct joint values)
  std::unique_ptr<mtc::Stage> moveToJointGoal(
    const std::string& label,
    const std::vector<double>& joint_angles,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name
  );

  // Create a relative movement stage
  std::unique_ptr<mtc::Stage> moveToRelative(
    const std::string& label,
    const std::string& direction,
    double distance,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name
  );

};