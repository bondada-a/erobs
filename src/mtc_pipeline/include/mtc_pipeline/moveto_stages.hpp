#pragma once

#include "mtc_pipeline/base_stages.hpp"

#include <moveit/task_constructor/solvers/planner_interface.h>
#include <nlohmann/json.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

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

  // Main orchestrator step runner with cancellation support
  bool run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node,
           std::function<bool()> should_cancel);

  // Create a move to named pose (from JSON poses or SRDF named states)
  std::unique_ptr<mtc::Stage> moveToNamedPose(
    const std::string& label,
    const std::string& pose_key,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name
  );

  // Create a move to joint angles (direct joint values)
  std::unique_ptr<mtc::Stage> moveToJoints(
    const std::string& label,
    const std::vector<double>& joint_angles,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name
  );

  // Create a Cartesian move to pose
  std::unique_ptr<mtc::Stage> moveToCartesian(
    const std::string& label,
    const geometry_msgs::msg::PoseStamped& pose,
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