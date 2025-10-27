#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/vision_stages.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/task_constructor/solvers/planner_interface.h>
#include <nlohmann/json.hpp>

#include <memory>
#include <string>

class VisionPickPlaceStages : public BaseStages {
public:
  VisionPickPlaceStages(const rclcpp::Node::SharedPtr& node);

  // Main execution: detect tags and perform pick and place
  bool run(const nlohmann::json& step, const nlohmann::json& poses);

private:
  std::shared_ptr<VisionStages> vision_;

  // Compute grasp pose from tag pose with offset
  geometry_msgs::msg::PoseStamped compute_grasp_pose(
    const geometry_msgs::msg::PoseStamped& tag_pose,
    const nlohmann::json& offset);

  // Compute approach/retreat pose with vertical offset
  geometry_msgs::msg::PoseStamped compute_offset_pose(
    const geometry_msgs::msg::PoseStamped& base_pose,
    double z_offset);

  // Create gripper open/close stage
  std::unique_ptr<mtc::Stage> make_gripper_stage(
    const std::string& label,
    const mtc::solvers::PlannerInterfacePtr& planner,
    bool open,
    const std::string& gripper_type);

  // Create Cartesian move stage to target pose
  std::unique_ptr<mtc::Stage> make_cartesian_move_stage(
    const std::string& label,
    const geometry_msgs::msg::PoseStamped& target_pose,
    const mtc::solvers::PlannerInterfacePtr& planner,
    bool apply_wrist_constraint = true);
};