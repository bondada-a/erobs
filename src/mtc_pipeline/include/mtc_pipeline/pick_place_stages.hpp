#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/planner_interface.h>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <vector>

namespace mtc = moveit::task_constructor;

class PickPlaceStages {
public:
  PickPlaceStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config);

  // Modular stage creators:
  std::vector<std::unique_ptr<mtc::Stage>> makePickStages();
  std::vector<std::unique_ptr<mtc::Stage>> makePlaceStages();

  // For custom named moves, e.g. "move to pickup_approach" or "move to place_approach"
  std::unique_ptr<mtc::Stage> makeMoveToNamedStage(
    const std::string& label,
    const std::string& pose_key,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name
  );

  // Optionally: helpers for gripper open/close
  std::unique_ptr<mtc::Stage> makeGripperStage(
    const std::string& label,
    const std::string& hand_group_name,
    const std::string& goal_state,  // e.g. "hande_open" or "hande_close"
    const mtc::solvers::PlannerInterfacePtr& planner
  );

private:
  rclcpp::Node::SharedPtr node_;
  nlohmann::json config_;
};
