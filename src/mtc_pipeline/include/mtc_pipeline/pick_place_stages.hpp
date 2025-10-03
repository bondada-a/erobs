#pragma once

#include "mtc_pipeline/base_stages.hpp"

#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace mtc = moveit::task_constructor;

class PickPlaceStages : public BaseStages {
public:
  // Constructor
  PickPlaceStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config);

  // Create move stage to named pose
  std::unique_ptr<mtc::Stage> makeMoveToNamedStage(
    const std::string& label,
    const std::string& pose_key,
    const nlohmann::json& poses,
    const mtc::solvers::PlannerInterfacePtr& planner
  );

  // Helper for gripper open/close
  std::unique_ptr<mtc::Stage> makeGripperStage(
    const std::string& label,
    const mtc::solvers::PlannerInterfacePtr& planner,
    bool open
  );

  // Orchestrator step runner
  bool run(const nlohmann::json& step, const nlohmann::json& poses);
};
