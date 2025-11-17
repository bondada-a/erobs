#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/action/pick_place_action.hpp"

#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

class PickPlaceStages : public BaseStages {
public:
  // Constructor
  PickPlaceStages(const rclcpp::Node::SharedPtr& node);

  // Orchestrator step runner
  bool run(const mtc_pipeline::action::PickPlaceAction::Goal& goal, const nlohmann::json& poses);

private:
  // Internal helper: Create move stage to named pose
  std::unique_ptr<mtc::Stage> make_move_to_named_stage(
    const std::string& label,
    const std::string& pose_key,
    const nlohmann::json& poses,
    const mtc::solvers::PlannerInterfacePtr& planner
  );

  // Internal helper: Gripper open/close
  std::unique_ptr<mtc::Stage> make_gripper_stage(
    const std::string& label,
    const mtc::solvers::PlannerInterfacePtr& planner,
    bool open,
    const std::string& gripper_type
  );
};
