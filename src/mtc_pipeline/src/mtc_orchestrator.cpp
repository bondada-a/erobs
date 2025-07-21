#include "mtc_pipeline/pick_place_stages.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Use NodeOptions and enable parameter overrides
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  // Pass NodeOptions to node creation
  auto node = rclcpp::Node::make_shared("mtc_orchestrator", options);

  // Load poses config JSON file (use param or hardcode)
  std::string poses_file = "./recorded_poses.json";
  node->get_parameter("poses_file", poses_file);

  std::ifstream ifs(poses_file);
  if (!ifs) {
    RCLCPP_ERROR(node->get_logger(), "Unable to open poses_file: %s", poses_file.c_str());
    return 1;
  }
  nlohmann::json config;
  ifs >> config;

  // Create modular stage helper
  PickPlaceStages modular_stages(node, config);

  // Create and setup the Task
  moveit::task_constructor::Task task;
  task.stages()->setName("Pick and Place Modular Task");
  task.loadRobotModel(node);

  // Compose the pipeline by adding modular stages
  // --- Pick Stages ---
  for (auto& stage : modular_stages.makePickStages()) {
    task.add(std::move(stage));
  }

  // --- Here, you could add your own custom commands or logic ---

  // --- Place Stages ---
  for (auto& stage : modular_stages.makePlaceStages()) {
    task.add(std::move(stage));
  }

  // Planning
  try {
    task.init();
  } catch (const moveit::task_constructor::InitStageException& e) {
    RCLCPP_ERROR(node->get_logger(), "Stage initialization failed: %s", e.what());
    return 1;
  }

  if (!task.plan(5)) {
    RCLCPP_ERROR(node->get_logger(), "Task planning failed");
    return 1;
  }

  // Execution
  if (task.solutions().empty()) {
    RCLCPP_ERROR(node->get_logger(), "No solutions found to execute");
    return 1;
  }

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Task execution failed");
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
