#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include "mtc_pipeline/pick_place_stages.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mtc_test_runner");

  std::string file_path = "/home/user/poses.json";
  node->declare_parameter("poses_file", file_path);
  node->get_parameter("poses_file", file_path);

  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open pose file: %s", file_path.c_str());
    return 1;
  }

  nlohmann::json config;
  file >> config;

  if (!config.contains("poses") || !config.contains("sequence")) {
    RCLCPP_ERROR(node->get_logger(), "Pose file must contain 'poses' and 'sequence'");
    return 1;
  }

  for (const auto& step : config["sequence"]) {
    const std::string action = step.value("action", "");
    if (action != "pick_and_place") {
      RCLCPP_INFO(node->get_logger(), "Skipping unsupported action: '%s'", action.c_str());
      continue;
    }

    PickPlaceStages runner(node, config);
    if (!runner.run(step, config["poses"], node)) {
      RCLCPP_ERROR(node->get_logger(), "Step failed.");
      return 1;
    }
  }

  rclcpp::shutdown();
  return 0;
}
