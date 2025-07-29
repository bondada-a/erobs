#include "mtc_pipeline/pick_place_stages.hpp"
#include "mtc_pipeline/tool_exchange_stages.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("mtc_orchestrator", options);

  // Load JSON file (should contain both "poses" and "sequence")
  std::string config_file = "./script.json";
  node->get_parameter("poses_file", config_file);

  std::ifstream ifs(config_file);
  if (!ifs) {
    RCLCPP_ERROR(node->get_logger(), "Unable to open config file: %s", config_file.c_str());
    return 1;
  }
  nlohmann::json config;
  ifs >> config;

  if (!config.contains("poses") || !config.contains("sequence")) {
    RCLCPP_ERROR(node->get_logger(), "JSON must contain 'poses' and 'sequence'");
    return 1;
  }

  const nlohmann::json& poses = config["poses"];
  const nlohmann::json& sequence = config["sequence"];

  // Instantiate modules
  PickPlaceStages pick_place_module(node, config);
  ToolExchangeStages tool_exchange_module(node, config);
  
  
  for (const auto& step : sequence) {
      std::string action = step.at("action").get<std::string>();
      bool success = false;
      const int max_attempts = 3;  // or any number you want
      int attempt = 0;

      for (; attempt < max_attempts; ++attempt) {
          if (action == "pick_and_place") {
              success = pick_place_module.run(step, poses, node);
          }
          else if (action == "tool_exchange") {
              success = tool_exchange_module.run(step, poses, node);
          }
          // ...add more actions as needed...

          if (success) break;  // Succeeded
          RCLCPP_WARN(node->get_logger(),
              "Step %s failed on attempt %d/%d, retrying...", action.c_str(), attempt + 1, max_attempts);
      }

      if (!success) {
          RCLCPP_ERROR(node->get_logger(),
              "%s step failed after %d attempts", action.c_str(), max_attempts);
          return 1;
      }
  }


  rclcpp::shutdown();
  return 0;
}
