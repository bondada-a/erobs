// tool_exchange_stages.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

class ToolExchangeStages {
public:
  ToolExchangeStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config);
  bool run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node);
private:
  rclcpp::Node::SharedPtr node_;
  nlohmann::json config_;
};
