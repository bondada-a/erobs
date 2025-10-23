#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include <nlohmann/json.hpp>
#include <std_msgs/msg/color_rgba.hpp>

class PipettorStages : public BaseStages {
public:
    PipettorStages(const rclcpp::Node::SharedPtr& node);

    bool run(const nlohmann::json& step, const nlohmann::json& poses);

private:
    // Helper to format operation name for RViz display
    std::string format_operation_name(const std::string& operation,
                                      double volume_pct,
                                      const std_msgs::msg::ColorRGBA& led_color) const;
};
