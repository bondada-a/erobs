#pragma once

#include "mtc_pipeline/base_stages.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nlohmann/json.hpp>

#include <memory>
#include <optional>
#include <string>

class VisionStages : public BaseStages {
public:
  VisionStages(const rclcpp::Node::SharedPtr& node);

  // Main execution: detect tag and move to it
  bool run(const nlohmann::json& step, const nlohmann::json& poses);

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;

  bool trigger_capture();
  std::optional<geometry_msgs::msg::PoseStamped> detect_tag(
    int tag_id,
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr& detections);
  bool move_to_pose(const geometry_msgs::msg::PoseStamped& target_pose);
};