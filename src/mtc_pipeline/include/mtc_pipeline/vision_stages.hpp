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
  // TF2 for coordinate transforms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscription to AprilTag detections
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_subscription_;

  // Service client for Zivid camera capture
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;

  // Store latest detections
  apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr latest_detections_;

  // Trigger Zivid camera capture
  bool trigger_capture();

  // Detect AprilTag and get its pose in base_link frame
  std::optional<geometry_msgs::msg::PoseStamped> detect_tag(int tag_id, double timeout_seconds);

  // Move robot to detected pose
  bool move_to_pose(const geometry_msgs::msg::PoseStamped& target_pose);
};