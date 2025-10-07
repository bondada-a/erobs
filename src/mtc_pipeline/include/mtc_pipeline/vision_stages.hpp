#pragma once

#include "mtc_pipeline/base_stages.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <nlohmann/json.hpp>

#include <memory>
#include <optional>
#include <string>

class VisionStages : public BaseStages {
public:
  // Constructor
  VisionStages(const rclcpp::Node::SharedPtr& node);

  // Main execution for vision-based movement
  bool run(const nlohmann::json& step, const nlohmann::json& poses);

  // Detect AprilTag and get its pose
  std::optional<geometry_msgs::msg::PoseStamped> detect_tag(
    int tag_id,
    double timeout_seconds
  );

  // Calculate approach pose based on tag position
  geometry_msgs::msg::PoseStamped calculate_approach_pose(
    const geometry_msgs::msg::PoseStamped& tag_pose,
    const std::string& approach_direction,
    double approach_distance,
    bool use_preset_height = false,
    double preset_height = 0.15
  );

private:
  // TF2 for coordinate transforms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscription to AprilTag detections
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_subscription_;

  // Store latest detections
  apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr latest_detections_;

  // Robot base frame (usually "base_link")
  std::string robot_base_frame_;

  // Camera frame (from AprilTag detector)
  std::string camera_frame_;

  // Convert tag detection to robot base frame
  std::optional<geometry_msgs::msg::PoseStamped> transform_to_base_frame(
    const geometry_msgs::msg::PoseStamped& pose_in_camera_frame
  );

  // Helper to generate movement stages
  bool create_vision_move_task(
    const geometry_msgs::msg::PoseStamped& target_pose,
    const std::string& planning_type = "joint"
  );
};