#pragma once

#include "mtc_pipeline/base_stages.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nlohmann/json.hpp>

#include <memory>
#include <optional>
#include <string>
#include <map>
#include <vector>

class VisionStages : public BaseStages {
public:
  VisionStages(const rclcpp::Node::SharedPtr& node);

  // Main execution: detect tag and move to it
  bool run(const nlohmann::json& step, const nlohmann::json& poses);

private:
  struct CachedDetection {
    geometry_msgs::msg::PoseStamped pose;
    rclcpp::Time timestamp;
    std::vector<double> robot_joints;
  };

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;

  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::map<int, CachedDetection> detection_cache_;
  std::vector<double> current_joints_;
  double cache_timeout_sec_ = 30.0;
  double joint_movement_threshold_ = 0.01;

  void detection_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  bool has_valid_cached_detection(int tag_id);
  bool robot_has_moved(const std::vector<double>& old_joints);

  bool trigger_capture();
  std::optional<geometry_msgs::msg::PoseStamped> detect_tag(
    int tag_id,
    const apriltag_msgs::msg::AprilTagDetectionArray& detections);
  bool move_to_pose(const geometry_msgs::msg::PoseStamped& target_pose);
};