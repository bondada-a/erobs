#pragma once

#include "mtc_pipeline/base_stages.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <zivid_interfaces/srv/capture_and_detect_markers.hpp>
#include <nlohmann/json.hpp>

#include <memory>
#include <optional>
#include <string>
#include <map>
#include <vector>
#include <array>

class VisionStages : public BaseStages {
public:
  VisionStages(const rclcpp::Node::SharedPtr& node);

  // Main execution: detect tag and move to it
  bool run(const nlohmann::json& step, const nlohmann::json& poses);

  // Make detection available to vision-based pick/place
  std::optional<geometry_msgs::msg::PoseStamped> detect_and_transform_tag(int tag_id, double timeout = 10.0);

private:
  struct CachedDetection {
    geometry_msgs::msg::PoseStamped pose;
    rclcpp::Time timestamp;
    std::vector<double> robot_joints;
    std::array<geometry_msgs::msg::Point, 4> corners_3d;  // For future validation
  };

  // TF handling
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // Optional for debugging

  // Zivid service client
  rclcpp::Client<zivid_interfaces::srv::CaptureAndDetectMarkers>::SharedPtr capture_marker_client_;

  // Joint state tracking for cache invalidation
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Detection cache
  std::map<int, CachedDetection> detection_cache_;
  std::vector<double> current_joints_;

  // Parameters
  double cache_timeout_sec_ = 30.0;
  double joint_movement_threshold_ = 0.01;
  std::string marker_dictionary_ = "aruco4x4_50";  // Default ArUco dictionary
  bool publish_marker_frames_ = false;  // Publish TF for RViz debugging
  std::string ik_frame_ = "robotiq_hande_end";  // TCP frame (robotiq_hande_end, epick_tip, etc.)
  double z_offset_ = -0.02;  // Z offset in meters (negative moves down)

  // Helper methods
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool has_valid_cached_detection(int tag_id);
  bool robot_has_moved(const std::vector<double>& old_joints);

  std::optional<geometry_msgs::msg::PoseStamped> transform_to_base_link(const geometry_msgs::msg::Pose& pose_camera);
  void broadcast_marker_tf(int marker_id, const geometry_msgs::msg::PoseStamped& pose_base);
  void cache_detection(int marker_id, const geometry_msgs::msg::PoseStamped& pose,
                       const std::array<geometry_msgs::msg::Point, 4>& corners);

  bool move_to_pose(const geometry_msgs::msg::PoseStamped& target_pose);
};