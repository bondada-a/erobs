#pragma once

#include "mtc_pipeline/base_stages.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <zivid_interfaces/srv/capture_and_detect_markers.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <nlohmann/json.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

class VisionStages : public BaseStages {
public:
  VisionStages(const rclcpp::Node::SharedPtr& node);

  // Main execution: detect tag and move to it
  bool run(const nlohmann::json& step, const nlohmann::json& poses);

  // Make detection available to vision-based pick/place
  std::optional<geometry_msgs::msg::PoseStamped> detect_and_transform_tag(int tag_id, double timeout = 10.0);

private:
  // TF handling
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // Optional for debugging

  // Zivid service client
  rclcpp::Client<zivid_interfaces::srv::CaptureAndDetectMarkers>::SharedPtr capture_marker_client_;

  // Parameters
  std::string marker_dictionary_ = "aruco4x4_50";  // Default ArUco dictionary
  bool publish_marker_frames_ = false;  // Publish TF for RViz debugging
  std::string ik_frame_ = "";  // Empty = auto-detect at runtime, or specify TCP frame
  double z_offset_ = 0.0;  // 0.0 = auto-set based on detected gripper

  // Gripper detection result
  struct GripperDetection {
    std::string ik_frame;
    double z_offset;
  };

  // Collision object info from config
  struct ObjectInfo {
    std::string name;
    std::string shape;  // "box" or "cylinder"
    std::vector<double> dimensions;  // box: [x, y, z], cylinder: [height, radius]
    std::vector<double> tag_offset;  // [x, y, z] offset from tag to object center
  };

  // Helper methods
  GripperDetection detect_current_gripper();
  std::optional<geometry_msgs::msg::PoseStamped> transform_to_base_link(const geometry_msgs::msg::Pose& pose_camera);
  void broadcast_marker_tf(int marker_id, const geometry_msgs::msg::PoseStamped& pose_base);

  bool move_to_pose(const geometry_msgs::msg::PoseStamped& target_pose);

  // Collision object management
  void load_vision_objects_config(const std::string& config_path);
  std::optional<ObjectInfo> get_object_info_for_tag(int tag_id) const;
  void add_collision_object_for_tag(int tag_id, const geometry_msgs::msg::PoseStamped& tag_pose);
  void remove_collision_object(const std::string& object_name);
  geometry_msgs::msg::PoseStamped calculate_object_pose(
    const geometry_msgs::msg::PoseStamped& tag_pose,
    const std::vector<double>& tag_offset) const;

  // Planning scene interface
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  // Object database loaded from config
  std::unordered_map<int, ObjectInfo> object_database_;
  std::string vision_objects_config_path_;
};