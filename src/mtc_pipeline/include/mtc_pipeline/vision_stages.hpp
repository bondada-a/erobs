// Vision-guided motion: detect ArUco markers via Zivid and move to them.

#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/action/vision_move_to_action.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <zivid_interfaces/srv/capture_and_detect_markers.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <optional>
#include <string>
#include <unordered_map>

class VisionStages : public BaseStages {
public:
    VisionStages(const rclcpp::Node::SharedPtr& node);
    bool run(const mtc_pipeline::action::VisionMoveToAction::Goal& goal);

    // Public for vision_pick_place_stages to use
    std::optional<geometry_msgs::msg::PoseStamped> detect_and_transform_tag(int tag_id, double timeout = 10.0);

private:
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Zivid
    rclcpp::Client<zivid_interfaces::srv::CaptureAndDetectMarkers>::SharedPtr capture_marker_client_;

    // Config
    std::string marker_dictionary_ = "aruco4x4_50";
    bool publish_marker_frames_ = false;
    std::string ik_frame_;
    double z_offset_ = 0.0;

    // Gripper detection
    struct GripperDetection { std::string ik_frame; double z_offset; };
    GripperDetection detect_current_gripper();

    // Transforms
    std::optional<geometry_msgs::msg::PoseStamped> transform_to_base_link(const geometry_msgs::msg::Pose& pose);
    void broadcast_marker_tf(int marker_id, const geometry_msgs::msg::PoseStamped& pose);
    bool move_to_pose(const geometry_msgs::msg::PoseStamped& target);

    // Collision objects
    struct ObjectInfo {
        std::string name;
        std::string shape;
        std::vector<double> dimensions;
        std::vector<double> tag_offset;
    };
    std::unordered_map<int, ObjectInfo> object_database_;
    std::string vision_objects_config_path_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

    void load_vision_objects_config(const std::string& path);
    std::optional<ObjectInfo> get_object_info_for_tag(int tag_id) const;
    void add_collision_object_for_tag(int tag_id, const geometry_msgs::msg::PoseStamped& pose);
    void remove_collision_object(const std::string& name);
    geometry_msgs::msg::PoseStamped calculate_object_pose(
        const geometry_msgs::msg::PoseStamped& tag_pose,
        const std::vector<double>& offset) const;
};
