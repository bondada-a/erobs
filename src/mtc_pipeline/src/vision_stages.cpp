#include "mtc_pipeline/vision_stages.hpp"
#include <moveit/task_constructor/stages/move_to.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>

VisionStages::VisionStages(const rclcpp::Node::SharedPtr& node)
  : BaseStages(node)
{
  // Initialize TF2 for camera→base_link transforms
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  // Zivid service client for 3D marker detection
  capture_marker_client_ = node->create_client<zivid_interfaces::srv::CaptureAndDetectMarkers>(
    "/capture_and_detect_markers");

  // Joint state tracking for cache invalidation
  joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&VisionStages::joint_state_callback, this, std::placeholders::_1));

  // Read parameters
  if (!node->has_parameter("marker_dictionary")) {
    node->declare_parameter("marker_dictionary", marker_dictionary_);
  }
  marker_dictionary_ = node->get_parameter("marker_dictionary").as_string();

  if (!node->has_parameter("publish_marker_frames")) {
    node->declare_parameter("publish_marker_frames", publish_marker_frames_);
  }
  publish_marker_frames_ = node->get_parameter("publish_marker_frames").as_bool();

  // Auto-detect gripper if not explicitly set
  if (!node->has_parameter("ik_frame")) {
    node->declare_parameter("ik_frame", "");  // Empty = auto-detect
  }
  std::string ik_frame_param = node->get_parameter("ik_frame").as_string();

  if (ik_frame_param.empty()) {
    // Auto-detect by checking which frames exist in TF
    RCLCPP_INFO(node->get_logger(), "Auto-detecting gripper TCP frame...");
    RCLCPP_INFO(node->get_logger(), "  Waiting for TF tree to populate (5 seconds)...");

    // Wait longer for TF tree to populate
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Try multiple times with delays
    bool epick_found = false;
    bool hande_found = false;

    for (int attempt = 0; attempt < 10; ++attempt) {
      // Check for EPick first (more specific)
      if (tf_buffer_->canTransform("base", "epick_tip", tf2::TimePointZero, std::chrono::milliseconds(100))) {
        epick_found = true;
        break;
      }
      // Check for Hand-E
      if (tf_buffer_->canTransform("base", "robotiq_hande_end", tf2::TimePointZero, std::chrono::milliseconds(100))) {
        hande_found = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (epick_found) {
      ik_frame_ = "epick_tip";
      z_offset_ = 0.1;  // EPick default offset
      RCLCPP_INFO(node->get_logger(), "  ✓ Detected: Robotiq EPick gripper (epick_tip)");
    }
    else if (hande_found) {
      ik_frame_ = "robotiq_hande_end";
      z_offset_ = -0.02;  // Hand-E default offset
      RCLCPP_INFO(node->get_logger(), "  ✓ Detected: Robotiq Hand-E gripper (robotiq_hande_end)");
    }
    else {
      RCLCPP_ERROR(node->get_logger(), "  ✗ Could not auto-detect gripper!");
      RCLCPP_ERROR(node->get_logger(), "  → Using fallback: epick_tip (adjust in launch file if needed)");
      ik_frame_ = "epick_tip";  // Changed fallback to EPick since that's what you're using
      z_offset_ = 0.1;
    }
  } else {
    ik_frame_ = ik_frame_param;

    // Still allow manual z_offset override
    if (!node->has_parameter("z_offset")) {
      node->declare_parameter("z_offset", z_offset_);
    }
    z_offset_ = node->get_parameter("z_offset").as_double();
  }

  RCLCPP_INFO(node->get_logger(),
    "VisionStages initialized with Zivid ArUco detection (dictionary: %s, ik_frame: %s, z_offset: %.3fm, cache: 30s)",
    marker_dictionary_.c_str(), ik_frame_.c_str(), z_offset_);
}

bool VisionStages::run(const nlohmann::json& step, const nlohmann::json& poses)
{
  const int tag_id = step.at("tag_id").get<int>();
  const double timeout = step.value("timeout", 10.0);

  auto tag_pose = detect_and_transform_tag(tag_id, timeout);
  if (!tag_pose) {
    RCLCPP_ERROR(node()->get_logger(), "Failed to detect tag %d", tag_id);
    return false;
  }

  return move_to_pose(*tag_pose);
}

std::optional<geometry_msgs::msg::PoseStamped> VisionStages::detect_and_transform_tag(
  int tag_id, double timeout)
{
  // Check if we have a valid cached detection
  if (has_valid_cached_detection(tag_id)) {
    const auto& cached = detection_cache_[tag_id];
    double age = (node()->now() - cached.timestamp).seconds();
    RCLCPP_INFO(node()->get_logger(),
      "Using cached detection for tag %d (age: %.1fs, robot stationary)",
      tag_id, age);
    return cached.pose;
  }

  RCLCPP_INFO(node()->get_logger(),
    "No valid cached detection for tag %d, capturing with Zivid...", tag_id);

  // Wait for service
  if (!capture_marker_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node()->get_logger(),
      "Zivid capture_and_detect_markers service not available");
    return std::nullopt;
  }

  // Prepare service request
  auto request = std::make_shared<zivid_interfaces::srv::CaptureAndDetectMarkers::Request>();
  request->marker_ids = {tag_id};
  request->marker_dictionary = marker_dictionary_;

  RCLCPP_INFO(node()->get_logger(),
    "Calling Zivid detection service (dictionary: %s, marker: %d)...",
    marker_dictionary_.c_str(), tag_id);

  // Call service with timeout
  auto future = capture_marker_client_->async_send_request(request);

  // Zivid capture + detection takes 2-5 seconds typically
  const auto service_timeout = std::chrono::duration<double>(timeout);

  // Wait for future without spinning (node is already being spun by action server)
  auto wait_status = future.wait_for(service_timeout);

  if (wait_status != std::future_status::ready) {
    RCLCPP_ERROR(node()->get_logger(),
      "Zivid service call timeout (%.1fs)", timeout);
    return std::nullopt;
  }

  auto result = future.get();

  if (!result->success) {
    RCLCPP_ERROR(node()->get_logger(),
      "Zivid detection failed: %s", result->message.c_str());
    return std::nullopt;
  }

  // Find our marker in the results
  for (const auto& marker : result->detection_result.detected_markers) {
    if (marker.id == tag_id) {
      RCLCPP_INFO(node()->get_logger(),
        "ArUco marker %d detected at [%.3f, %.3f, %.3f] in camera frame",
        marker.id,
        marker.pose.position.x,
        marker.pose.position.y,
        marker.pose.position.z);

      // Transform from camera frame to base_link
      auto pose_base = transform_to_base_link(marker.pose);
      if (!pose_base) {
        RCLCPP_ERROR(node()->get_logger(),
          "Failed to transform marker %d from camera to base_link", tag_id);
        return std::nullopt;
      }

      RCLCPP_INFO(node()->get_logger(),
        "Transformed to base_link: [%.3f, %.3f, %.3f]",
        pose_base->pose.position.x,
        pose_base->pose.position.y,
        pose_base->pose.position.z);

      // Cache the detection with 3D corners
      std::array<geometry_msgs::msg::Point, 4> corners;
      for (size_t i = 0; i < 4 && i < marker.corners_in_camera_coordinates.size(); i++) {
        corners[i] = marker.corners_in_camera_coordinates[i];
      }
      cache_detection(tag_id, *pose_base, corners);

      // Optionally broadcast TF for RViz debugging
      if (publish_marker_frames_) {
        broadcast_marker_tf(tag_id, *pose_base);
      }

      return pose_base;
    }
  }

  RCLCPP_WARN(node()->get_logger(),
    "Marker %d not found in Zivid detection results (%zu markers detected)",
    tag_id, result->detection_result.detected_markers.size());
  return std::nullopt;
}

std::optional<geometry_msgs::msg::PoseStamped> VisionStages::transform_to_base_link(
  const geometry_msgs::msg::Pose& pose_camera)
{
  try {
    // Wait for transform to be available
    std::string camera_frame = "zivid_optical_frame";  // From Zivid camera node
    if (!tf_buffer_->canTransform("base_link", camera_frame,
                                 tf2::TimePointZero, std::chrono::seconds(1))) {
      RCLCPP_ERROR(node()->get_logger(),
        "Transform from %s to base_link not available", camera_frame.c_str());
      return std::nullopt;
    }

    // Get transform
    auto transform = tf_buffer_->lookupTransform("base_link", camera_frame,
                                                 tf2::TimePointZero);

    // Create PoseStamped in camera frame
    geometry_msgs::msg::PoseStamped pose_camera_stamped;
    pose_camera_stamped.header.frame_id = camera_frame;
    pose_camera_stamped.header.stamp = node()->now();
    pose_camera_stamped.pose = pose_camera;

    // Transform to base_link
    geometry_msgs::msg::PoseStamped pose_base;
    tf2::doTransform(pose_camera_stamped, pose_base, transform);

    return pose_base;

  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node()->get_logger(),
      "TF transform failed: %s", ex.what());
    return std::nullopt;
  }
}

void VisionStages::broadcast_marker_tf(int marker_id,
                                        const geometry_msgs::msg::PoseStamped& pose_base)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node()->now();
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "aruco_" + std::to_string(marker_id);

  transform.transform.translation.x = pose_base.pose.position.x;
  transform.transform.translation.y = pose_base.pose.position.y;
  transform.transform.translation.z = pose_base.pose.position.z;
  transform.transform.rotation = pose_base.pose.orientation;

  tf_broadcaster_->sendTransform(transform);

  RCLCPP_DEBUG(node()->get_logger(),
    "Published TF: aruco_%d in base_link", marker_id);
}

void VisionStages::cache_detection(int marker_id,
                                    const geometry_msgs::msg::PoseStamped& pose,
                                    const std::array<geometry_msgs::msg::Point, 4>& corners)
{
  CachedDetection cached;
  cached.pose = pose;
  cached.timestamp = node()->now();
  cached.robot_joints = current_joints_;
  cached.corners_3d = corners;

  detection_cache_[marker_id] = cached;

  RCLCPP_DEBUG(node()->get_logger(),
    "Cached detection for marker %d with 3D corners", marker_id);
}

bool VisionStages::move_to_pose(const geometry_msgs::msg::PoseStamped& target_pose)
{
  RCLCPP_INFO(node()->get_logger(),
    "Target pose in base_link:");
  RCLCPP_INFO(node()->get_logger(),
    "  Position: [%.3f, %.3f, %.3f]",
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    target_pose.pose.position.z);
  RCLCPP_INFO(node()->get_logger(),
    "  Orientation (quat): [%.3f, %.3f, %.3f, %.3f]",
    target_pose.pose.orientation.x,
    target_pose.pose.orientation.y,
    target_pose.pose.orientation.z,
    target_pose.pose.orientation.w);

  // Use detected pose with 180° Z-axis rotation for correct gripper approach
  geometry_msgs::msg::PoseStamped approach_pose = target_pose;

  // Apply 180° rotation around Z axis to flip gripper orientation
  tf2::Quaternion detected_orientation;
  tf2::fromMsg(target_pose.pose.orientation, detected_orientation);

  tf2::Quaternion z_rotation;
  z_rotation.setRPY(0, 0, M_PI);  // 180° around Z axis

  tf2::Quaternion final_orientation = detected_orientation * z_rotation;
  final_orientation.normalize();

  approach_pose.pose.orientation = tf2::toMsg(final_orientation);

  // Add Z-offset to account for TCP position (configurable per gripper)
  approach_pose.pose.position.z += z_offset_;

  RCLCPP_INFO(node()->get_logger(),
    "  Using detected pose with 180° Z-rotation and %.3fm Z-offset", z_offset_);

  // Create MTC task using configured IK frame (TCP)
  auto task = create_task_template("Vision Move", "", ik_frame_);

  // Use Cartesian planner for straight-line motion to detected position
  auto planner = make_cartesian_planner();

  // Create MoveTo stage
  auto move_stage = std::make_unique<mtc::stages::MoveTo>("move to tag", planner);

  // Inherit properties (group, ik_frame) from parent task - standard pattern
  move_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
  move_stage->setGroup(default_arm_group_name());
  move_stage->setGoal(approach_pose);

  task.add(std::move(move_stage));

  // Small delay to ensure robot state is settled before execution
  RCLCPP_INFO(node()->get_logger(), "Waiting for robot to settle before execution...");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Execute
  return load_plan_execute(task);
}

void VisionStages::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.empty()) {
    return;
  }
  current_joints_ = msg->position;
}

bool VisionStages::has_valid_cached_detection(int tag_id)
{
  auto it = detection_cache_.find(tag_id);
  if (it == detection_cache_.end()) {
    return false;
  }

  double age = (node()->now() - it->second.timestamp).seconds();
  if (age > cache_timeout_sec_) {
    RCLCPP_DEBUG(node()->get_logger(),
      "Cached detection for tag %d expired (age: %.1fs)", tag_id, age);
    return false;
  }

  if (robot_has_moved(it->second.robot_joints)) {
    RCLCPP_DEBUG(node()->get_logger(),
      "Cached detection for tag %d invalid (robot moved)", tag_id);
    return false;
  }

  return true;
}

bool VisionStages::robot_has_moved(const std::vector<double>& old_joints)
{
  if (current_joints_.empty() || old_joints.size() != current_joints_.size()) {
    return true;
  }

  for (size_t i = 0; i < old_joints.size(); i++) {
    if (std::abs(old_joints[i] - current_joints_[i]) > joint_movement_threshold_) {
      return true;
    }
  }

  return false;
}
