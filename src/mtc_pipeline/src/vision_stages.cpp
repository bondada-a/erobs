#include "mtc_pipeline/vision_stages.hpp"
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

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

  // Read parameters
  if (!node->has_parameter("marker_dictionary")) {
    node->declare_parameter("marker_dictionary", marker_dictionary_);
  }
  marker_dictionary_ = node->get_parameter("marker_dictionary").as_string();

  if (!node->has_parameter("publish_marker_frames")) {
    node->declare_parameter("publish_marker_frames", publish_marker_frames_);
  }
  publish_marker_frames_ = node->get_parameter("publish_marker_frames").as_bool();

  // Read ik_frame parameter (empty string = auto-detect at runtime)
  if (!node->has_parameter("ik_frame")) {
    node->declare_parameter("ik_frame", "");
  }
  ik_frame_ = node->get_parameter("ik_frame").as_string();

  // Read z_offset parameter (0.0 = auto-set based on detected gripper)
  if (!node->has_parameter("z_offset")) {
    node->declare_parameter("z_offset", 0.0);
  }
  z_offset_ = node->get_parameter("z_offset").as_double();

  // Initialize PlanningSceneInterface for collision object management
  planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  // Read vision objects config path
  if (!node->has_parameter("vision_objects_config")) {
    // Default to package config directory
    std::string package_share = ament_index_cpp::get_package_share_directory("mtc_pipeline");
    vision_objects_config_path_ = package_share + "/config/vision_objects.json";
    node->declare_parameter("vision_objects_config", vision_objects_config_path_);
  } else {
    vision_objects_config_path_ = node->get_parameter("vision_objects_config").as_string();
  }

  // Load vision objects configuration
  load_vision_objects_config(vision_objects_config_path_);

  // Log initialization mode
  if (ik_frame_.empty()) {
    RCLCPP_INFO(node->get_logger(),
      "VisionStages initialized (ik_frame will be auto-detected at runtime)");
  } else {
    RCLCPP_INFO(node->get_logger(),
      "VisionStages initialized with manual config (ik_frame: %s, z_offset: %.3fm)",
      ik_frame_.c_str(), z_offset_);
  }
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
  RCLCPP_INFO(node()->get_logger(),
    "Capturing fresh detection for tag %d with Zivid...", tag_id);

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
        "ArUco marker %d detected at [%.3f, %.3f, %.3f] in camera frame (zivid_optical_frame)",
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

      // Optionally broadcast TF for RViz debugging
      if (publish_marker_frames_) {
        broadcast_marker_tf(tag_id, *pose_base);
      }

      // Add collision object to planning scene
      add_collision_object_for_tag(tag_id, *pose_base);

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

    // Explicitly set frame_id after transform (tf2::doTransform doesn't always set it)
    pose_base.header.frame_id = "base_link";
    pose_base.header.stamp = node()->now();

    RCLCPP_DEBUG(node()->get_logger(),
      "After transform: frame_id='%s', position=[%.3f, %.3f, %.3f]",
      pose_base.header.frame_id.c_str(),
      pose_base.pose.position.x, pose_base.pose.position.y, pose_base.pose.position.z);

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

bool VisionStages::move_to_pose(const geometry_msgs::msg::PoseStamped& target_pose)
{
  // Determine which ik_frame and z_offset to use
  std::string active_ik_frame;
  double active_z_offset;

  if (ik_frame_.empty()) {
    // Runtime auto-detection (when MoveIt is running)
    auto detection = detect_current_gripper();
    active_ik_frame = detection.ik_frame;
    active_z_offset = detection.z_offset;
    RCLCPP_INFO(node()->get_logger(),
      "Auto-detected gripper: %s (z_offset: %.3fm)",
      active_ik_frame.c_str(), active_z_offset);
  } else {
    // Use manually configured values from launch parameters
    active_ik_frame = ik_frame_;

    // If z_offset is default (0.0), infer from ik_frame
    if (std::abs(z_offset_) < 1e-6) {
      active_z_offset = (active_ik_frame.find("epick") != std::string::npos) ? 0.1 : -0.02;
      RCLCPP_INFO(node()->get_logger(),
        "Using configured ik_frame: %s with inferred z_offset: %.3fm",
        active_ik_frame.c_str(), active_z_offset);
    } else {
      active_z_offset = z_offset_;
      RCLCPP_INFO(node()->get_logger(),
        "Using configured gripper: %s (z_offset: %.3fm)",
        active_ik_frame.c_str(), active_z_offset);
    }
  }

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
  approach_pose.pose.position.z += active_z_offset;

  RCLCPP_INFO(node()->get_logger(),
    "  Using detected pose with 180° Z-rotation and %.3fm Z-offset", active_z_offset);

  // Create MTC task using configured IK frame (TCP)
  auto task = create_task_template("Vision Move", "", active_ik_frame);

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

VisionStages::GripperDetection VisionStages::detect_current_gripper() {
  GripperDetection detection;

  // Check for EPick first (more specific frame)
  if (tf_buffer_->canTransform("base", "epick_tip", tf2::TimePointZero,
                                std::chrono::seconds(1))) {
    detection.ik_frame = "epick_tip";
    detection.z_offset = 0.027;  // 10cm above marker
    RCLCPP_DEBUG(node()->get_logger(), "Auto-detected EPick gripper (epick_tip)");
    return detection;
  }

  // Check for Hand-E
  if (tf_buffer_->canTransform("base", "robotiq_hande_end", tf2::TimePointZero,
                                std::chrono::seconds(1))) {
    detection.ik_frame = "robotiq_hande_end";
    detection.z_offset = -0.02;  // 2cm below marker (TCP below fingers)
    RCLCPP_DEBUG(node()->get_logger(), "Auto-detected Hand-E gripper (robotiq_hande_end)");
    return detection;
  }

  // Fallback to flange (standalone mode - no gripper attached)
  RCLCPP_INFO(node()->get_logger(),
    "No gripper-specific TCP frame detected. Using 'flange' (standalone mode).");
  detection.ik_frame = "flange";
  detection.z_offset = 0.0;  // No offset needed for flange
  return detection;
}

// ============================================================================
// COLLISION OBJECT MANAGEMENT
// ============================================================================

void VisionStages::load_vision_objects_config(const std::string& config_path) {
  RCLCPP_INFO(node()->get_logger(),
    "Loading vision objects config from: %s", config_path.c_str());

  std::ifstream config_file(config_path);
  if (!config_file.is_open()) {
    RCLCPP_WARN(node()->get_logger(),
      "Could not open vision objects config file: %s. Collision objects disabled.",
      config_path.c_str());
    return;
  }

  try {
    nlohmann::json config;
    config_file >> config;

    if (!config.contains("vision_objects")) {
      RCLCPP_WARN(node()->get_logger(),
        "Config file missing 'vision_objects' field. No objects loaded.");
      return;
    }

    // Parse each object entry
    for (auto& [tag_id_str, obj_json] : config["vision_objects"].items()) {
      int tag_id = std::stoi(tag_id_str);

      ObjectInfo info;
      info.name = obj_json.at("name").get<std::string>();
      info.shape = obj_json.at("shape").get<std::string>();
      info.dimensions = obj_json.at("dimensions").get<std::vector<double>>();
      info.tag_offset = obj_json.at("tag_offset").get<std::vector<double>>();

      object_database_[tag_id] = info;

      RCLCPP_INFO(node()->get_logger(),
        "Loaded object config: tag_id=%d -> '%s' (%s)",
        tag_id, info.name.c_str(), info.shape.c_str());
    }

    RCLCPP_INFO(node()->get_logger(),
      "Successfully loaded %zu vision object definitions", object_database_.size());

  } catch (const std::exception& e) {
    RCLCPP_ERROR(node()->get_logger(),
      "Failed to parse vision objects config: %s", e.what());
  }
}

std::optional<VisionStages::ObjectInfo> VisionStages::get_object_info_for_tag(int tag_id) const {
  auto it = object_database_.find(tag_id);
  if (it != object_database_.end()) {
    return it->second;
  }
  return std::nullopt;
}

geometry_msgs::msg::PoseStamped VisionStages::calculate_object_pose(
  const geometry_msgs::msg::PoseStamped& tag_pose,
  const std::vector<double>& tag_offset) const {

  // Extract only the rotation from tag pose (not translation!)
  tf2::Quaternion tag_rotation;
  tf2::fromMsg(tag_pose.pose.orientation, tag_rotation);

  // Create offset vector in tag's local frame
  tf2::Vector3 offset_local(tag_offset[0], tag_offset[1], tag_offset[2]);

  // Rotate offset to world frame (using only rotation, not full transform)
  tf2::Vector3 offset_world = tf2::quatRotate(tag_rotation, offset_local);

  // Create object pose by adding rotated offset to tag position
  geometry_msgs::msg::PoseStamped object_pose = tag_pose;
  object_pose.pose.position.x += offset_world.x();
  object_pose.pose.position.y += offset_world.y();
  object_pose.pose.position.z += offset_world.z();
  // Orientation stays the same as tag (no rotation offset)

  return object_pose;
}

void VisionStages::add_collision_object_for_tag(
  int tag_id,
  const geometry_msgs::msg::PoseStamped& tag_pose) {

  // Look up object info
  auto object_info = get_object_info_for_tag(tag_id);
  if (!object_info) {
    RCLCPP_DEBUG(node()->get_logger(),
      "No collision object defined for tag_id=%d. Skipping.", tag_id);
    return;
  }

  RCLCPP_INFO(node()->get_logger(),
    "Adding collision object '%s' for tag %d to planning scene",
    object_info->name.c_str(), tag_id);

  // Remove existing object if present (re-detection logic)
  remove_collision_object(object_info->name);

  // Calculate object pose from tag pose + offset
  auto object_pose = calculate_object_pose(tag_pose, object_info->tag_offset);

  RCLCPP_INFO(node()->get_logger(),
    "Tag pose (input): [%.3f, %.3f, %.3f] in frame '%s'",
    tag_pose.pose.position.x, tag_pose.pose.position.y, tag_pose.pose.position.z,
    tag_pose.header.frame_id.c_str());

  RCLCPP_INFO(node()->get_logger(),
    "Object pose (calculated): [%.3f, %.3f, %.3f] in frame '%s'",
    object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z,
    object_pose.header.frame_id.c_str());

  // Create collision object message
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = object_pose.header.frame_id;  // Use same frame as object pose
  collision_object.header.stamp = node()->now();
  collision_object.id = object_info->name;
  collision_object.operation = collision_object.ADD;

  // Verify frame consistency
  if (object_pose.header.frame_id != "base_link") {
    RCLCPP_WARN(node()->get_logger(),
      "Object pose is in frame '%s', expected 'base_link'. This may cause incorrect placement!",
      object_pose.header.frame_id.c_str());
  }

  // Create shape primitive
  shape_msgs::msg::SolidPrimitive primitive;

  if (object_info->shape == "box") {
    if (object_info->dimensions.size() != 3) {
      RCLCPP_ERROR(node()->get_logger(),
        "Box shape requires 3 dimensions [x, y, z], got %zu",
        object_info->dimensions.size());
      return;
    }
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = object_info->dimensions[0];  // x
    primitive.dimensions[1] = object_info->dimensions[1];  // y
    primitive.dimensions[2] = object_info->dimensions[2];  // z

  } else if (object_info->shape == "cylinder") {
    if (object_info->dimensions.size() != 2) {
      RCLCPP_ERROR(node()->get_logger(),
        "Cylinder shape requires 2 dimensions [height, radius], got %zu",
        object_info->dimensions.size());
      return;
    }
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = object_info->dimensions[0];  // height
    primitive.dimensions[1] = object_info->dimensions[1];  // radius

  } else {
    RCLCPP_ERROR(node()->get_logger(),
      "Unknown shape type: '%s'. Supported: 'box', 'cylinder'",
      object_info->shape.c_str());
    return;
  }

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(object_pose.pose);

  RCLCPP_INFO(node()->get_logger(),
    "Publishing collision object '%s' with dimensions [%.3f, %.3f, %.3f] at pose [%.3f, %.3f, %.3f]",
    object_info->name.c_str(),
    primitive.dimensions[0], primitive.dimensions[1], primitive.dimensions[2],
    object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z);

  // Apply to planning scene
  std::vector<moveit_msgs::msg::CollisionObject> objects = {collision_object};
  planning_scene_interface_->applyCollisionObjects(objects);

  RCLCPP_INFO(node()->get_logger(),
    "✓ Collision object '%s' successfully added to planning scene",
    object_info->name.c_str());
}

void VisionStages::remove_collision_object(const std::string& object_name) {
  // Check if object exists in scene
  auto known_objects = planning_scene_interface_->getKnownObjectNames();

  bool exists = std::find(known_objects.begin(), known_objects.end(), object_name)
                != known_objects.end();

  if (!exists) {
    RCLCPP_DEBUG(node()->get_logger(),
      "Object '%s' not in scene, skipping removal", object_name.c_str());
    return;
  }

  RCLCPP_INFO(node()->get_logger(),
    "Removing existing collision object: '%s'", object_name.c_str());

  std::vector<std::string> object_ids = {object_name};
  planning_scene_interface_->removeCollisionObjects(object_ids);
}
