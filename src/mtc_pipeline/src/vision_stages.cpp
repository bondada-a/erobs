#include "mtc_pipeline/vision_stages.hpp"

#include <moveit/task_constructor/stages/move_to.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <chrono>
#include <thread>

VisionStages::VisionStages(const rclcpp::Node::SharedPtr& node)
  : BaseStages(node),
    robot_base_frame_("base_link"),
    camera_frame_("zivid_optical_frame")
{
  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribe to AprilTag detections
  tag_subscription_ = node->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "/apriltag/detections",  // Default topic from apriltag_ros
    10,
    [this](apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
      latest_detections_ = msg;
    }
  );

  RCLCPP_INFO(node->get_logger(), "VisionStages initialized - waiting for AprilTag detections");
}

bool VisionStages::run(const nlohmann::json& step, const nlohmann::json& poses)
{
  // Parse parameters from JSON
  const int tag_id = step.at("tag_id").get<int>();
  const double approach_distance = step.value("approach_distance", 0.1);
  const double timeout = step.value("timeout", 5.0);
  const std::string approach_direction = step.value("approach_direction", "z");
  const bool use_preset_height = step.value("use_preset_height", false);
  const double preset_height = step.value("preset_height", 0.15);
  const std::string planning_type = step.value("planning_type", "joint");

  RCLCPP_INFO(node()->get_logger(),
    "Vision task: Detect tag %d and move to %fm from %s direction",
    tag_id, approach_distance, approach_direction.c_str());

  // Detect the tag
  auto tag_pose_opt = detect_tag(tag_id, timeout);
  if (!tag_pose_opt) {
    RCLCPP_ERROR(node()->get_logger(), "Failed to detect tag %d within %.1f seconds", tag_id, timeout);
    return false;
  }

  RCLCPP_INFO(node()->get_logger(),
    "Tag %d detected at position: [%.3f, %.3f, %.3f]",
    tag_id,
    tag_pose_opt->pose.position.x,
    tag_pose_opt->pose.position.y,
    tag_pose_opt->pose.position.z);

  // Calculate approach pose
  auto approach_pose = calculate_approach_pose(
    *tag_pose_opt,
    approach_direction,
    approach_distance,
    use_preset_height,
    preset_height
  );

  RCLCPP_INFO(node()->get_logger(),
    "Approach pose calculated at: [%.3f, %.3f, %.3f]",
    approach_pose.pose.position.x,
    approach_pose.pose.position.y,
    approach_pose.pose.position.z);

  // Create and execute movement to approach pose
  return create_vision_move_task(approach_pose, planning_type);
}

std::optional<geometry_msgs::msg::PoseStamped> VisionStages::detect_tag(
  int tag_id,
  double timeout_seconds)
{
  const auto start_time = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::duration<double>(timeout_seconds);

  // Clear any old detections
  latest_detections_ = nullptr;

  while (rclcpp::ok()) {
    // Check timeout
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed > timeout) {
      RCLCPP_WARN(node()->get_logger(), "Tag detection timeout after %.1f seconds", timeout_seconds);
      return std::nullopt;
    }

    // Process callbacks
    rclcpp::spin_some(node());

    // Check if we have detections
    if (latest_detections_ && !latest_detections_->detections.empty()) {
      for (const auto& detection : latest_detections_->detections) {
        if (detection.id == tag_id) {
          // The pose is published via TF, not in the detection message
          // Construct the tag frame name (format: "tag36h11:ID")
          std::string tag_frame = detection.family + ":" + std::to_string(detection.id);

          try {
            // Look up the transform from tag frame to base frame
            if (!tf_buffer_->canTransform(robot_base_frame_, tag_frame,
                                         tf2::TimePointZero, std::chrono::milliseconds(500))) {
              RCLCPP_WARN(node()->get_logger(),
                "Transform not available from %s to %s, retrying...",
                tag_frame.c_str(), robot_base_frame_.c_str());
              continue;
            }

            auto transform = tf_buffer_->lookupTransform(
              robot_base_frame_, tag_frame, tf2::TimePointZero);

            // Convert transform to PoseStamped
            geometry_msgs::msg::PoseStamped tag_pose;
            tag_pose.header.frame_id = robot_base_frame_;
            tag_pose.header.stamp = node()->now();
            tag_pose.pose.position.x = transform.transform.translation.x;
            tag_pose.pose.position.y = transform.transform.translation.y;
            tag_pose.pose.position.z = transform.transform.translation.z;
            tag_pose.pose.orientation = transform.transform.rotation;

            RCLCPP_INFO(node()->get_logger(),
              "Tag %d detected at [%.3f, %.3f, %.3f]",
              tag_id, tag_pose.pose.position.x,
              tag_pose.pose.position.y, tag_pose.pose.position.z);

            return tag_pose;
          } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node()->get_logger(),
              "Failed to get transform for tag %d: %s", tag_id, ex.what());
            continue;
          }
        }
      }
    }

    // Small sleep to avoid busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return std::nullopt;
}

geometry_msgs::msg::PoseStamped VisionStages::calculate_approach_pose(
  const geometry_msgs::msg::PoseStamped& tag_pose,
  const std::string& approach_direction,
  double approach_distance,
  bool use_preset_height,
  double preset_height)
{
  geometry_msgs::msg::PoseStamped approach_pose = tag_pose;

  // Apply offset based on approach direction
  if (approach_direction == "x" || approach_direction == "forward") {
    approach_pose.pose.position.x -= approach_distance;
  } else if (approach_direction == "-x" || approach_direction == "backward") {
    approach_pose.pose.position.x += approach_distance;
  } else if (approach_direction == "y" || approach_direction == "right") {
    approach_pose.pose.position.y -= approach_distance;
  } else if (approach_direction == "-y" || approach_direction == "left") {
    approach_pose.pose.position.y += approach_distance;
  } else if (approach_direction == "z" || approach_direction == "up") {
    approach_pose.pose.position.z += approach_distance;
  } else if (approach_direction == "-z" || approach_direction == "down") {
    approach_pose.pose.position.z -= approach_distance;
  } else {
    RCLCPP_WARN(node()->get_logger(), "Unknown approach direction '%s', using 'z'", approach_direction.c_str());
    approach_pose.pose.position.z += approach_distance;
  }

  // Override Z if requested (useful for table-top manipulation)
  if (use_preset_height) {
    approach_pose.pose.position.z = preset_height;
  }

  // For now, keep the same orientation as the tag
  // In the future, we might want to align the gripper perpendicular to the tag

  return approach_pose;
}

// Note: This function is kept for potential future use if we need to transform
// poses from camera frame. Currently, apriltag_ros publishes transforms directly
// via TF, so we get poses already in the desired frame.
std::optional<geometry_msgs::msg::PoseStamped> VisionStages::transform_to_base_frame(
  const geometry_msgs::msg::PoseStamped& pose_in_camera_frame)
{
  try {
    // Wait for transform to be available
    if (!tf_buffer_->canTransform(robot_base_frame_, pose_in_camera_frame.header.frame_id,
                                   tf2::TimePointZero, std::chrono::seconds(1))) {
      RCLCPP_ERROR(node()->get_logger(),
        "Transform not available from %s to %s",
        pose_in_camera_frame.header.frame_id.c_str(),
        robot_base_frame_.c_str());
      return std::nullopt;
    }

    // Transform the pose
    geometry_msgs::msg::PoseStamped transformed_pose;
    tf2::doTransform(pose_in_camera_frame, transformed_pose,
                     tf_buffer_->lookupTransform(robot_base_frame_,
                                                  pose_in_camera_frame.header.frame_id,
                                                  tf2::TimePointZero));

    transformed_pose.header.frame_id = robot_base_frame_;
    transformed_pose.header.stamp = node()->now();

    return transformed_pose;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node()->get_logger(), "Transform failed: %s", ex.what());
    return std::nullopt;
  }
}

bool VisionStages::create_vision_move_task(
  const geometry_msgs::msg::PoseStamped& target_pose,
  const std::string& planning_type)
{
  // Create MTC task
  auto task = create_task_template("Vision Move Task");

  // Select planner based on planning type
  auto planner = (planning_type == "cartesian") ?
                  make_cartesian_planner() : make_pipeline_planner();

  // Create MoveTo stage for target pose
  auto move_stage = std::make_unique<mtc::stages::MoveTo>("move to detected pose", planner);
  move_stage->setGroup(default_arm_group_name());

  // Set the target pose
  // Note: MoveIt expects pose in planning frame (usually base_link)
  move_stage->setGoal(target_pose);

  task.add(std::move(move_stage));

  // Plan and execute
  return load_plan_execute(task);
}