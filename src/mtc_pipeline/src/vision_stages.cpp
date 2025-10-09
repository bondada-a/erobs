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
  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribe to AprilTag detections
  tag_subscription_ = node->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "/detections",
    10,
    [this, node](apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
      RCLCPP_INFO(node->get_logger(), "Received %zu AprilTag detections", msg->detections.size());
      for (const auto& det : msg->detections) {
        RCLCPP_INFO(node->get_logger(), "  - Tag ID: %d, Family: %s", det.id, det.family.c_str());
      }
      latest_detections_ = msg;
    }
  );

  // Create service client for Zivid camera capture
  capture_client_ = node->create_client<std_srvs::srv::Trigger>("/capture_2d");

  RCLCPP_INFO(node->get_logger(), "VisionStages initialized");
}

bool VisionStages::run(const nlohmann::json& step, const nlohmann::json& poses)
{
  // Parse minimal parameters
  const int tag_id = step.at("tag_id").get<int>();
  const double timeout = step.value("timeout", 10.0);

  // Trigger Zivid camera capture
  RCLCPP_INFO(node()->get_logger(), "Triggering camera capture...");
  if (!trigger_capture()) {
    RCLCPP_ERROR(node()->get_logger(), "Failed to trigger camera capture");
    return false;
  }

  RCLCPP_INFO(node()->get_logger(), "Detecting tag %d...", tag_id);

  // Detect the tag
  auto tag_pose_opt = detect_tag(tag_id, timeout);
  if (!tag_pose_opt) {
    RCLCPP_ERROR(node()->get_logger(), "Failed to detect tag %d", tag_id);
    return false;
  }

  RCLCPP_INFO(node()->get_logger(),
    "Tag %d detected at [%.3f, %.3f, %.3f]",
    tag_id,
    tag_pose_opt->pose.position.x,
    tag_pose_opt->pose.position.y,
    tag_pose_opt->pose.position.z);

  // Move to the detected pose
  return move_to_pose(*tag_pose_opt);
}

bool VisionStages::trigger_capture()
{
  // Wait for service
  if (!capture_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node()->get_logger(), "Capture service not available");
    return false;
  }

  // Call capture service
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = capture_client_->async_send_request(request);

  // Wait for result (Zivid captures can take ~1-2 seconds)
  // Use wait_for instead of spin_until_future_complete to avoid executor conflict
  auto status = future.wait_for(std::chrono::seconds(5));
  if (status != std::future_status::ready)
  {
    RCLCPP_ERROR(node()->get_logger(), "Capture service call timeout");
    return false;
  }

  auto result = future.get();
  if (!result->success) {
    RCLCPP_ERROR(node()->get_logger(), "Capture failed: %s", result->message.c_str());
    return false;
  }

  RCLCPP_INFO(node()->get_logger(), "Camera capture successful");
  return true;
}

std::optional<geometry_msgs::msg::PoseStamped> VisionStages::detect_tag(
  int tag_id,
  double timeout_seconds)
{
  const auto start_time = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::duration<double>(timeout_seconds);

  latest_detections_ = nullptr;

  while (rclcpp::ok()) {
    // Check timeout
    if (std::chrono::steady_clock::now() - start_time > timeout) {
      RCLCPP_WARN(node()->get_logger(), "Tag detection timeout");
      return std::nullopt;
    }

    // Don't call spin_some - node is already being spun by action server executor
    // Just check if data has arrived via subscription callback

    // Check for detections
    if (latest_detections_ && !latest_detections_->detections.empty()) {
      for (const auto& detection : latest_detections_->detections) {
        if (detection.id == tag_id) {
          // Tag frame format: "tag36h11:ID"
          std::string tag_frame = detection.family + ":" + std::to_string(detection.id);

          try {
            // Get transform from tag to base_link
            if (!tf_buffer_->canTransform("base_link", tag_frame,
                                         tf2::TimePointZero, std::chrono::milliseconds(100))) {
              continue;
            }

            auto transform = tf_buffer_->lookupTransform("base_link", tag_frame, tf2::TimePointZero);

            // Convert to PoseStamped
            geometry_msgs::msg::PoseStamped tag_pose;
            tag_pose.header.frame_id = "base_link";
            tag_pose.header.stamp = node()->now();
            tag_pose.pose.position.x = transform.transform.translation.x;
            tag_pose.pose.position.y = transform.transform.translation.y;
            tag_pose.pose.position.z = transform.transform.translation.z;
            tag_pose.pose.orientation = transform.transform.rotation;

            return tag_pose;

          } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node()->get_logger(), "TF error: %s", ex.what());
            continue;
          }
        }
      }
    }

    // Sleep briefly to allow callbacks to be processed by main executor
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return std::nullopt;
}

bool VisionStages::move_to_pose(const geometry_msgs::msg::PoseStamped& target_pose)
{
  // Create approach pose by applying rotation to tag pose
  // This ensures the gripper approaches from the correct direction
  geometry_msgs::msg::PoseStamped approach_pose = target_pose;

  // Convert quaternion to tf2
  tf2::Quaternion tag_orientation;
  tf2::fromMsg(target_pose.pose.orientation, tag_orientation);

  // Create rotation: 180° around Y-axis to flip approach direction
  // This makes the TCP approach the tag from the correct side
  tf2::Quaternion approach_rotation;
  approach_rotation.setRPY(0, M_PI, 0);  // 180° around Y

  // Apply rotation: new_orientation = tag_orientation * approach_rotation
  tf2::Quaternion final_orientation = tag_orientation * approach_rotation;
  final_orientation.normalize();

  // Convert back to message
  approach_pose.pose.orientation = tf2::toMsg(final_orientation);

  RCLCPP_INFO(node()->get_logger(),
    "Approach pose: pos=[%.3f, %.3f, %.3f]",
    approach_pose.pose.position.x,
    approach_pose.pose.position.y,
    approach_pose.pose.position.z);

  // Create MTC task with TCP frame set in template
  auto task = create_task_template("Vision Move", "", "robotiq_hande_tcp");

  // Use Cartesian planner for straight-line TCP motion to detected pose
  // This creates more direct paths than OMPL joint-space planning
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
