#include "mtc_pipeline/vision_stages.hpp"
#include <moveit/task_constructor/stages/move_to.h>
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
    "/apriltag/detections",
    10,
    [this](apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
      latest_detections_ = msg;
    }
  );

  RCLCPP_INFO(node->get_logger(), "VisionStages initialized");
}

bool VisionStages::run(const nlohmann::json& step, const nlohmann::json& poses)
{
  // Parse minimal parameters
  const int tag_id = step.at("tag_id").get<int>();
  const double timeout = step.value("timeout", 5.0);

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

    // Process callbacks
    rclcpp::spin_some(node());

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

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return std::nullopt;
}

bool VisionStages::move_to_pose(const geometry_msgs::msg::PoseStamped& target_pose)
{
  // Create MTC task
  auto task = create_task_template("Vision Move");

  // Create simple MoveTo stage
  auto move_stage = std::make_unique<mtc::stages::MoveTo>("move to tag", make_pipeline_planner());
  move_stage->setGroup(default_arm_group_name());
  move_stage->setGoal(target_pose);

  task.add(std::move(move_stage));

  // Execute
  return load_plan_execute(task);
}
