#include "mtc_pipeline/vision_stages.hpp"
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

VisionStages::VisionStages(const rclcpp::Node::SharedPtr& node)
    : BaseStages(node)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    capture_marker_client_ = node->create_client<zivid_interfaces::srv::CaptureAndDetectMarkers>(
        "/capture_and_detect_markers");

    // Declare and get parameters
    node->declare_parameter("marker_dictionary", marker_dictionary_);
    node->declare_parameter("publish_marker_frames", publish_marker_frames_);
    node->declare_parameter("ik_frame", std::string(""));
    node->declare_parameter("z_offset", 0.0);

    marker_dictionary_ = node->get_parameter("marker_dictionary").as_string();
    publish_marker_frames_ = node->get_parameter("publish_marker_frames").as_bool();
    ik_frame_ = node->get_parameter("ik_frame").as_string();
    z_offset_ = node->get_parameter("z_offset").as_double();

    planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Load vision objects config
    std::string default_config = ament_index_cpp::get_package_share_directory("mtc_pipeline")
        + "/config/vision_objects.json";
    node->declare_parameter("vision_objects_config", default_config);
    vision_objects_config_path_ = node->get_parameter("vision_objects_config").as_string();
    load_vision_objects_config(vision_objects_config_path_);

    RCLCPP_INFO(node->get_logger(), "VisionStages initialized (ik_frame: %s)",
        ik_frame_.empty() ? "auto-detect" : ik_frame_.c_str());
}

bool VisionStages::run(const mtc_pipeline::action::VisionMoveToAction::Goal& goal)
{
    auto tag_pose = detect_and_transform_tag(goal.tag_id, goal.timeout);
    if (!tag_pose) {
        RCLCPP_ERROR(node()->get_logger(), "Failed to detect tag %d", goal.tag_id);
        return false;
    }
    return move_to_pose(*tag_pose);
}

std::optional<geometry_msgs::msg::PoseStamped> VisionStages::detect_and_transform_tag(
    int tag_id, double timeout)
{
    RCLCPP_INFO(node()->get_logger(), "Detecting tag %d...", tag_id);

    if (!capture_marker_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node()->get_logger(), "Zivid service not available");
        return std::nullopt;
    }

    auto request = std::make_shared<zivid_interfaces::srv::CaptureAndDetectMarkers::Request>();
    request->marker_ids = {tag_id};
    request->marker_dictionary = marker_dictionary_;

    auto future = capture_marker_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
        RCLCPP_ERROR(node()->get_logger(), "Zivid service timeout");
        return std::nullopt;
    }

    auto result = future.get();
    if (!result->success) {
        RCLCPP_ERROR(node()->get_logger(), "Detection failed: %s", result->message.c_str());
        return std::nullopt;
    }

    for (const auto& marker : result->detection_result.detected_markers) {
        if (marker.id == tag_id) {
            RCLCPP_INFO(node()->get_logger(), "Tag %d at [%.3f, %.3f, %.3f] in camera frame",
                marker.id, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);

            auto pose_base = transform_to_base_link(marker.pose);
            if (!pose_base) return std::nullopt;

            RCLCPP_INFO(node()->get_logger(), "Transformed to base_link: [%.3f, %.3f, %.3f]",
                pose_base->pose.position.x, pose_base->pose.position.y, pose_base->pose.position.z);

            if (publish_marker_frames_) broadcast_marker_tf(tag_id, *pose_base);
            add_collision_object_for_tag(tag_id, *pose_base);
            return pose_base;
        }
    }

    RCLCPP_WARN(node()->get_logger(), "Tag %d not in results (%zu markers)",
        tag_id, result->detection_result.detected_markers.size());
    return std::nullopt;
}

std::optional<geometry_msgs::msg::PoseStamped> VisionStages::transform_to_base_link(
    const geometry_msgs::msg::Pose& pose_camera)
{
    try {
        std::string camera_frame = "zivid_optical_frame";
        if (!tf_buffer_->canTransform("base_link", camera_frame, tf2::TimePointZero,
                                       std::chrono::seconds(1))) {
            RCLCPP_ERROR(node()->get_logger(), "TF %s -> base_link not available", camera_frame.c_str());
            return std::nullopt;
        }

        auto transform = tf_buffer_->lookupTransform("base_link", camera_frame, tf2::TimePointZero);

        geometry_msgs::msg::PoseStamped pose_in;
        pose_in.header.frame_id = camera_frame;
        pose_in.header.stamp = node()->now();
        pose_in.pose = pose_camera;

        geometry_msgs::msg::PoseStamped pose_out;
        tf2::doTransform(pose_in, pose_out, transform);
        pose_out.header.frame_id = "base_link";
        pose_out.header.stamp = node()->now();
        return pose_out;

    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(node()->get_logger(), "TF failed: %s", ex.what());
        return std::nullopt;
    }
}

void VisionStages::broadcast_marker_tf(int marker_id, const geometry_msgs::msg::PoseStamped& pose)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = node()->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "aruco_" + std::to_string(marker_id);
    tf.transform.translation.x = pose.pose.position.x;
    tf.transform.translation.y = pose.pose.position.y;
    tf.transform.translation.z = pose.pose.position.z;
    tf.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
}

bool VisionStages::move_to_pose(const geometry_msgs::msg::PoseStamped& target)
{
    std::string active_ik_frame;
    double active_z_offset;

    if (ik_frame_.empty()) {
        auto detection = detect_current_gripper();
        active_ik_frame = detection.ik_frame;
        active_z_offset = detection.z_offset;
        RCLCPP_INFO(node()->get_logger(), "Auto-detected: %s (z_offset: %.3f)",
            active_ik_frame.c_str(), active_z_offset);
    } else {
        active_ik_frame = ik_frame_;
        active_z_offset = (std::abs(z_offset_) < 1e-6)
            ? (active_ik_frame.find("epick") != std::string::npos ? 0.1 : -0.02)
            : z_offset_;
    }

    // Apply 180° Z rotation and z_offset
    geometry_msgs::msg::PoseStamped approach = target;

    tf2::Quaternion q_orig, q_rot;
    tf2::fromMsg(target.pose.orientation, q_orig);
    q_rot.setRPY(0, 0, M_PI);
    tf2::Quaternion q_final = q_orig * q_rot;
    q_final.normalize();
    approach.pose.orientation = tf2::toMsg(q_final);
    approach.pose.position.z += active_z_offset;

    RCLCPP_INFO(node()->get_logger(), "Moving to [%.3f, %.3f, %.3f] with 180° Z-rot, z_offset=%.3f",
        approach.pose.position.x, approach.pose.position.y, approach.pose.position.z, active_z_offset);

    auto task = create_task_template("Vision Move", "", active_ik_frame);
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to tag", make_cartesian_planner());
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(default_arm_group_name());
    stage->setGoal(approach);
    task.add(std::move(stage));

    return load_plan_execute(task);
}

VisionStages::GripperDetection VisionStages::detect_current_gripper()
{
    if (tf_buffer_->canTransform("base", "epick_tip", tf2::TimePointZero, std::chrono::seconds(1))) {
        return {"epick_tip", 0.027};
    }
    if (tf_buffer_->canTransform("base", "robotiq_hande_end", tf2::TimePointZero, std::chrono::seconds(1))) {
        return {"robotiq_hande_end", -0.02};
    }
    RCLCPP_INFO(node()->get_logger(), "No gripper detected, using flange");
    return {"flange", 0.0};
}

void VisionStages::load_vision_objects_config(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        RCLCPP_WARN(node()->get_logger(), "Could not open %s, collision objects disabled", path.c_str());
        return;
    }

    try {
        nlohmann::json config;
        file >> config;

        if (!config.contains("vision_objects")) return;

        for (auto& [tag_str, obj] : config["vision_objects"].items()) {
            ObjectInfo info;
            info.name = obj.at("name").get<std::string>();
            info.shape = obj.at("shape").get<std::string>();
            info.dimensions = obj.at("dimensions").get<std::vector<double>>();
            info.tag_offset = obj.at("tag_offset").get<std::vector<double>>();
            object_database_[std::stoi(tag_str)] = info;
        }
        RCLCPP_INFO(node()->get_logger(), "Loaded %zu vision objects", object_database_.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node()->get_logger(), "Config parse error: %s", e.what());
    }
}

std::optional<VisionStages::ObjectInfo> VisionStages::get_object_info_for_tag(int tag_id) const
{
    auto it = object_database_.find(tag_id);
    return (it != object_database_.end()) ? std::optional{it->second} : std::nullopt;
}

geometry_msgs::msg::PoseStamped VisionStages::calculate_object_pose(
    const geometry_msgs::msg::PoseStamped& tag_pose,
    const std::vector<double>& offset) const
{
    tf2::Quaternion q;
    tf2::fromMsg(tag_pose.pose.orientation, q);
    tf2::Vector3 offset_local(offset[0], offset[1], offset[2]);
    tf2::Vector3 offset_world = tf2::quatRotate(q, offset_local);

    geometry_msgs::msg::PoseStamped result = tag_pose;
    result.pose.position.x += offset_world.x();
    result.pose.position.y += offset_world.y();
    result.pose.position.z += offset_world.z();
    return result;
}

void VisionStages::add_collision_object_for_tag(int tag_id, const geometry_msgs::msg::PoseStamped& tag_pose)
{
    auto info = get_object_info_for_tag(tag_id);
    if (!info) return;

    remove_collision_object(info->name);

    auto object_pose = calculate_object_pose(tag_pose, info->tag_offset);

    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = object_pose.header.frame_id;
    obj.header.stamp = node()->now();
    obj.id = info->name;
    obj.operation = obj.ADD;

    shape_msgs::msg::SolidPrimitive prim;
    if (info->shape == "box" && info->dimensions.size() == 3) {
        prim.type = prim.BOX;
        prim.dimensions = {info->dimensions[0], info->dimensions[1], info->dimensions[2]};
    } else if (info->shape == "cylinder" && info->dimensions.size() == 2) {
        prim.type = prim.CYLINDER;
        prim.dimensions = {info->dimensions[0], info->dimensions[1]};
    } else {
        RCLCPP_ERROR(node()->get_logger(), "Invalid shape: %s", info->shape.c_str());
        return;
    }

    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(object_pose.pose);
    planning_scene_->applyCollisionObjects({obj});

    RCLCPP_INFO(node()->get_logger(), "Added collision object '%s'", info->name.c_str());
}

void VisionStages::remove_collision_object(const std::string& name)
{
    auto known = planning_scene_->getKnownObjectNames();
    if (std::find(known.begin(), known.end(), name) != known.end()) {
        planning_scene_->removeCollisionObjects({name});
    }
}
