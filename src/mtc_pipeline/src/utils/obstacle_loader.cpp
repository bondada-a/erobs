#include "mtc_pipeline/obstacle_loader.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace mtc_pipeline {

geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::Quaternion msg_quat;
    msg_quat.x = tf_quat.x();
    msg_quat.y = tf_quat.y();
    msg_quat.z = tf_quat.z();
    msg_quat.w = tf_quat.w();

    return msg_quat;
}

bool loadPlanningSceneObstacles(const rclcpp::Logger& logger, const std::string& yaml_path) {
    RCLCPP_INFO(logger, "Loading planning scene obstacles from: %s", yaml_path.c_str());

    try {
        // Create MoveIt planning scene interface
        moveit::planning_interface::PlanningSceneInterface scene_interface;

        // Load YAML file
        YAML::Node config = YAML::LoadFile(yaml_path);

        if (!config["obstacles"]) {
            RCLCPP_WARN(logger, "No 'obstacles' key found in YAML");
            return true;  // Empty scene is valid
        }

        // Build collision objects from YAML
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

        for (const auto& obs : config["obstacles"]) {
            moveit_msgs::msg::CollisionObject collision_object;

            // Basic info
            collision_object.id = obs["name"].as<std::string>();
            collision_object.header.frame_id = obs["frame"].as<std::string>();
            collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

            // Parse pose
            geometry_msgs::msg::Pose pose;
            pose.position.x = obs["pose"]["x"].as<double>();
            pose.position.y = obs["pose"]["y"].as<double>();
            pose.position.z = obs["pose"]["z"].as<double>();

            double roll = obs["pose"]["roll"].as<double>();
            double pitch = obs["pose"]["pitch"].as<double>();
            double yaw = obs["pose"]["yaw"].as<double>();
            pose.orientation = rpyToQuaternion(roll, pitch, yaw);

            // Parse primitive based on type
            shape_msgs::msg::SolidPrimitive primitive;
            std::string type = obs["type"].as<std::string>();

            if (type == "box") {
                primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
                primitive.dimensions.resize(3);
                std::vector<double> size = obs["size"].as<std::vector<double>>();
                primitive.dimensions[0] = size[0];  // width (x)
                primitive.dimensions[1] = size[1];  // depth (y)
                primitive.dimensions[2] = size[2];  // height (z)

            } else if (type == "cylinder") {
                primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
                primitive.dimensions.resize(2);
                primitive.dimensions[0] = obs["height"].as<double>();  // height
                primitive.dimensions[1] = obs["radius"].as<double>();  // radius

            } else if (type == "sphere") {
                primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
                primitive.dimensions.resize(1);
                primitive.dimensions[0] = obs["radius"].as<double>();  // radius

            } else {
                RCLCPP_WARN(logger, "Unknown obstacle type '%s' for '%s', skipping",
                           type.c_str(), collision_object.id.c_str());
                continue;
            }

            // Add primitive and pose to collision object
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose);

            collision_objects.push_back(collision_object);

            RCLCPP_INFO(logger, "  - Added %s '%s' in frame '%s'",
                       type.c_str(), collision_object.id.c_str(),
                       collision_object.header.frame_id.c_str());
        }

        // Apply all obstacles at once (publishes to /planning_scene)
        if (!collision_objects.empty()) {
            scene_interface.applyCollisionObjects(collision_objects);
            RCLCPP_INFO(logger, "✓ Successfully loaded %zu obstacles", collision_objects.size());
        } else {
            RCLCPP_INFO(logger, "No obstacles to load");
        }

        return true;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(logger, "YAML parsing error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Failed to load obstacles: %s", e.what());
        return false;
    }
}

}  // namespace mtc_pipeline
