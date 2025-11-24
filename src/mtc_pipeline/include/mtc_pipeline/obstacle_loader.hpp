#ifndef OBSTACLE_LOADER_HPP
#define OBSTACLE_LOADER_HPP

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace mtc_pipeline {

/**
 * @brief Load planning scene obstacles from YAML file
 *
 * Reads obstacle definitions from YAML and publishes them to MoveIt's
 * planning scene using PlanningSceneInterface. Supports box, cylinder,
 * and sphere primitives with full 6DOF poses (position + RPY orientation).
 *
 * @param logger ROS 2 logger for status messages
 * @param yaml_path Absolute path to YAML config file
 * @return true if obstacles loaded successfully, false otherwise
 *
 * YAML Format:
 *   obstacles:
 *     - name: "table"
 *       type: "box"  # box, cylinder, or sphere
 *       frame: "world"
 *       pose:
 *         x: 0.0
 *         y: 0.0
 *         z: 0.5
 *         roll: 0.0
 *         pitch: 0.0
 *         yaw: 0.0
 *       size: [1.0, 2.0, 0.05]  # For box: [width, depth, height]
 *       # OR for cylinder: height: 1.0, radius: 0.1
 *       # OR for sphere: radius: 0.2
 */
bool loadPlanningSceneObstacles(
    const rclcpp::Logger& logger,
    const std::string& yaml_path
);

/**
 * @brief Convert roll-pitch-yaw to quaternion
 * @param roll Rotation around X axis (radians)
 * @param pitch Rotation around Y axis (radians)
 * @param yaw Rotation around Z axis (radians)
 * @return Quaternion as geometry_msgs message
 */
geometry_msgs::msg::Quaternion rpyToQuaternion(
    double roll,
    double pitch,
    double yaw
);

}  // namespace mtc_pipeline

#endif  // OBSTACLE_LOADER_HPP
