#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_array.hpp> 

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_arm");

// // ---------------------- BEGIN: ArUco Integration Code ----------------------

// // This variable will store the detected ArUco marker pose.
// geometry_msgs::msg::Pose aruco_pose;
// // Flag to check if a marker has been detected.
// bool marker_detected = false;

// // Callback lambda that receives the PoseArray message from the ArUco node.
// // It takes the first pose from the array and stores it as the target pose.
// auto aruco_callback = [&](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
//   if (!msg->poses.empty() && !marker_detected) {
//     aruco_pose = msg->poses[0];  // Use the first detected marker
//     marker_detected = true;  
//     RCLCPP_INFO(logger, "ArUco marker detected: x=%.3f, y=%.3f, z=%.3f",
//                 aruco_pose.position.x, aruco_pose.position.y, aruco_pose.position.z);
//   }
// };

// // Subscribe to the "/aruco_poses" topic where the ArUco node publishes detected marker poses.
// auto aruco_subscription = node->create_subscription<geometry_msgs::msg::PoseArray>(
//     "/aruco_poses", 10, aruco_callback);

// // Wait until an ArUco marker is detected, with a timeout (e.g., 10 seconds).
// RCLCPP_INFO(logger, "Waiting for ArUco marker detection...");
// rclcpp::Rate rate(10);  // 10 Hz polling rate
// int timeout = 100;      // 100 iterations = 10 seconds timeout

// while (rclcpp::ok() && !marker_detected && timeout-- > 0) {
//   rclcpp::spin_some(node);  // Process incoming messages
//   rate.sleep();

//   // Optionally, print a waiting message every 2 seconds (every 20 iterations).
//   if (timeout % 20 == 0) {
//     RCLCPP_INFO(logger, "Still waiting for ArUco marker... %d seconds left", timeout / 10);
//   }
// }

// if (!marker_detected) {
//   RCLCPP_ERROR(logger, "No ArUco marker detected within timeout period. Exiting.");
//   rclcpp::shutdown();
//   return 1;
// }

// // Now that a marker has been detected, use its pose as the target pose for motion planning.
// move_group_interface.setPoseTarget(aruco_pose);

// ---------------------- END: ArUco Integration Code ----------------------



  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.position.x =  -0.075;
    msg.position.y = 0.294;
    msg.position.z = 0.6612;
    msg.orientation.w = -7;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}