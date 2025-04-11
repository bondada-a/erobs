/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_array.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

// ArucoPoseHandler class for subscribing to aruco poses and transforming them
class ArucoPoseHandler : public rclcpp::Node
{
public:
  ArucoPoseHandler() : Node("aruco_pose_handler")
  {
    // Hardcoded transformation matrix from frame C to frame M
    transform_MC_ = Eigen::Matrix4d();
    transform_MC_ << 
      -0.424,  0.034, -0.905,  0.807,
       0.906, -0.004, -0.424,  0.417,
      -0.018, -0.999, -0.029,  0.257,
       0.000,  0.000,  0.000,  1.000;
    
    // Create subscription to aruco_poses
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/aruco_poses", 10, 
      std::bind(&ArucoPoseHandler::aruco_pose_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "ArucoPoseHandler initialized, waiting for poses...");
  }

  bool has_pose() const
  {
    return has_pose_;
  }

  geometry_msgs::msg::Pose get_latest_transformed_pose() const
  {
    return latest_transformed_pose_;
  }

private:
  void aruco_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty ArUco pose array");
      return;
    }

    // Get the first pose (Frame A in Frame C)
    geometry_msgs::msg::Pose pose_in_C = msg->poses[0];
    
    // Log the original pose
    RCLCPP_INFO(this->get_logger(), 
                "Received pose in frame C - Position: (%.6f, %.6f, %.6f)", 
                pose_in_C.position.x, pose_in_C.position.y, pose_in_C.position.z);
    
    // Transform to frame M
    geometry_msgs::msg::Pose pose_in_M = transform_pose(pose_in_C);
    
    latest_transformed_pose_ = pose_in_M;
    has_pose_ = true;
    
    // Log the transformed pose
    RCLCPP_INFO(this->get_logger(), 
                "Transformed pose in frame M - Position: (%.6f, %.6f, %.6f)", 
                pose_in_M.position.x, pose_in_M.position.y, pose_in_M.position.z);
  }

  geometry_msgs::msg::Pose transform_pose(const geometry_msgs::msg::Pose& pose_in_C)
  {
    // Create homogeneous position vector
    Eigen::Vector4d pos_C(pose_in_C.position.x, 
                         pose_in_C.position.y, 
                         pose_in_C.position.z, 
                         1.0);
    
    // Apply transformation
    Eigen::Vector4d pos_M = transform_MC_ * pos_C;
    
    // Create output pose (keeping the original orientation for now)
    geometry_msgs::msg::Pose pose_in_M;
    pose_in_M.position.x = pos_M(0);
    pose_in_M.position.y = pos_M(1);
    pose_in_M.position.z = 0.4;//pos_M(2);
    
    // Set orientation - using same orientation as input for now
    // In a real application, you might want to transform the orientation too
    pose_in_M.orientation = pose_in_C.orientation;
    
    return pose_in_M;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  geometry_msgs::msg::Pose latest_transformed_pose_;
  bool has_pose_ = false;
  Eigen::Matrix4d transform_MC_;
};

int main(int argc, char * argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the ArucoPoseHandler node
  auto aruco_handler_node = std::make_shared<ArucoPoseHandler>();
  
  // Create a parameter client node
  auto parameter_client_node = rclcpp::Node::make_shared("param_client");
  auto parent_parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(parameter_client_node, "move_group");
  
  // Wait for move_group service
  while (!parent_parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "move_group service not available, waiting again...");
  }

  // Get robot config parameters from parameter server
  auto parameters = parent_parameters_client->get_parameters(
    {"robot_description_semantic", "robot_description"});

  // Create the Node for moveit
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  std::string parameter_value = parameters[0].value_to_string();
  node->declare_parameter<std::string>("robot_description_semantic", parameter_value);
  parameter_value = parameters[1].value_to_string();
  node->declare_parameter<std::string>("robot_description", parameter_value);

  // Create the MoveIt MoveGroup Interface
  RCLCPP_INFO(logger, "Assembling move_group_interface");
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_arm");

  // Spin the ArucoPoseHandler node until we get a pose
  RCLCPP_INFO(logger, "Waiting for ArUco poses...");
  while (!aruco_handler_node->has_pose()) {
    rclcpp::spin_some(aruco_handler_node);
    std::this_thread::sleep_for(100ms);
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "ROS context is no longer valid");
      return 1;
    }
  }

  // Get the transformed pose from the ArucoPoseHandler
  auto target_pose = aruco_handler_node->get_latest_transformed_pose();
  
  // Set orientation (since we didn't transform it)
  tf2::Quaternion q;
  q.setRPY(0.0, 1.57, 1.57);  // Example orientation
  q.normalize();
  target_pose.orientation = tf2::toMsg(q);
  
  RCLCPP_INFO(logger, "Setting target pose: (%.6f, %.6f, %.6f)",
              target_pose.position.x, target_pose.position.y, target_pose.position.z);
  
  // Set the target pose
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

  // Execute the plan
  if (success) {
    RCLCPP_INFO(logger, "Planning succeeded! Executing plan...");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
