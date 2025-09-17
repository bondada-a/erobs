#include "mtc_pipeline/moveto_stages.hpp"
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <atomic>
#include <chrono>
#include <boost/algorithm/string/join.hpp>

namespace mtc = moveit::task_constructor;

MoveToStages::MoveToStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
    : node_(node), config_(config) {}

std::vector<std::string> MoveToStages::getJointNames() {
    return {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
}

// Convert degrees to radians for joint angles
std::map<std::string, double> MoveToStages::convertDegreesToRadians(const std::vector<double>& angles_deg) {
    const std::vector<std::string> joint_names = getJointNames();
    std::map<std::string, double> joint_goal;
    
    for (size_t i = 0; i < std::min(angles_deg.size(), joint_names.size()); ++i) {
        joint_goal[joint_names[i]] = angles_deg[i] * M_PI / 180.0;
    }
    
    return joint_goal;
}

// Create move stage to named pose or joint angles
std::unique_ptr<mtc::Stage> MoveToStages::makeMoveToNamedStage(
    const std::string& label,
    const std::string& pose_key,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name,
    bool is_named_state) {
    
    if (is_named_state) {
        // For named states, we need to get the joint values from the robot model
        // This will be handled in the run function where we have access to the robot model
        throw std::runtime_error("Named states should be handled in the run function, not in makeMoveToNamedStage");
    } else {
        // For joint poses, use MoveTo with planner
        auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
        stage->setGroup(arm_group_name);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
        stage->setIKFrame("flange");
        
        auto& angles_deg = config_["poses"][pose_key];
        if (!angles_deg.is_array() || angles_deg.size() != 6) {
            throw std::runtime_error(pose_key + " must be an array of 6 numbers");
        }
        
        std::vector<double> angles_vec;
        for (const auto& angle : angles_deg) {
            angles_vec.push_back(angle.get<double>());
        }
        
        auto joint_goal = convertDegreesToRadians(angles_vec);
        stage->setGoal(joint_goal);
        return stage;
    }
}

// Create move stage to specific joint angles
std::unique_ptr<mtc::Stage> MoveToStages::makeMoveToJointStage(
    const std::string& label,
    const std::vector<double>& joint_angles,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name) {
    
    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->setGroup(arm_group_name);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
    stage->setIKFrame("flange");
    
    auto joint_goal = convertDegreesToRadians(joint_angles);
    stage->setGoal(joint_goal);
    
    return stage;
}

// Create move stage to specific pose
std::unique_ptr<mtc::Stage> MoveToStages::makeMoveToPoseStage(
    const std::string& label,
    const geometry_msgs::msg::PoseStamped& pose,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name) {
    
    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->setGroup(arm_group_name);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
    stage->setIKFrame("flange");
    stage->setGoal(pose);
    
    return stage;
}

// Create relative move stage
std::unique_ptr<mtc::Stage> MoveToStages::makeMoveRelativeStage(
    const std::string& label,
    const std::string& direction,
    double distance,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name) {
    
    auto stage = std::make_unique<mtc::stages::MoveRelative>(label, planner);
    stage->properties().set("marker_ns", "relative_move");
    stage->properties().set("link", "flange");
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
    stage->setGroup(arm_group_name);
    stage->setMinMaxDistance(std::abs(distance), std::abs(distance));
    
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "flange";
    
    // Set direction based on input
    if (direction == "forward" || direction == "x") {
        vec.vector.x = (distance >= 0.0) ? 1.0 : -1.0;
    } else if (direction == "right" || direction == "y") {
        vec.vector.y = (distance >= 0.0) ? 1.0 : -1.0;
    } else if (direction == "up" || direction == "z") {
        vec.vector.z = (distance >= 0.0) ? 1.0 : -1.0;
    } else if (direction == "backward" || direction == "-x") {
        vec.vector.x = (distance >= 0.0) ? -1.0 : 1.0;
    } else if (direction == "left" || direction == "-y") {
        vec.vector.y = (distance >= 0.0) ? -1.0 : 1.0;
    } else if (direction == "down" || direction == "-z") {
        vec.vector.z = (distance >= 0.0) ? -1.0 : 1.0;
    } else {
        throw std::runtime_error("Invalid direction: " + direction + 
            ". Use: forward/x, right/y, up/z, backward/-x, left/-y, down/-z");
    }
    
    stage->setDirection(vec);
    return stage;
}

// Execute move to task
bool MoveToStages::run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node) {
    std::string target_type = step.value("target_type", "pose");
    std::string planning_type = step.value("planning_type", "joint");
    std::string arm_group_name = step.value("arm_group", "ur_arm");
    
    // Update config with poses
    config_["poses"] = poses;
    
    // Create planner based on planning type
    mtc::solvers::PlannerInterfacePtr planner;
    
    if (planning_type == "cartesian") {
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.2);
        cartesian_planner->setMaxAccelerationScalingFactor(0.2);
        cartesian_planner->setStepSize(0.001);
        cartesian_planner->setMinFraction(0.8);
        planner = cartesian_planner;
    } else {
        auto joint_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl");
        joint_planner->setMaxVelocityScalingFactor(0.2);
        joint_planner->setMaxAccelerationScalingFactor(0.2);
        planner = joint_planner;
    }
    
    // Create task
    mtc::Task task;
    task.stages()->setName("MoveTo Task");
    
    task.setProperty("group", arm_group_name);
    
    // Set up IK frame
    geometry_msgs::msg::PoseStamped ik_frame_pose;
    ik_frame_pose.header.frame_id = "flange";
    ik_frame_pose.pose.orientation.w = 1.0;
    task.setProperty("ik_frame", ik_frame_pose);
    
    // Add current state
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));
    
    // Add movement stage based on target type
    if (target_type == "named_state") {
        std::string named_state = step["target"];
        // For named states, we'll handle this after loading the robot model
        // to get the actual joint values
    } else if (target_type == "joints") {
        std::vector<double> joint_angles = step["target"].get<std::vector<double>>();
        task.add(makeMoveToJointStage("move_to_joints", joint_angles, planner, arm_group_name));
    } else if (target_type == "relative") {
        std::string direction = step["direction"];
        double distance = step["distance"].get<double>();
        task.add(makeMoveRelativeStage("move_relative", direction, distance, planner, arm_group_name));
    } else {
        // Default: pose from JSON
        std::string pose_key = step["target"];
        task.add(makeMoveToNamedStage("move_to_" + pose_key, pose_key, planner, arm_group_name, false));
    }
    
    // Initialize and execute task
    try {
        // Load robot model
        task.loadRobotModel(node);
        
        // Debug: Print available groups and states
        auto robot_model = task.getRobotModel();
        if (robot_model) {
            RCLCPP_INFO(node->get_logger(), "Robot model loaded successfully");
            RCLCPP_INFO(node->get_logger(), "Available groups: %s", 
                       boost::algorithm::join(robot_model->getJointModelGroupNames(), ", ").c_str());
            
            if (target_type == "named_state") {
                std::string named_state = step["target"];
                auto group = robot_model->getJointModelGroup(arm_group_name);
                if (group) {
                    RCLCPP_INFO(node->get_logger(), "Group '%s' found", arm_group_name.c_str());
                    
                    // Get the named state from the robot model
                    // For now, we'll use hardcoded values for common named states
                    std::map<std::string, double> joint_goal;
                    if (named_state == "moveit_home") {
                        // These values are from the SRDF file we saw earlier
                        joint_goal["shoulder_pan_joint"] = 0.0;
                        joint_goal["shoulder_lift_joint"] = -1.578;
                        joint_goal["elbow_joint"] = 0.0;
                        joint_goal["wrist_1_joint"] = 0.0;
                        joint_goal["wrist_2_joint"] = 0.0;
                        joint_goal["wrist_3_joint"] = -1.5782;
                        
                        // Create and add the move stage
                        auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_" + named_state, planner);
                        stage->setGroup(arm_group_name);
                        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
                        stage->setIKFrame("flange");
                        stage->setGoal(joint_goal);
                        task.add(std::move(stage));
                        
                        RCLCPP_INFO(node->get_logger(), "Added move stage for named state: %s", named_state.c_str());
                    } else {
                        RCLCPP_ERROR(node->get_logger(), "Named state '%s' not implemented", named_state.c_str());
                        return false;
                    }
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Group '%s' not found in robot model", arm_group_name.c_str());
                    return false;
                }
            }
        }
        
        task.init();
    } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Stage initialization failed: " << e);
        return false;
    }
    
    if (!task.plan(5)) {
        RCLCPP_ERROR(node->get_logger(), "Task planning failed");
        return false;
    }
    
    if (task.solutions().empty()) {
        RCLCPP_ERROR(node->get_logger(), "No solutions found to execute");
        return false;
    }
    
    auto solution = task.solutions().front();
    auto result = task.execute(*solution);
    
    if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "MoveTo task completed successfully");
    } else {
        RCLCPP_ERROR(node->get_logger(), "MoveTo task failed with code: %d", result.val);
    }
    
    return (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

// Main orchestrator step runner with cancellation support
bool MoveToStages::run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node, 
                      std::function<bool()> should_cancel) {
    std::string target_type = step.value("target_type", "pose");
    std::string planning_type = step.value("planning_type", "joint");
    std::string arm_group_name = step.value("arm_group", "ur_arm");
    
    // FSM-style: No cancellation check before starting
    
    // Update config with poses
    config_["poses"] = poses;
    
    // Create planner based on planning type
    mtc::solvers::PlannerInterfacePtr planner;
    
    if (planning_type == "cartesian") {
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.2);
        cartesian_planner->setMaxAccelerationScalingFactor(0.2);
        cartesian_planner->setStepSize(0.001);
        cartesian_planner->setMinFraction(0.8);
        planner = cartesian_planner;
    } else {
        auto joint_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl");
        joint_planner->setMaxVelocityScalingFactor(0.2);
        joint_planner->setMaxAccelerationScalingFactor(0.2);
        planner = joint_planner;
    }
    
    // Create task
    mtc::Task task;
    task.stages()->setName("MoveTo Task");
    
    task.setProperty("group", arm_group_name);
    
    // Set up IK frame
    geometry_msgs::msg::PoseStamped ik_frame_pose;
    ik_frame_pose.header.frame_id = "flange";
    ik_frame_pose.pose.orientation.w = 1.0;
    task.setProperty("ik_frame", ik_frame_pose);
    
    // Add current state
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));
    
    // Add movement stage based on target type
    if (target_type == "named_state") {
        std::string named_state = step["target"];
        // For named states, we'll handle this after loading the robot model
    } else if (target_type == "joints") {
        std::vector<double> joint_angles = step["target"].get<std::vector<double>>();
        task.add(makeMoveToJointStage("move_to_joints", joint_angles, planner, arm_group_name));
    } else if (target_type == "relative") {
        std::string direction = step["direction"];
        double distance = step["distance"].get<double>();
        task.add(makeMoveRelativeStage("move_relative", direction, distance, planner, arm_group_name));
    } else {
        // Default: pose from JSON
        std::string pose_key = step["target"];
        task.add(makeMoveToNamedStage("move_to_" + pose_key, pose_key, planner, arm_group_name, false));
    }
    
    // Initialize and execute task
    try {
        // FSM-style: No cancellation check before initialization
        
        // Load robot model
        task.loadRobotModel(node);
        
        // Debug: Print available groups and states
        auto robot_model = task.getRobotModel();
        if (robot_model) {
            RCLCPP_INFO(node->get_logger(), "Robot model loaded successfully");
            RCLCPP_INFO(node->get_logger(), "Available groups: %s", 
                       boost::algorithm::join(robot_model->getJointModelGroupNames(), ", ").c_str());
            
            if (target_type == "named_state") {
                std::string named_state = step["target"];
                auto group = robot_model->getJointModelGroup(arm_group_name);
                if (group) {
                    RCLCPP_INFO(node->get_logger(), "Group '%s' found", arm_group_name.c_str());
                    
                    // Get the named state from the robot model
                    std::map<std::string, double> joint_goal;
                    if (named_state == "moveit_home") {
                        joint_goal = {{"shoulder_pan_joint", 0.0}, {"shoulder_lift_joint", -1.57}, 
                                     {"elbow_joint", 0.0}, {"wrist_1_joint", -1.57}, 
                                     {"wrist_2_joint", 0.0}, {"wrist_3_joint", 0.0}};
                    } else if (named_state == "home") {
                        joint_goal = {{"shoulder_pan_joint", 0.0}, {"shoulder_lift_joint", -1.57}, 
                                     {"elbow_joint", 0.0}, {"wrist_1_joint", -1.57}, 
                                     {"wrist_2_joint", 0.0}, {"wrist_3_joint", 0.0}};
                    } else {
                        RCLCPP_ERROR(node->get_logger(), "Unknown named state: %s", named_state.c_str());
                        return false;
                    }
                    
                    std::vector<double> joint_angles;
                    for (const auto& joint_name : getJointNames()) {
                        joint_angles.push_back(joint_goal[joint_name]);
                    }
                    task.add(makeMoveToJointStage("move_to_" + named_state, joint_angles, planner, arm_group_name));
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Group '%s' not found in robot model", arm_group_name.c_str());
                    return false;
                }
            }
        }
        
        // FSM-style: No cancellation check before planning
        
        task.init();
    } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Stage initialization failed: " << e);
        return false;
    }
    
    // FSM-style: No cancellation check before planning
    
    if (!task.plan(5)) {
        RCLCPP_ERROR(node->get_logger(), "Task planning failed");
        return false;
    }
    
    if (task.solutions().empty()) {
        RCLCPP_ERROR(node->get_logger(), "No solutions found to execute");
        return false;
    }
    
    // FSM-style: No cancellation check before execution
    
    auto solution = task.solutions().front();
    
    // Execute with FSM-style behavior (complete movement, then check cancellation)
    RCLCPP_INFO(node->get_logger(), "Starting execution with FSM-style behavior...");
    
    // COMMENTED OUT: Multi-thread cancellation monitoring
    // This will now behave like FSM: complete movement, then check for cancellation
    /*
    // Start execution in a separate thread to allow monitoring
    std::atomic<bool> execution_complete{false};
    std::atomic<bool> execution_success{false};
    std::exception_ptr execution_exception{nullptr};
    
    std::thread execution_thread([&]() {
        try {
            auto result = task.execute(*solution);
            execution_success = (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
            execution_complete = true;
        } catch (...) {
            execution_exception = std::current_exception();
            execution_complete = true;
        }
    });
    
    // Monitor execution with cancellation checks (FSM-style loop)
    while (!execution_complete) {
        // Check for cancellation every 100ms (like FSM approach)
        if (should_cancel && should_cancel()) {
            RCLCPP_WARN(node->get_logger(), "MoveTo task cancelled during execution - stopping gracefully...");
            
            // Graceful software cancellation - no hardware emergency stop needed
            try {
                RCLCPP_INFO(node->get_logger(), "Cancelling MTC task execution gracefully...");
                
                // The MTC task will handle the cancellation gracefully
                // No need for hardware emergency stop - this is software cancellation
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node->get_logger(), "Exception during task cancellation: %s", e.what());
            }
            
            // Detach the execution thread and return immediately
            execution_thread.detach();
            
            RCLCPP_WARN(node->get_logger(), "MoveTo task cancelled - execution stopped gracefully");
            return false;
        }
        
        // Sleep for 100ms before next check (like FSM approach)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Wait for execution thread to complete
    if (execution_thread.joinable()) {
        execution_thread.join();
    }
    
    // Handle any exceptions
    if (execution_exception) {
        try {
            std::rethrow_exception(execution_exception);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "MoveTo execution exception: %s", e.what());
            return false;
        }
    }
    
    // Check final result
    if (should_cancel && should_cancel()) {
        RCLCPP_WARN(node->get_logger(), "MoveTo task cancelled after execution");
        return false;
    }
    
    if (execution_success) {
        RCLCPP_INFO(node->get_logger(), "MoveTo task completed successfully");
    } else {
        RCLCPP_ERROR(node->get_logger(), "MoveTo task failed");
    }
    
    return execution_success;
    */
    
    // FSM-STYLE EXECUTION: Blocking call, then check cancellation
    RCLCPP_INFO(node->get_logger(), "Executing MTC task (blocking call)...");
    auto result = task.execute(*solution);
    
    // Check for cancellation AFTER execution completes (FSM-style)
    if (should_cancel && should_cancel()) {
        RCLCPP_WARN(node->get_logger(), "MoveTo task cancelled after execution (FSM-style)");
        return false;
    }
    
    bool execution_success = (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    
    if (execution_success) {
        RCLCPP_INFO(node->get_logger(), "MoveTo task completed successfully");
    } else {
        RCLCPP_ERROR(node->get_logger(), "MoveTo task failed");
    }
    
    return execution_success;
}
