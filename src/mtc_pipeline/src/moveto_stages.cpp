#include "mtc_pipeline/moveto_stages.hpp"
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <future>

namespace mtc = moveit::task_constructor;

MoveToStages::MoveToStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
    : node_(node), config_(config) {}

std::vector<std::string> MoveToStages::getJointNames() {
    return {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
}

std::map<std::string, double> MoveToStages::convertDegreesToRadians(const std::vector<double>& angles_deg) {
    const std::vector<std::string> joint_names = getJointNames();
    std::map<std::string, double> joint_goal;
    
    for (size_t i = 0; i < std::min(angles_deg.size(), joint_names.size()); ++i) {
        joint_goal[joint_names[i]] = angles_deg[i] * M_PI / 180.0;
    }
    
    return joint_goal;
}

std::unique_ptr<mtc::Stage> MoveToStages::makeMoveToNamedStage(
    const std::string& label,
    const std::string& pose_key,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name,
    bool is_named_state) {
    
    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->setGroup(arm_group_name);
    
    // Configure IK frame for Cartesian planning
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
    
    // Explicitly set the IK frame for Cartesian planning
    stage->setIKFrame("flange");
    
    if (is_named_state) {
        // Use SRDF named state (like "home", "ready", etc.)
        stage->setGoal(pose_key);
    } else {
        // Use joint angles from JSON poses
        auto& angles_deg = config_["poses"][pose_key];
        if (!angles_deg.is_array() || angles_deg.size() != 6) {
            throw std::runtime_error(pose_key + " must be an array of 6 numbers");
        }
        
        std::vector<double> angles_vec;
        for (const auto& angle : angles_deg) {
            angles_vec.push_back(angle.get<double>());
        }
        
        // For Cartesian planning, we need to convert joint angles to a pose
        // For now, let's use joint angles and let MTC handle the conversion
        auto joint_goal = convertDegreesToRadians(angles_vec);
        stage->setGoal(joint_goal);
    }
    
    return stage;
}

std::unique_ptr<mtc::Stage> MoveToStages::makeMoveToJointStage(
    const std::string& label,
    const std::vector<double>& joint_angles,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name) {
    
    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->setGroup(arm_group_name);
    
    // Configure IK frame for Cartesian planning
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
    
    // Explicitly set the IK frame for Cartesian planning
    stage->setIKFrame("flange");
    
    auto joint_goal = convertDegreesToRadians(joint_angles);
    stage->setGoal(joint_goal);
    
    return stage;
}

std::unique_ptr<mtc::Stage> MoveToStages::makeMoveToPoseStage(
    const std::string& label,
    const geometry_msgs::msg::PoseStamped& pose,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& arm_group_name) {
    
    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->setGroup(arm_group_name);
    
    // Configure IK frame for Cartesian planning
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
    
    // Explicitly set the IK frame for Cartesian planning
    stage->setIKFrame("flange");
    
    stage->setGoal(pose);
    
    return stage;
}

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

bool MoveToStages::run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node) {
    // Parse the step configuration
    std::string target_type = step.value("target_type", "pose"); // "pose", "joints", "named_state", or "relative"
    std::string planning_type = step.value("planning_type", "joint"); // "joint" or "cartesian"
    std::string arm_group_name = step.value("arm_group", "ur_arm");
    
    nlohmann::json temp_config = config_;
    temp_config["poses"] = poses;
    
    // Create planners based on planning type
    mtc::solvers::PlannerInterfacePtr planner;
    
    if (planning_type == "cartesian") {
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.2);
        cartesian_planner->setMaxAccelerationScalingFactor(0.2);
        cartesian_planner->setStepSize(0.001);
        cartesian_planner->setMinFraction(0.8);
        planner = cartesian_planner;
    } else {
        // Default to joint space planning
        auto joint_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
        joint_planner->setMaxVelocityScalingFactor(0.2);
        joint_planner->setMaxAccelerationScalingFactor(0.2);
        planner = joint_planner;
    }
    
    // Create the task
    mtc::Task task;
    task.stages()->setName("MoveTo Task");
    task.loadRobotModel(node);
    
    task.setProperty("group", arm_group_name);
    
    // Set up IK frame - use flange for consistency with other modules
    geometry_msgs::msg::PoseStamped ik_frame_pose;
    ik_frame_pose.header.frame_id = "flange";
    ik_frame_pose.pose.orientation.w = 1.0;
    task.setProperty("ik_frame", ik_frame_pose);
    
    // For Cartesian planning, ensure the IK frame is properly configured
    if (planning_type == "cartesian") {
        RCLCPP_INFO(node->get_logger(), "Using Cartesian planning with IK frame: flange");
    }
    
    // Add current state
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));
    
    // Add the movement stage based on target type
    if (target_type == "named_state") {
        std::string named_state = step["target"];
        task.add(makeMoveToNamedStage("move_to_" + named_state, named_state, planner, arm_group_name, true));
    } else if (target_type == "joints") {
        std::vector<double> joint_angles = step["target"].get<std::vector<double>>();
        task.add(makeMoveToJointStage("move_to_joints", joint_angles, planner, arm_group_name));
    } else if (target_type == "relative") {
        // Parse relative movement parameters
        std::string direction = step["direction"];
        double distance = step["distance"].get<double>();
        task.add(makeMoveRelativeStage("move_relative", direction, distance, planner, arm_group_name));
    } else {
        // Default: pose from JSON
        std::string pose_key = step["target"];
        
        // For Cartesian planning, we need to convert joint angles to a pose
        if (planning_type == "cartesian") {
            // Get the joint angles from the JSON
            auto& angles_deg = poses[pose_key];
            if (!angles_deg.is_array() || angles_deg.size() != 6) {
                throw std::runtime_error(pose_key + " must be an array of 6 numbers");
            }
            
            // Convert to radians
            std::vector<double> angles_vec;
            for (const auto& angle : angles_deg) {
                angles_vec.push_back(angle.get<double>());
            }
            auto joint_angles_rad = convertDegreesToRadians(angles_vec);
            
            // Create a pose from the joint angles using forward kinematics
            auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
            if (psm && psm->getPlanningScene()) {
                psm->requestPlanningSceneState();
                const auto& state = psm->getPlanningScene()->getCurrentState();
                auto group = state.getJointModelGroup(arm_group_name);
                if (group) {
                    // Set the joint angles
                    moveit::core::RobotState target_state = state;
                    
                    // Convert map to vector for joint positions
                    std::vector<double> joint_angles_vec;
                    const auto& joint_names = group->getVariableNames();
                    for (const auto& joint_name : joint_names) {
                        auto it = joint_angles_rad.find(joint_name);
                        if (it != joint_angles_rad.end()) {
                            joint_angles_vec.push_back(it->second);
                        } else {
                            joint_angles_vec.push_back(0.0); // Default value
                        }
                    }
                    target_state.setJointGroupPositions(group, joint_angles_vec);
                    
                    // Get the pose of the flange
                    const auto& link_model = target_state.getLinkModel("flange");
                    if (link_model) {
                        const auto& transform = target_state.getGlobalLinkTransform("flange");
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header.frame_id = "base_link";
                        pose.pose.position.x = transform.translation().x();
                        pose.pose.position.y = transform.translation().y();
                        pose.pose.position.z = transform.translation().z();
                        
                        // Convert Eigen quaternion to geometry_msgs quaternion
                        Eigen::Quaterniond quat(transform.rotation());
                        pose.pose.orientation.w = quat.w();
                        pose.pose.orientation.x = quat.x();
                        pose.pose.orientation.y = quat.y();
                        pose.pose.orientation.z = quat.z();
                        
                        task.add(makeMoveToPoseStage("move_to_" + pose_key, pose, planner, arm_group_name));
                    } else {
                        // Fallback to joint planning
                        task.add(makeMoveToNamedStage("move_to_" + pose_key, pose_key, planner, arm_group_name, false));
                    }
                } else {
                    // Fallback to joint planning
                    task.add(makeMoveToNamedStage("move_to_" + pose_key, pose_key, planner, arm_group_name, false));
                }
            } else {
                // Fallback to joint planning
                task.add(makeMoveToNamedStage("move_to_" + pose_key, pose_key, planner, arm_group_name, false));
            }
        } else {
            // Use joint planning
            task.add(makeMoveToNamedStage("move_to_" + pose_key, pose_key, planner, arm_group_name, false));
        }
    }
    
    // Print current robot state for debugging
    {
        auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
        if (psm && psm->getPlanningScene()) {
            psm->requestPlanningSceneState();
            const auto& state = psm->getPlanningScene()->getCurrentState();
            auto group = state.getJointModelGroup(arm_group_name);
            if (!group) {
                RCLCPP_WARN(node->get_logger(), "Joint model group %s not found in live scene!", arm_group_name.c_str());
            } else {
                std::vector<double> joint_values;
                state.copyJointGroupPositions(group, joint_values);
                const std::vector<std::string>& joint_names = group->getVariableNames();
                RCLCPP_INFO(node->get_logger(), "[LIVE] Start state for group '%s':", arm_group_name.c_str());
                for (size_t i = 0; i < joint_names.size(); ++i) {
                    RCLCPP_INFO(node->get_logger(), "  %s: %.4f", joint_names[i].c_str(), joint_values[i]);
                }
            }
        } else {
            RCLCPP_WARN(node->get_logger(), "Could not get PlanningSceneMonitor state!");
        }
    }
    
    try {
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
    
    // === PAUSE FOR RViz VERIFICATION ===
    RCLCPP_INFO(node->get_logger(), "=== PLANNING SUCCESSFUL ===");
    RCLCPP_INFO(node->get_logger(), "Plan created with %zu solutions", task.solutions().size());
    RCLCPP_INFO(node->get_logger(), "PAUSING FOR 3 MINUTES - Please verify the plan in RViz!");
    RCLCPP_INFO(node->get_logger(), "You can see the planned trajectory in RViz before execution begins.");
    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to abort, or wait for automatic execution...");
    
    // Wait for 3 minutes (180 seconds)
    for (int i = 2; i > 0; --i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (i % 1 == 0) {  // Print countdown every second
            RCLCPP_INFO(node->get_logger(), "Execution will begin in %d seconds...", i);
        }
    }
    
    RCLCPP_INFO(node->get_logger(), "Starting execution now...");
    
    // === Execute the planned solution ===
    auto solution = task.solutions().front();
    auto result = task.execute(*solution);
    
    RCLCPP_INFO(node->get_logger(), "Task execution result code: %d", result.val);
    if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "MoveTo task reports execution success!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "MoveTo task reports execution failure!");
    }
    
    // Wait for robot to reach stable state after execution
    if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Waiting for robot to reach stable state...");
        
        // Monitor joint states to detect when robot has settled
        std::promise<bool> stability_promise;
        auto stability_future = stability_promise.get_future();
        
        auto joint_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            [&stability_promise](const sensor_msgs::msg::JointState::SharedPtr msg) {
                bool stable = true;
                const double velocity_threshold = 0.01;
                
                for (const auto& velocity : msg->velocity) {
                    if (std::abs(velocity) > velocity_threshold) {
                        stable = false;
                        break;
                    }
                }
                
                if (stable) {
                    stability_promise.set_value(true);
                }
            });
        
        // Wait for stability with timeout
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(10);
        bool stable = false;
        
        while (std::chrono::steady_clock::now() - start_time < timeout) {
            rclcpp::spin_some(node);
            if (stability_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                stable = true;
                break;
            }
        }
        
        if (stable) {
            RCLCPP_INFO(node->get_logger(), "Robot reached stable state after movement.");
        } else {
            RCLCPP_WARN(node->get_logger(), "Robot may not have reached stable state, continuing...");
        }
    }
    
    return (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}
