#include "mtc_pipeline/tool_exchange_stages.hpp"
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <future>

namespace mtc = moveit::task_constructor;

ToolExchangeStages::ToolExchangeStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
    : node_(node), config_(config) {}

// Execute tool exchange operation (load or dock)
bool ToolExchangeStages::run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node)
{
    std::string operation = step.value("operation", "load");
    int dock_number = step.value("dock_number", 3);
    std::string approach_pose = step["poses"][0];

    // Update config with poses
    config_["poses"] = poses;

    constexpr double DOCK_SPACING = 0.1524;
    double dock_offset_y = DOCK_SPACING * static_cast<double>(3 - dock_number);

    // Helper function to configure planners
    auto configurePlanner = [](auto planner, double vel_scale = 0.2, double acc_scale = 0.2) {
        planner->setMaxVelocityScalingFactor(vel_scale);
        planner->setMaxAccelerationScalingFactor(acc_scale);
        return planner;
    };

    // Setup planners
    auto sampling_planner = configurePlanner(std::make_shared<mtc::solvers::PipelinePlanner>(node));
    auto interpolation_planner = configurePlanner(std::make_shared<mtc::solvers::JointInterpolationPlanner>());
    auto cartesian_planner = configurePlanner(std::make_shared<mtc::solvers::CartesianPath>());
    cartesian_planner->setStepSize(0.001);
    cartesian_planner->setMinFraction(0.8);

    std::string arm_group_name = "ur_arm";
    std::string hand_frame = "flange";

    mtc::Task task;
    task.stages()->setName(operation == "load" ? "Load Tool Task" : 
                          operation == "dock" ? "Dock Tool Task" : "Tool Exchange Task");

    task.setProperty("group", arm_group_name);
    geometry_msgs::msg::PoseStamped ik_frame_pose;
    ik_frame_pose.header.frame_id = hand_frame;
    ik_frame_pose.pose.orientation.w = 1.0;
    task.setProperty("ik_frame", ik_frame_pose);

    // Helper function to create move stages
    auto addNamedMoveStage = [&](const std::string& label, const std::string& pose_key) {
        auto& angles_deg = poses[pose_key];
        if (!angles_deg.is_array() || angles_deg.size() != 6)
            throw std::runtime_error(pose_key + " must be an array of 6 numbers");
        
        static const std::vector<std::string> JOINT_NAMES = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };
        
        std::map<std::string, double> joint_goal;
        for (size_t i = 0; i < 6; ++i)
            joint_goal[JOINT_NAMES[i]] = angles_deg[i].get<double>() * M_PI / 180.0;
        
        auto stage = std::make_unique<mtc::stages::MoveTo>(label, sampling_planner);
        stage->setGroup(arm_group_name);
        stage->setGoal(joint_goal);
        task.add(std::move(stage));
    };

    // Helper function to create relative move stages
    auto addRelativeMoveStage = [&](const std::string& name, double distance, 
                                   double x, double y, double z) {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(name, cartesian_planner);
        stage->properties().set("marker_ns", "approach_object");
        stage->properties().set("link", hand_frame);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
        stage->setMinMaxDistance(std::abs(distance), std::abs(distance));
        
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame;
        vec.vector.x = x;
        vec.vector.y = y;
        vec.vector.z = z;
        stage->setDirection(vec);
        task.add(std::move(stage));
    };

    // Helper function relative to dock 3 (y offset)
    auto addDockShiftStage = [&](double offset) {
        if (std::abs(offset) > 1e-4) {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("shift to dock", cartesian_planner);
            stage->properties().set("marker_ns", "dock_shift");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
            stage->setMinMaxDistance(std::abs(offset), std::abs(offset));
            
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.y = (offset >= 0.0) ? 1.0 : -1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }
    };

    task.add(std::make_unique<mtc::stages::CurrentState>("current"));

    // Load operation: attach tool to robot
    if (operation == "load") {
        addNamedMoveStage("move to load approach", approach_pose);
        addDockShiftStage(dock_offset_y);
        addRelativeMoveStage("attach_tool", 0.1, 1.0, 0.0, 0.0);      
        addRelativeMoveStage("detach_holder", 0.15, 0.0, 0.0, -1.0);  
        addRelativeMoveStage("move_up", 0.2, -1.0, 0.0, 0.0);         
        } 
    // Dock operation: remove tool from robot
    else if (operation == "dock") {
        addNamedMoveStage("move to dock approach", approach_pose);
        addDockShiftStage(dock_offset_y);
        addRelativeMoveStage("align_holder", 0.035, 1.0, 0.0, 0.0);   
        addRelativeMoveStage("detach_tool", 0.02, 0.0, 0.0, 1.0);     
        addRelativeMoveStage("dock connect", 0.04, -1.0, 0.0, 0.0);   
    } else {
        RCLCPP_ERROR(node->get_logger(), "Unknown operation: %s", operation.c_str());
        return false;
    }

    // Initialize and execute task
    try {
        task.loadRobotModel(node);
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
        RCLCPP_INFO(node->get_logger(), "Tool exchange completed successfully");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Tool exchange failed with code: %d", result.val);
    }

    return (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}
