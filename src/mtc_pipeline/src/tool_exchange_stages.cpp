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

bool ToolExchangeStages::run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node)
{
    std::string operation = step.value("operation", "load");
    int dock_number = step.value("dock_number", 3);
    std::string approach_pose = step["poses"][0];

    nlohmann::json temp_config = config_;
    temp_config["poses"] = poses;

    constexpr double DOCK_SPACING = 0.1524;
    double dock_offset_y = DOCK_SPACING * static_cast<double>(3 - dock_number);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
    sampling_planner->setMaxVelocityScalingFactor(0.2);
    sampling_planner->setMaxAccelerationScalingFactor(0.2);

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    interpolation_planner->setMaxVelocityScalingFactor(0.2);
    interpolation_planner->setMaxAccelerationScalingFactor(0.2);

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.2);
    cartesian_planner->setMaxAccelerationScalingFactor(0.2);
    cartesian_planner->setStepSize(0.001);
    cartesian_planner->setMinFraction(0.9);

    std::string arm_group_name = "ur_arm";
    std::string hand_frame = "flange";

    mtc::Task task;
    if (operation == "load")
        task.stages()->setName("Load Tool Task");
    else if (operation == "dock")
        task.stages()->setName("Dock Tool Task");
    else
        task.stages()->setName("Tool Exchange Task");

    task.loadRobotModel(node);

    task.setProperty("group", arm_group_name);
    geometry_msgs::msg::PoseStamped ik_frame_pose;
    ik_frame_pose.header.frame_id = hand_frame;
    ik_frame_pose.pose.orientation.w = 1.0;
    task.setProperty("ik_frame", ik_frame_pose);

    auto addNamedMoveStage = [&](mtc::Task& task, const std::string& label, const std::string& pose_key,
                                const mtc::solvers::PlannerInterfacePtr& planner) {
        auto& angles_deg = poses[pose_key];
        if (!angles_deg.is_array() || angles_deg.size() != 6)
            throw std::runtime_error(pose_key + " must be an array of 6 numbers");
        const std::vector<std::string> joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint",      "wrist_2_joint",      "wrist_3_joint"
        };
        std::map<std::string, double> joint_goal;
        for (size_t i = 0; i < 6; ++i)
            joint_goal[joint_names[i]] = angles_deg[i].get<double>() * M_PI / 180.0;
        auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
        stage->setGroup(arm_group_name);
        stage->setGoal(joint_goal);
        task.add(std::move(stage));
    };

    auto makeDockShiftStage = [&](double offset, const std::string& name) -> std::unique_ptr<mtc::stages::MoveRelative> {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(name, cartesian_planner);
        stage->properties().set("marker_ns", "dock_shift");
        stage->properties().set("link", hand_frame);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
        stage->setMinMaxDistance(std::abs(offset), std::abs(offset));
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame;  // Move relative to flange frame orientation
        vec.vector.y = (offset >= 0.0) ? 1.0 : -1.0;
        stage->setDirection(vec);
        return stage;
    };

    task.add(std::make_unique<mtc::stages::CurrentState>("current"));

    if (operation == "load") {
        addNamedMoveStage(task, "move to load approach", approach_pose, sampling_planner);
        if (std::abs(dock_offset_y) > 1e-4)
            task.add(makeDockShiftStage(dock_offset_y, "shift to dock"));
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("attach_tool", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
            stage->setMinMaxDistance(0.1, 0.1);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;  // Move relative to flange frame orientation
            vec.vector.x = 1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("detach_holder", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
            stage->setMinMaxDistance(0.15, 0.15);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;  // Move relative to flange frame orientation
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("move_up", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
            stage->setMinMaxDistance(0.2, 0.2);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;  // Move relative to flange frame orientation
            vec.vector.x = -1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }
    } else if (operation == "dock") {
        addNamedMoveStage(task, "move to dock approach", approach_pose, sampling_planner);
        if (std::abs(dock_offset_y) > 1e-4)
            task.add(makeDockShiftStage(dock_offset_y, "shift to dock"));
        
        // Debug: Print current robot pose before Cartesian movement
        RCLCPP_INFO(node->get_logger(), "=== DEBUG: Before align_holder stage ===");
        auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
        if (psm && psm->getPlanningScene()) {
            psm->requestPlanningSceneState();
            const auto& state = psm->getPlanningScene()->getCurrentState();
            auto group = state.getJointModelGroup(arm_group_name);
            if (group) {
                std::vector<double> joint_values;
                state.copyJointGroupPositions(group, joint_values);
                const std::vector<std::string>& joint_names = group->getVariableNames();
                RCLCPP_INFO(node->get_logger(), "Current joint positions:");
                for (size_t i = 0; i < joint_names.size(); ++i)
                    RCLCPP_INFO(node->get_logger(), "  %s: %.4f", joint_names[i].c_str(), joint_values[i]);
                
                // Get current end-effector pose
                const auto& transform = state.getGlobalLinkTransform(hand_frame);
                RCLCPP_INFO(node->get_logger(), "Current %s pose: x=%.3f, y=%.3f, z=%.3f", 
                           hand_frame.c_str(), transform.translation().x(), transform.translation().y(), transform.translation().z());
            }
        }
        
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("align_holder", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
            stage->setMinMaxDistance(0.02, 0.05);  // Reduced to smaller, more achievable distances
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;  // Move relative to flange frame orientation
            vec.vector.x = 1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("detach_tool", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
            stage->setMinMaxDistance(0.01, 0.03);  // Reduced to very small distances
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;  // Move relative to flange frame orientation
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("dock connect", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
            stage->setMinMaxDistance(0.03, 0.05);  // Reduced to smaller distances
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;  // Move relative to flange frame orientation
            vec.vector.x = -1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Unknown operation: %s", operation.c_str());
        return false;
    }

    // --- Print the current (LIVE) robot state for debugging ---
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
                for (size_t i = 0; i < joint_names.size(); ++i)
                    RCLCPP_INFO(node->get_logger(), "  %s: %.4f", joint_names[i].c_str(), joint_values[i]);
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
    for (int i = 180; i > 0; --i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (i % 30 == 0) {  // Print countdown every 30 seconds
            RCLCPP_INFO(node->get_logger(), "Execution will begin in %d seconds...", i);
        }
    }
    
    RCLCPP_INFO(node->get_logger(), "Starting execution now...");

    // === Execute the planned solution (all stages) ===
    auto solution = task.solutions().front();
    auto result = task.execute(*solution);

    RCLCPP_INFO(node->get_logger(), "Task execution result code: %d", result.val);
    if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        RCLCPP_INFO(node->get_logger(), "MTC task reports execution success!");
    else
        RCLCPP_ERROR(node->get_logger(), "MTC task reports execution failure!");

    // Wait for robot to reach stable state after execution instead of hardcoded sleep
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
            RCLCPP_INFO(node->get_logger(), "Robot reached stable state after tool exchange.");
        } else {
            RCLCPP_WARN(node->get_logger(), "Robot may not have reached stable state, continuing...");
        }
    }

    return (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}
