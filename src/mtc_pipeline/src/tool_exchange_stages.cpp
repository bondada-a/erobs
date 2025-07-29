#include "mtc_pipeline/tool_exchange_stages.hpp"
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <map>

namespace mtc = moveit::task_constructor;

ToolExchangeStages::ToolExchangeStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config)
    : node_(node), config_(config) {}

bool ToolExchangeStages::run(const nlohmann::json& step, const nlohmann::json& poses, rclcpp::Node::SharedPtr node)
{
    // Get operation, dock_number, and which approach pose to use from JSON step
    std::string operation = step.value("operation", "load");  // "load" or "dock"
    int dock_number = step.value("dock_number", 3);           // Default: 3 (center)
    std::string approach_pose = step["poses"][0];             // e.g. "load_approach" or "dock_approach"

    // For modularity, use the passed-in poses JSON, not config_
    nlohmann::json temp_config = config_;
    temp_config["poses"] = poses;

    // Calculate dock offset
    constexpr double DOCK_SPACING = 0.1524; // meters
    double dock_offset_y = DOCK_SPACING * static_cast<double>(3 - dock_number);

    // Planners setup
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
    cartesian_planner->setMinFraction(0.5);
    // cartesian_planner->setPrecision(1e-4);  

    std::string arm_group_name = "ur_arm";
    std::string hand_group_name = "hande_gripper";
    std::string hand_frame = "flange";

    mtc::Task task;
    if (operation == "load")
        task.stages()->setName("Load Tool Task");
    else if (operation == "dock")
        task.stages()->setName("Dock Tool Task");
    else
        task.stages()->setName("Tool Exchange Task");

    task.loadRobotModel(node);

    // -------- CRITICAL: Set group/eef/ik_frame properties before adding stages!
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);
    // -----------------------------------------------------------

    // Helper to add a named MoveTo (copied from your code)
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

    // Helper to make a dock shift
    auto makeDockShiftStage = [&](double offset, const std::string& name) -> std::unique_ptr<mtc::stages::MoveRelative> {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(name, cartesian_planner);
        stage->properties().set("marker_ns", "dock_shift");
        stage->properties().set("link", hand_frame);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(std::abs(offset), std::abs(offset));
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame;
        vec.vector.y = (offset >= 0.0) ? 1.0 : -1.0;   // direction
        stage->setDirection(vec);
        return stage;
    };

    // 1. Current State
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));

    if (operation == "load") {
        // Move to load_approach from JSON
        addNamedMoveStage(task, "move to load approach", approach_pose, sampling_planner);
        if (std::abs(dock_offset_y) > 1e-4)
            task.add(makeDockShiftStage(dock_offset_y, "shift to dock"));

        // Attach to dock
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("attach_tool", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.1, 0.1);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.x = 1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }

        // Detach from holder
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("detach_holder", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.15, 0.15);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }

        // Move up
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("move_up", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.2, 0.2);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.x = -1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }
    } else if (operation == "dock") {
        // Move to dock_approach from JSON
        addNamedMoveStage(task, "move to dock approach", approach_pose, sampling_planner);
        if (std::abs(dock_offset_y) > 1e-4)
            task.add(makeDockShiftStage(dock_offset_y, "shift to dock"));

        // Align with holder
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("align_holder", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.2, 0.2);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.x = 1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }

        // Move to holder
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("detach_tool", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.15, 0.15);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }

        // Move up
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("dock connect", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.1, 0.1);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.x = -1.0;
            stage->setDirection(vec);
            task.add(std::move(stage));
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Unknown operation: %s", operation.c_str());
        return false;
    }

    // --- Planning and execution ---
    try {
        task.init();
    } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR(node->get_logger(), "Stage initialization failed: %s", e.what());
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

    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Task execution failed");
        return false;
    }

    return true;
}
