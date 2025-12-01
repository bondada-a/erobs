// Vision-guided pick and place using ArUco tag detection.

#pragma once

#include "mtc_pipeline/base_stages.hpp"
#include "mtc_pipeline/vision_stages.hpp"
#include "mtc_pipeline/action/vision_pick_place_action.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>

class VisionPickPlaceStages : public BaseStages {
public:
    /// @brief Construct VisionPickPlace stages with ROS 2 node
    VisionPickPlaceStages(const rclcpp::Node::SharedPtr& node);

    /// @brief Execute vision-guided pick-and-place from goal specification
    bool run(const mtc_pipeline::action::VisionPickPlaceAction::Goal& goal);

private:
    std::shared_ptr<VisionStages> vision_;

    /// @brief Compute grasp pose from detected tag pose and offset
    geometry_msgs::msg::PoseStamped compute_grasp_pose(
        const geometry_msgs::msg::PoseStamped& tag_pose,
        const nlohmann::json& offset);

    /// @brief Compute approach/retreat pose by applying z-axis offset
    geometry_msgs::msg::PoseStamped compute_offset_pose(
        const geometry_msgs::msg::PoseStamped& base_pose,
        double z_offset);

    /// @brief Create gripper open/close stage for specified gripper type
    std::unique_ptr<mtc::Stage> make_gripper_stage(
        const std::string& label,
        const mtc::solvers::PlannerInterfacePtr& planner,
        bool open,
        const std::string& gripper_type);

    /// @brief Create Cartesian move stage to target pose
    std::unique_ptr<mtc::Stage> make_cartesian_move_stage(
        const std::string& label,
        const geometry_msgs::msg::PoseStamped& target,
        const mtc::solvers::PlannerInterfacePtr& planner,
        bool apply_wrist_constraint = true);
};
