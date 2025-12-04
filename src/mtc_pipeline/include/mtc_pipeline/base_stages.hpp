// Base class for MTC stage implementations.
// Provides task templates, planners, and movement utilities for derived classes.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mtc = moveit::task_constructor;

class BaseStages {
public:
    /// @brief Construct base stages with ROS 2 node
    BaseStages(const rclcpp::Node::SharedPtr& node);
    virtual ~BaseStages() = default;

protected:
    /// @brief Get shared pointer to ROS 2 node
    rclcpp::Node::SharedPtr node() const;

    /// @brief Create MTC task with configured planners and group settings
    mtc::Task create_task_template(const std::string& name,
                                    const std::string& arm_group = "",
                                    const std::string& ik_frame = "") const;

    /// @brief Plan and execute MTC task, returning success status
    bool load_plan_execute(mtc::Task& task) const;

    /// @brief Convert degree array to joint name-value map
    std::map<std::string, double> joints_from_degrees(const std::vector<double>& angles_deg) const;

    /// @brief Get default UR joint names
    static const std::vector<std::string>& default_joint_names();

    /// @brief Get default arm planning group name
    static const std::string& default_arm_group_name();

    /// @brief Get default IK frame name
    static const std::string& default_ik_frame();

    /// @brief Convert degrees to radians
    static constexpr double deg_to_rad(double deg) { return deg * M_PI / 180.0; }

    /// @brief Create OMPL pipeline planner with 20% velocity scaling
    mtc::solvers::PlannerInterfacePtr make_pipeline_planner() const;

    /// @brief Create Cartesian path planner with 20% velocity scaling
    mtc::solvers::PlannerInterfacePtr make_cartesian_planner() const;

    /// @brief Create joint interpolation planner with 20% velocity scaling
    mtc::solvers::PlannerInterfacePtr make_joint_interpolation_planner() const;

    /// @brief Create MoveRelative stage from direction string
    std::unique_ptr<mtc::Stage> create_relative_move_stage(
        const std::string& label,
        const std::string& direction,
        double distance,
        const mtc::solvers::PlannerInterfacePtr& planner) const;

    /// @brief Create path constraint to keep wrist_3 level during manipulation
    moveit_msgs::msg::Constraints create_wrist3_level_constraint() const;

private:
    rclcpp::Node::SharedPtr node_;
};
