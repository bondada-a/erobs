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

// Shared utilities for modular MTC stage implementations
class BaseStages {
public:
    BaseStages(const rclcpp::Node::SharedPtr& node);
    virtual ~BaseStages() = default;

protected:
    rclcpp::Node::SharedPtr node() const;

    mtc::Task create_task_template(const std::string& name,
                                    const std::string& arm_group = "",
                                    const std::string& ik_frame = "") const;

    bool load_plan_execute(mtc::Task& task) const;

    std::map<std::string, double> joints_from_degrees(const std::vector<double>& angles_deg) const;
    static const std::vector<std::string>& default_joint_names();
    static const std::string& default_arm_group_name();
    static const std::string& default_ik_frame();
    static constexpr double deg_to_rad(double degrees) { return degrees * M_PI / 180.0; }

    mtc::solvers::PlannerInterfacePtr make_pipeline_planner() const;
    mtc::solvers::PlannerInterfacePtr make_cartesian_planner() const;
    mtc::solvers::PlannerInterfacePtr make_joint_interpolation_planner() const;

    // Movement stage creation helper
    std::unique_ptr<mtc::Stage> create_relative_move_stage(
        const std::string& label,
        const std::string& direction,
        double distance,
        const mtc::solvers::PlannerInterfacePtr& planner) const;

private:
    rclcpp::Node::SharedPtr node_;
};
