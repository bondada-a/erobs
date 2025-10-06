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
  struct PipelinePlannerDefaults {
    static const double vel_scale;
    static const double acc_scale;
    static const char* const pipeline_id;
  };

  struct CartesianPlannerDefaults {
    static const double vel_scale;
    static const double acc_scale;
    static const double step;
    static const double min_fraction;
  };

  struct JointInterpolationPlannerDefaults {
    static const double vel_scale;
    static const double acc_scale;
  };

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

  mtc::solvers::PlannerInterfacePtr make_pipeline_planner(
    const std::string& pipeline_id = PipelinePlannerDefaults::pipeline_id,
    double vel_scale = PipelinePlannerDefaults::vel_scale,
    double acc_scale = PipelinePlannerDefaults::acc_scale) const;

  mtc::solvers::PlannerInterfacePtr make_cartesian_planner(
    double vel_scale = CartesianPlannerDefaults::vel_scale,
    double acc_scale = CartesianPlannerDefaults::acc_scale,
    double step = CartesianPlannerDefaults::step,
    double min_fraction = CartesianPlannerDefaults::min_fraction) const;

  mtc::solvers::PlannerInterfacePtr make_joint_interpolation_planner(
    double vel_scale = JointInterpolationPlannerDefaults::vel_scale,
    double acc_scale = JointInterpolationPlannerDefaults::acc_scale) const;

  // Movement stage creation helper
  std::unique_ptr<mtc::Stage> create_relative_move_stage(
    const std::string& label,
    const std::string& direction,
    double distance,
    const mtc::solvers::PlannerInterfacePtr& planner) const;

private:
  rclcpp::Node::SharedPtr node_;
};
