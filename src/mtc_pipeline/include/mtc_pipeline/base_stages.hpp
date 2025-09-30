#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <nlohmann/json.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mtc = moveit::task_constructor;

// Shared utilities for modular MTC stage implementations
class BaseStages {
public:
  using ShouldCancelFn = std::function<bool()>;

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

  BaseStages(const rclcpp::Node::SharedPtr& node, const nlohmann::json& config);
  virtual ~BaseStages() = default;

protected:
  rclcpp::Node::SharedPtr node() const;

  nlohmann::json& config();
  const nlohmann::json& config() const;

  void refreshPoses(const nlohmann::json& poses);

  mtc::Task createTaskTemplate(const std::string& name,
                               const std::string& arm_group,
                               const std::string& ik_frame = "flange",
                               bool add_current_state = true) const;

  bool loadPlanExecute(mtc::Task& task,
                       int plan_attempts = 5,
                       const ShouldCancelFn& should_cancel = nullptr) const;

  std::map<std::string, double> jointsFromDegrees(const std::vector<double>& angles_deg) const;
  std::map<std::string, double> jointsFromRadians(const std::vector<double>& angles_rad) const;
  static const std::vector<std::string>& defaultJointNames();
  static const std::string& defaultArmGroupName();

  void configureOmplParameters() const;

  std::shared_ptr<mtc::solvers::PipelinePlanner> makePipelinePlanner(
    const std::string& pipeline_id = PipelinePlannerDefaults::pipeline_id,
    double vel_scale = PipelinePlannerDefaults::vel_scale,
    double acc_scale = PipelinePlannerDefaults::acc_scale) const;

  std::shared_ptr<mtc::solvers::CartesianPath> makeCartesianPlanner(
    double vel_scale = CartesianPlannerDefaults::vel_scale,
    double acc_scale = CartesianPlannerDefaults::acc_scale,
    double step = CartesianPlannerDefaults::step,
    double min_fraction = CartesianPlannerDefaults::min_fraction) const;

  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> makeJointInterpolationPlanner(
    double vel_scale = JointInterpolationPlannerDefaults::vel_scale,
    double acc_scale = JointInterpolationPlannerDefaults::acc_scale) const;

private:
  rclcpp::Node::SharedPtr node_;
  nlohmann::json config_;
};
