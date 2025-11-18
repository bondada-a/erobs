#include "mtc_pipeline/pipettor_stages.hpp"
#include "mtc_pipeline/pipettor_operation_stage.hpp"

#include <iomanip>
#include <sstream>

PipettorStages::PipettorStages(const rclcpp::Node::SharedPtr& node)
  : BaseStages(node)
{
  // No longer need action client here—PipettorOperationStage handles it
}

bool PipettorStages::run(const mtc_pipeline::action::PipettorAction::Goal& goal)
{
  // Extract pipettor parameters from goal
  const std::string& operation = goal.operation;
  const double volume_pct = goal.volume_pct;
  const std_msgs::msg::ColorRGBA& led_color = goal.led_color;

  // Create descriptive stage name for RViz display
  const std::string stage_name = format_operation_name(operation, volume_pct, led_color);
  RCLCPP_INFO(node()->get_logger(), "Creating pipettor MTC stage: %s", stage_name.c_str());

  // Create MTC task with pipettor operation stage
  auto task = create_task_template("Pipettor Task");

  // Create custom pipettor stage
  auto pipettor_stage = std::make_unique<PipettorOperationStage>(stage_name, node());
  pipettor_stage->setOperation(operation);
  pipettor_stage->setVolumePct(volume_pct);
  pipettor_stage->setLedColor(led_color);

  // Add stage to task
  task.add(std::move(pipettor_stage));

  // Plan and execute the task (this will show in RViz MTC panel)
  return load_plan_execute(task);
}

std::string PipettorStages::format_operation_name(
  const std::string& operation,
  double volume_pct,
  const std_msgs::msg::ColorRGBA& led_color) const
{
  std::ostringstream oss;
  oss << operation;

  if (operation == "SUCK" || operation == "EXPEL") {
    oss << " " << std::fixed << std::setprecision(0) << (volume_pct * 100.0) << "%";
  } else if (operation == "SET_LED") {
    // Format color as "(R,G,B)"
    oss << " ("
        << std::fixed << std::setprecision(0)
        << (led_color.r * 255) << ","
        << (led_color.g * 255) << ","
        << (led_color.b * 255) << ")";
  }

  return oss.str();
}

