#include "mtc_pipeline/pipettor_stages.hpp"
#include "mtc_pipeline/pipettor_operation_stage.hpp"
#include <iomanip>
#include <sstream>

PipettorStages::PipettorStages(const rclcpp::Node::SharedPtr& node)
    : BaseStages(node) {}

bool PipettorStages::run(const mtc_pipeline::action::PipettorAction::Goal& goal)
{
    // Format descriptive stage name for RViz
    std::ostringstream name;
    name << goal.operation;
    if (goal.operation == "SUCK" || goal.operation == "EXPEL") {
        name << " " << std::fixed << std::setprecision(0) << (goal.volume_pct * 100.0) << "%";
    } else if (goal.operation == "SET_LED") {
        name << " (" << int(goal.led_color.r * 255) << ","
             << int(goal.led_color.g * 255) << ","
             << int(goal.led_color.b * 255) << ")";
    }

    RCLCPP_INFO(node()->get_logger(), "Pipettor: %s", name.str().c_str());

    auto task = create_task_template("Pipettor Task");
    auto stage = std::make_unique<PipettorOperationStage>(name.str(), node());
    stage->setOperation(goal.operation);
    stage->setVolumePct(goal.volume_pct);
    stage->setLedColor(goal.led_color);
    task.add(std::move(stage));

    return load_plan_execute(task);
}
