#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/tool_exchange_stages.hpp"
#include "mtc_pipeline/action/tool_exchange_action.hpp"

class ToolExchangeActionServer : public BaseActionServer<mtc_pipeline::action::ToolExchangeAction, ToolExchangeStages>
{
public:
    ToolExchangeActionServer() : BaseActionServer("toolexchange_action_server", "toolexchange_action") {}

protected:
    nlohmann::json goal_to_step(const mtc_pipeline::action::ToolExchangeAction::Goal& goal) override
    {
        nlohmann::json step;
        step["operation"] = goal.operation;
        step["gripper"] = goal.gripper;
        step["dock_number"] = goal.dock_number;

        // Add approach pose to step (required by ToolExchangeStages)
        std::string approach_pose_key = goal.approach_pose.empty() ? "dock_approach" : goal.approach_pose;
        step["poses"] = {approach_pose_key};

        return step;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ToolExchangeActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
