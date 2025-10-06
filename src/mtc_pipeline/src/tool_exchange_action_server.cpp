#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/tool_exchange_stages.hpp"
#include "mtc_pipeline/action/tool_exchange_action.hpp"

class ToolExchangeActionServer : public BaseActionServer<mtc_pipeline::action::ToolExchangeAction, ToolExchangeStages>
{
public:
    ToolExchangeActionServer() : BaseActionServer("tool_exchange_action_server", "tool_exchange_action") {}

protected:
    nlohmann::json goal_to_step(const mtc_pipeline::action::ToolExchangeAction::Goal& goal) override
    {
        nlohmann::json step;
        step["operation"] = goal.operation;
        step["gripper"] = goal.gripper;
        step["dock_number"] = goal.dock_number;
        step["approach_pose"] = goal.approach_pose;

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
