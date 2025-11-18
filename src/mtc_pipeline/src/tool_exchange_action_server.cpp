#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/tool_exchange_stages.hpp"
#include "mtc_pipeline/action/tool_exchange_action.hpp"

class ToolExchangeActionServer : public BaseActionServer<mtc_pipeline::action::ToolExchangeAction, ToolExchangeStages>
{
public:
    ToolExchangeActionServer() : BaseActionServer("tool_exchange_action_server", "tool_exchange_action") {}
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
