#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/pipettor_stages.hpp"
#include "mtc_pipeline/action/pipettor_action.hpp"

class PipettorActionServer : public BaseActionServer<mtc_pipeline::action::PipettorAction, PipettorStages>
{
public:
    PipettorActionServer() : BaseActionServer("pipettor_action_server", "pipettor_action") {}
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PipettorActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
