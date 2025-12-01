#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/move_to_stages.hpp"
#include "mtc_pipeline/action/move_to_action.hpp"

class MoveToActionServer : public BaseActionServer<mtc_pipeline::action::MoveToAction, MoveToStages>
{
public:
    MoveToActionServer() : BaseActionServer("move_to_action_server", "move_to_action") {}
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
